from __future__ import annotations

import asyncio
import threading
import time
import weakref
from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .online_store import OnlineSessionStore


@dataclass(frozen=True)
class StoreSnapshot:
    session_dir: Path | None
    metadata: dict[str, Any]
    metadata_dirty: bool
    profile: dict[str, Any] | None

    @classmethod
    def read(cls, store: OnlineSessionStore) -> StoreSnapshot:
        return cls(store.session_dir, deepcopy(store.metadata), store.metadata_dirty,
                   deepcopy(store.last_save_profile))


@dataclass(frozen=True)
class StorageResult:
    value: Any
    error: Exception | None
    queue_ms: float
    work_ms: float
    finished_ns: int


class OnlineStorageWorker:
    """One writer across sessions; callers receive completed immutable snapshots.

    Futures are deliberately shielded from asyncio cancellation. A cancelled BLE
    waiter cannot stop a running OS file operation. Its store remains owned by
    this worker until the queued terminal operation completes.
    """

    MAX_PENDING = 16

    def __init__(self) -> None:
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="online-store")
        self._lock = threading.Lock()
        self._states: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()
        self._pending: set[Future] = set()
        self._closed = False

    def snapshot(self, store: OnlineSessionStore) -> StoreSnapshot:
        with self._lock:
            # Seed before the first operation takes ownership of the store.
            if store not in self._states:
                self._states[store] = StoreSnapshot.read(store)
            return self._states[store]

    def submit(self, store: OnlineSessionStore, method: str, *args, **kwargs) -> Future:
        self.snapshot(store)
        queued_ns = time.perf_counter_ns()
        with self._lock:
            if self._closed:
                raise RuntimeError("online storage is closing")
            # Reserve admission for terminal cleanup; data may never grow the
            # executor's otherwise unbounded internal queue.
            limit = self.MAX_PENDING if method == "abort" else self.MAX_PENDING - 2
            if len(self._pending) >= limit:
                raise RuntimeError("online storage queue is full")
            future = self._executor.submit(self._execute, store, method, args, kwargs, queued_ns)
            self._pending.add(future)
        future.add_done_callback(self._completed)
        return future

    def _completed(self, future: Future) -> None:
        with self._lock:
            self._pending.discard(future)

    def _execute(self, store, method, args, kwargs, queued_ns) -> StorageResult:
        started_ns = time.perf_counter_ns()
        value, error = None, None
        try:
            if method == "save_record":
                store.last_save_profile = None
            if not (method == "abort" and kwargs.get("terminal_source") == "host_disconnect"
                    and store.metadata.get("status") == "ended"):
                value = getattr(store, method)(*args, **kwargs)
        except Exception as exc:
            error = exc
        finally:
            state = StoreSnapshot.read(store)
            with self._lock:
                self._states[store] = state
        finished_ns = time.perf_counter_ns()
        return StorageResult(value, error, (started_ns - queued_ns) / 1e6,
                             (finished_ns - started_ns) / 1e6, finished_ns)

    async def wait(self, future: Future) -> StorageResult:
        return await asyncio.shield(asyncio.wrap_future(future))

    async def drain(self) -> None:
        while True:
            with self._lock:
                pending = tuple(self._pending)
            if not pending:
                return
            await asyncio.gather(*(self.wait(future) for future in pending))

    def close(self) -> None:
        with self._lock:
            self._closed = True
        # Already submitted writes/terminal cleanup must run to completion.
        self._executor.shutdown(wait=False, cancel_futures=False)
