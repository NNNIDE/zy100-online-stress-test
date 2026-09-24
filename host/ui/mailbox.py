from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any


class UiMailbox:
    """Producers never call Tk. Only the UI thread drains this bounded mailbox."""
    MAX_EVENTS = 4096
    MAX_LOGS = 8192
    PROGRESS_INTERVAL_NS = 200_000_000
    PROGRESS_STATUSES = {"receiving", "record", "ack_sent"}

    def __init__(self):
        self._lock = threading.Lock()
        self._events = deque()
        self._logs = deque()
        self._progress = None
        self._next_progress_ns = 0
        self._closed = False
        self._event_overflow = 0
        self._logs_dropped = 0

    def post(self, kind: str, payload: Any) -> None:
        item = (kind, payload, time.perf_counter_ns())
        with self._lock:
            if self._closed:
                return
            if kind in {"log", "ota_log"}:
                if len(self._logs) >= self.MAX_LOGS:
                    self._logs.popleft()
                    self._logs_dropped += 1
                self._logs.append(item)
                return
            progress = (kind == "event" and payload.get("type") == "online_stream_status"
                        and payload.get("status") in self.PROGRESS_STATUSES)
            if progress:
                self._progress = item
                return
            # A critical transition is a barrier; do not paint preceding progress
            # after an END/error/disconnect or a subsequent session's START.
            self._progress = None
            if len(self._events) >= self.MAX_EVENTS:
                self._event_overflow += 1
                return
            self._events.append(item)

    def take(self, now_ns: int | None = None):
        now = time.perf_counter_ns() if now_ns is None else now_ns
        with self._lock:
            events = [self._events.popleft() for _ in range(min(32, len(self._events)))]
            if not self._events and self._progress is not None and now >= self._next_progress_ns:
                events.append(self._progress)
                self._progress = None
                self._next_progress_ns = now + self.PROGRESS_INTERVAL_NS
            logs = [self._logs.popleft() for _ in range(min(64, len(self._logs)))]
            overflow, dropped = self._event_overflow, self._logs_dropped
            self._event_overflow = self._logs_dropped = 0
        return events + logs, overflow, dropped

    def close(self):
        with self._lock:
            self._closed = True
            self._events.clear()
            self._logs.clear()
            self._progress = None
