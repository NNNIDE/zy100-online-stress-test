from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any


TRACE_ENV = "ZY100_BLE_LINK_TRACE"
TRACE_DIR_ENV = "ZY100_BLE_LINK_TRACE_DIR"
TRACE_CAPACITY = 512


def configure_bleak_debug_logging(enabled: bool, output_dir: str | Path) -> Path | None:
    """Enable bounded Bleak DEBUG logging only for an explicit diagnostic run."""
    if not enabled:
        return None
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / "bleak_winrt_debug.log"
    handler = RotatingFileHandler(
        log_path, maxBytes=4 * 1024 * 1024, backupCount=3, encoding="utf-8"
    )
    handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)s %(name)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    ))
    logger = logging.getLogger("bleak")
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.propagate = False
    return log_path


class HostBleLinkTrace:
    """Bounded, opt-in host flight recorder; it never changes BLE traffic."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        output_dir: str | Path = "diagnostics",
        capacity: int = TRACE_CAPACITY,
    ) -> None:
        self.enabled = bool(enabled)
        self.output_dir = Path(output_dir)
        self._records: deque[dict[str, Any]] = deque(maxlen=capacity)
        self._pending_ping: dict[int, int] = {}
        self._lock = threading.Lock()
        self._generation = 0
        self._address = ""
        self._connected_ns: int | None = None

    @classmethod
    def from_environment(cls) -> HostBleLinkTrace:
        enabled = os.getenv(TRACE_ENV, "").strip().lower() in {"1", "true", "yes", "on"}
        return cls(
            enabled=enabled,
            output_dir=os.getenv(TRACE_DIR_ENV, "diagnostics") or "diagnostics",
        )

    @property
    def generation(self) -> int:
        return self._generation

    @staticmethod
    def _safe_value(value: Any) -> Any:
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, bytes):
            return value.hex()
        if isinstance(value, dict):
            return {str(key): HostBleLinkTrace._safe_value(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [HostBleLinkTrace._safe_value(item) for item in value]
        return repr(value)

    def record(self, event: str, **fields: Any) -> None:
        if not self.enabled:
            return
        now_ns = time.monotonic_ns()
        record: dict[str, Any] = {
            "wall_time_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "monotonic_ns": now_ns,
            "event": event,
            "generation": self._generation,
            "address": self._address,
        }
        record.update({key: self._safe_value(value) for key, value in fields.items()})
        with self._lock:
            self._records.append(record)

    def begin_connection(self, generation: int, address: str) -> None:
        if not self.enabled:
            return
        self._generation = int(generation)
        self._address = address
        self._connected_ns = None
        with self._lock:
            self._pending_ping.clear()
        self.record("connection_begin")

    def link_connected(self, **fields: Any) -> None:
        if not self.enabled:
            return
        self._connected_ns = time.monotonic_ns()
        self.record("link_connected", **fields)

    def note_ping_sent(self, seq: int, **fields: Any) -> None:
        if not self.enabled:
            return
        now_ns = time.monotonic_ns()
        with self._lock:
            self._pending_ping[int(seq) & 0xFF] = now_ns
        self.record("ping_sent", seq=int(seq) & 0xFF, **fields)

    def note_ping_ack(self, seq: int, **fields: Any) -> None:
        if not self.enabled:
            return
        now_ns = time.monotonic_ns()
        with self._lock:
            started_ns = self._pending_ping.pop(int(seq) & 0xFF, None)
        rtt_ms = None if started_ns is None else (now_ns - started_ns) / 1_000_000.0
        self.record("ping_ack", seq=int(seq) & 0xFF, ack_rtt_ms=rtt_ms, **fields)

    def record_exception(self, event: str, exc: BaseException, **fields: Any) -> None:
        self.record(
            event,
            exception_type=type(exc).__name__,
            exception=repr(exc),
            winerror=getattr(exc, "winerror", None),
            hresult=getattr(exc, "hresult", getattr(exc, "HRESULT", None)),
            **fields,
        )

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(record) for record in self._records]

    async def dump(self, reason: str, **fields: Any) -> Path | None:
        if not self.enabled:
            return None
        self.record("trace_dump", reason=reason, **fields)
        records = self.snapshot()
        generation = self._generation
        output_dir = self.output_dir

        def write_jsonl() -> Path:
            output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = output_dir / f"ble_link_trace_g{generation}_{stamp}.jsonl"
            with path.open("w", encoding="utf-8", newline="\n") as stream:
                for record in records:
                    stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            return path

        return await asyncio.to_thread(write_jsonl)
