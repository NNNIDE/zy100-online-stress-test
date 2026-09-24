from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Any

from .export_speed import export_ci_mode, export_target_ci_ms, format_ci_ms


EXPORT_NOTIFY_QUEUE_MAX = 2048


@dataclass(frozen=True, slots=True)
class QueuedExportNotification:
    uuid: str
    payload: bytes
    received_perf_ns: int
    received_wall_ms: int


class _Metric:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.total = 0.0
        self.min_value: float | None = None
        self.max_value: float | None = None

    def add(self, value: float) -> None:
        self.count += 1
        self.total += value
        self.min_value = value if self.min_value is None else min(self.min_value, value)
        self.max_value = value if self.max_value is None else max(self.max_value, value)

    @property
    def avg(self) -> float:
        return self.total / self.count if self.count else 0.0

    @property
    def min(self) -> float:
        return self.min_value if self.min_value is not None else 0.0

    @property
    def max(self) -> float:
        return self.max_value if self.max_value is not None else 0.0


class ExportPerfStats:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.export_start_ms: int | None = None
        self.export_start_perf_ns: int | None = None
        self.first_notify_ms: int | None = None
        self.first_notify_perf_ns: int | None = None
        self.first_data_notify_ms: int | None = None
        self.first_data_notify_perf_ns: int | None = None
        self.end_notify_ms: int | None = None
        self.end_notify_perf_ns: int | None = None
        self.confirm_send_ms: int | None = None
        self.confirm_send_perf_ns: int | None = None
        self._last_notify_perf_ns: int | None = None

        self.total_notify_count = 0
        self.data_notify_count = 0
        self.notify_bytes = 0
        self.app_payload_bytes = 0
        self.queue_depth_max = 0
        self.queue_overflow_count = 0
        self.parse_error_count = 0
        self.crc_ok: bool | None = None
        self.crc_calc_us = 0.0
        self.file_write_ms = 0.0
        self.summary_logged = False
        self.overflow_reported = False
        self.feuf_payload_max: int | None = None
        self.feuf_data_frame_count = 0
        self.feuf_data_payload_bytes = 0
        self.feuf_data_payload_min = 0
        self.feuf_data_payload_max = 0
        self.feuf_data_le64 = 0
        self.feuf_data_gt192 = 0

        self.notify_gap_ms = _Metric()
        self.callback_cost_us = _Metric()

        self.mtu_requested = "platform_default"
        self.backend = "unknown"
        self.mtu_effective = "unknown"
        self.phy_requested = "unsupported"
        self.phy_effective = "unknown"
        self.conn_param_requested = "none"
        self.conn_param_effective = "unknown"
        self.conn_param_control = "unsupported"
        self.ci_mode = export_ci_mode()
        self.ci_target_ms = format_ci_ms(export_target_ci_ms())
        self.ci_raw = "unknown"
        self.ci_raw_type = "unknown"
        self.ci_effective_ms = "unknown"
        self.ci_target_reached = "unknown"
        self.capability_level = "unknown"

    def prepare_for_new_notification(self) -> None:
        if self.summary_logged:
            self.reset()

    def note_callback(
        self,
        *,
        payload_len: int,
        received_perf_ns: int,
        received_wall_ms: int,
        callback_cost_us: float,
        queue_depth: int,
    ) -> None:
        self.prepare_for_new_notification()
        if self.first_notify_ms is None:
            self.first_notify_ms = received_wall_ms
            self.first_notify_perf_ns = received_perf_ns
        if self._last_notify_perf_ns is not None:
            self.notify_gap_ms.add((received_perf_ns - self._last_notify_perf_ns) / 1_000_000.0)
        self._last_notify_perf_ns = received_perf_ns
        self.total_notify_count += 1
        self.notify_bytes += payload_len
        self.callback_cost_us.add(callback_cost_us)
        self.queue_depth_max = max(self.queue_depth_max, queue_depth)

    def note_overflow(self) -> None:
        self.queue_overflow_count += 1

    def note_event(self, event: Any, item: QueuedExportNotification, *, parser_cost_us: float) -> None:
        kind = str(getattr(event, "kind", ""))
        if kind == "start":
            self.export_start_ms = item.received_wall_ms
            self.export_start_perf_ns = item.received_perf_ns
        elif kind == "data":
            if self.first_data_notify_ms is None:
                self.first_data_notify_ms = item.received_wall_ms
                self.first_data_notify_perf_ns = item.received_perf_ns
            self.data_notify_count += 1
            self.app_payload_bytes = max(self.app_payload_bytes, int(getattr(event, "bytes_received", 0) or 0))
            self.crc_calc_us += parser_cost_us
        elif kind == "end":
            self.end_notify_ms = item.received_wall_ms
            self.end_notify_perf_ns = item.received_perf_ns
            self.app_payload_bytes = max(self.app_payload_bytes, int(getattr(event, "bytes_received", 0) or 0))
            self.crc_ok = bool(getattr(event, "ok", False))

    def note_parser_error(self) -> None:
        self.parse_error_count += 1

    def note_confirm_send(self) -> None:
        self.confirm_send_ms = int(time.time() * 1000)
        self.confirm_send_perf_ns = time.perf_counter_ns()

    def add_file_write_ms(self, value: float) -> None:
        self.file_write_ms += value

    def note_feuf_stats(self, *, payload_max: int | None, data_payload_lengths: list[int]) -> None:
        self.feuf_payload_max = payload_max
        self.feuf_data_frame_count = len(data_payload_lengths)
        self.feuf_data_payload_bytes = sum(data_payload_lengths)
        self.feuf_data_payload_min = min(data_payload_lengths) if data_payload_lengths else 0
        self.feuf_data_payload_max = max(data_payload_lengths) if data_payload_lengths else 0
        self.feuf_data_le64 = sum(1 for length in data_payload_lengths if length <= 64)
        self.feuf_data_gt192 = sum(1 for length in data_payload_lengths if length > 192)

    def update_ble_snapshot(self, snapshot: Any) -> None:
        for name in (
            "backend",
            "mtu_requested",
            "mtu_effective",
            "phy_requested",
            "phy_effective",
            "conn_param_requested",
            "conn_param_effective",
            "conn_param_control",
            "ci_mode",
            "ci_target_ms",
            "ci_raw",
            "ci_raw_type",
            "ci_effective_ms",
            "ci_target_reached",
            "capability_level",
        ):
            value = getattr(snapshot, name, None)
            if value is not None:
                setattr(self, name, str(value))

    def summary_lines(self, *, result: str) -> list[str]:
        if self.summary_logged:
            return []
        self.summary_logged = True

        total_ms = self._elapsed_ms(self.export_start_perf_ns, self.confirm_send_perf_ns or self.end_notify_perf_ns)
        app_ms = self._elapsed_ms(self.first_data_notify_perf_ns, self.end_notify_perf_ns)
        notify_ms = self._elapsed_ms(self.first_notify_perf_ns, self.end_notify_perf_ns)
        app_bps = self._bps(self.app_payload_bytes, app_ms)
        notify_bps = self._bps(self.notify_bytes, notify_ms)
        crc_ms = self.crc_calc_us / 1000.0
        crc_ok = 1 if self.crc_ok else 0
        feuf_payload_max = "unknown" if self.feuf_payload_max is None else str(self.feuf_payload_max)
        feuf_payload_fit_ble = 1 if self.feuf_payload_max == 200 else 0
        feuf_data_payload_avg = (
            self.feuf_data_payload_bytes / self.feuf_data_frame_count
            if self.feuf_data_frame_count
            else 0.0
        )
        capability_level = self._capability_level()

        return [
            f"[HOST_EXPORT_PERF] result={result} bytes={self.app_payload_bytes}",
            (
                "[HOST_EXPORT_PERF] "
                f"notify_count={self.total_notify_count} data_notify_count={self.data_notify_count} "
                f"notify_bytes={self.notify_bytes}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"time total_ms={total_ms:.1f} export_start_ms={self.export_start_ms or 0} "
                f"first_notify_ms={self.first_notify_ms or 0} first_data_notify_ms={self.first_data_notify_ms or 0} "
                f"end_notify_ms={self.end_notify_ms or 0} confirm_send_ms={self.confirm_send_ms or 0}"
            ),
            f"[HOST_EXPORT_PERF] rate app_Bps={app_bps:.1f} notify_Bps={notify_bps:.1f}",
            (
                "[HOST_EXPORT_PERF] "
                f"ci_mode={self.ci_mode} ci_target_ms={self.ci_target_ms} "
                f"ci_effective_ms={self.ci_effective_ms} ci_target_reached={self.ci_target_reached}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"gap_ms min={self.notify_gap_ms.min:.3f} max={self.notify_gap_ms.max:.3f} "
                f"avg={self.notify_gap_ms.avg:.3f}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"callback_cost_us min={self.callback_cost_us.min:.1f} max={self.callback_cost_us.max:.1f} "
                f"avg={self.callback_cost_us.avg:.1f}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"queue max={self.queue_depth_max} overflow={self.queue_overflow_count}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"parse_error_count={self.parse_error_count} crc ok={crc_ok} "
                f"crc_ms={crc_ms:.3f} file_write_ms={self.file_write_ms:.3f}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"feuf_payload_max={feuf_payload_max} feuf_payload_fit_ble={feuf_payload_fit_ble} "
                f"feuf_data_frame_count={self.feuf_data_frame_count} "
                f"feuf_data_payload_bytes={self.feuf_data_payload_bytes} "
                f"feuf_data_payload_avg={feuf_data_payload_avg:.1f} "
                f"feuf_data_payload_min={self.feuf_data_payload_min} "
                f"feuf_data_payload_max={self.feuf_data_payload_max} "
                f"feuf_data_le64={self.feuf_data_le64} feuf_data_gt192={self.feuf_data_gt192}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"conn ci={self.conn_param_effective} mtu={self.mtu_effective} phy={self.phy_effective} "
                f"mtu_requested={self.mtu_requested} phy_requested={self.phy_requested} "
                f"conn_param_requested={self.conn_param_requested} conn_param_control={self.conn_param_control}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"host_ble backend={self.backend} ci_control={self.conn_param_control} "
                f"ci_effective_ms={self.ci_effective_ms} runtime_target_reached={self.ci_target_reached} "
                f"mtu_effective={self.mtu_effective} phy_effective={self.phy_effective}"
            ),
            (
                "[HOST_EXPORT_PERF] "
                f"host_ble callback_avg_us={self.callback_cost_us.avg:.1f} "
                f"queue_overflow={self.queue_overflow_count} capability_level={capability_level}"
            ),
        ]

    @staticmethod
    def _elapsed_ms(start_ns: int | None, end_ns: int | None) -> float:
        if start_ns is None or end_ns is None or end_ns < start_ns:
            return 0.0
        return (end_ns - start_ns) / 1_000_000.0

    @staticmethod
    def _bps(byte_count: int, elapsed_ms: float) -> float:
        if elapsed_ms <= 0:
            return 0.0
        return byte_count * 1000.0 / elapsed_ms

    def _capability_level(self) -> str:
        if self.capability_level != "unknown":
            return self.capability_level
        if self.queue_overflow_count > 0 or self.callback_cost_us.avg > 1000.0:
            return "app_pipeline_bottleneck"
        if self.conn_param_control in {"unsupported", "failed"} and self.ci_effective_ms == "unknown":
            return "api_unavailable"
        if self.ci_effective_ms != "unknown" and self.ci_target_reached == "1":
            return "runtime_60ms"
        if self.ci_effective_ms != "unknown" and self.ci_target_reached == "0":
            return "runtime_60ms_not_reached"
        return "unknown"


class ExportNotificationPipeline:
    def __init__(self, *, max_queue: int = EXPORT_NOTIFY_QUEUE_MAX) -> None:
        self.queue: asyncio.Queue[QueuedExportNotification] = asyncio.Queue(maxsize=max_queue)
        self.stats = ExportPerfStats()

    def reset(self) -> None:
        self.stats.reset()
        while True:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except asyncio.QueueEmpty:
                break

    def enqueue(
        self,
        *,
        uuid: str,
        payload: bytes,
        callback_cost_us: float,
        received_perf_ns: int,
        received_wall_ms: int,
    ) -> bool:
        item = QueuedExportNotification(
            uuid=uuid,
            payload=payload,
            received_perf_ns=received_perf_ns,
            received_wall_ms=received_wall_ms,
        )
        try:
            self.queue.put_nowait(item)
        except asyncio.QueueFull:
            self.stats.note_callback(
                payload_len=len(payload),
                received_perf_ns=received_perf_ns,
                received_wall_ms=received_wall_ms,
                callback_cost_us=callback_cost_us,
                queue_depth=self.queue.qsize(),
            )
            self.stats.note_overflow()
            return False

        self.stats.note_callback(
            payload_len=len(payload),
            received_perf_ns=received_perf_ns,
            received_wall_ms=received_wall_ms,
            callback_cost_us=callback_cost_us,
            queue_depth=self.queue.qsize(),
        )
        return True
