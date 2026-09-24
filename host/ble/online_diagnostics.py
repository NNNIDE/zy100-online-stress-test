from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


ONLINE_DIAG_INTERVAL_NS = 5_000_000_000
ONLINE_DIAG_HEARTBEAT_SECONDS = 0.1


@dataclass(slots=True)
class _MaxMetric:
    count: int = 0
    total_ms: float = 0.0
    max_ms: float = 0.0
    bins: list[int] = field(default_factory=lambda: [0] * 15)
    BOUNDS = (0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, float("inf"))

    def note(self, value_ms: float) -> None:
        value = max(0.0, float(value_ms))
        self.count += 1
        self.total_ms += value
        self.max_ms = max(self.max_ms, value)
        for index, bound in enumerate(self.BOUNDS):
            if value <= bound:
                self.bins[index] += 1
                break

    def percentile_bound(self, fraction: float) -> float:
        target = max(1, int(self.count * fraction + 0.999999))
        total = 0
        for bound, count in zip(self.BOUNDS, self.bins):
            total += count
            if total >= target:
                return bound
        return 0.0

    @property
    def average_ms(self) -> float:
        return self.total_ms / self.count if self.count else 0.0

    def reset(self) -> None:
        self.count = 0
        self.total_ms = 0.0
        self.max_ms = 0.0
        self.bins[:] = [0] * len(self.BOUNDS)


class OnlineHostDiagnostics:
    """Low-frequency, in-memory diagnostics for the Online BLE hot path."""

    def __init__(self) -> None:
        self.active = False
        self.session_id = 0
        self.started_ns = 0
        self.next_log_ns = 0
        self.last_callback_ns = 0
        self.last_sequence = 0
        self.last_frame_type = 0
        self.last_record_id = 0
        self.last_fragment_offset = 0
        self.callback_count = 0
        self.callback_bytes = 0
        self.queue_depth_max = 0
        self.ack_fallback_count = 0
        self._final_logged = False
        self.notify_gap = _MaxMetric()
        self.queue_wait = _MaxMetric()
        self.parse = _MaxMetric()
        self.save = _MaxMetric()
        self._worst_save_profile: dict[str, Any] | None = None
        self.ack_write = _MaxMetric()
        self.ack_metadata = _MaxMetric()
        self.event_callback = _MaxMetric()
        self.log_callback = _MaxMetric()
        self.ui_dispatch = _MaxMetric()
        self.loop_slip = _MaxMetric()
        self.storage_queue = _MaxMetric()
        self.storage_work = _MaxMetric()
        self.saved_to_ack = _MaxMetric()
        self.ui_work = _MaxMetric()
        self.session_save_time = "NA"
        self.session_metrics = {name: _MaxMetric() for name in
                                ("save", "loop", "queue", "ui", "storage_queue", "storage_work", "saved_to_ack", "ui_work")}
        self.session_queue_max = 0
        self.session_save_profile: dict[str, Any] | None = None

    def start(self, session_id: int, now_ns: int | None = None) -> None:
        self.__init__()
        now = time.perf_counter_ns() if now_ns is None else int(now_ns)
        self.active = True
        self.session_id = int(session_id)
        self.started_ns = now
        self.next_log_ns = now + ONLINE_DIAG_INTERVAL_NS
        self.last_callback_ns = now

    def note_callback(self, payload_len: int, received_ns: int, queue_depth: int) -> None:
        if not self.active:
            return
        now = int(received_ns)
        if self.last_callback_ns:
            self.notify_gap.note((now - self.last_callback_ns) / 1_000_000.0)
        self.last_callback_ns = now
        self.callback_count += 1
        self.callback_bytes += max(0, int(payload_len))
        self.queue_depth_max = max(self.queue_depth_max, max(0, int(queue_depth)))
        self.session_queue_max = max(self.session_queue_max, max(0, int(queue_depth)))

    def note_frame(self, frame_type: int, sequence: int) -> None:
        if not self.active:
            return
        self.last_frame_type = int(frame_type)
        self.last_sequence = int(sequence)

    def note_fragment(self, record_id: int, offset: int) -> None:
        if not self.active:
            return
        self.last_record_id = int(record_id)
        self.last_fragment_offset = int(offset)

    def note_queue_wait(self, value_ms: float) -> None:
        if self.active:
            self.queue_wait.note(value_ms)
            self.session_metrics["queue"].note(value_ms)

    def note_parse(self, value_ms: float) -> None:
        if self.active:
            self.parse.note(value_ms)

    def note_save(self, value_ms: float, *, profile: dict[str, Any] | None = None) -> None:
        if self.active:
            if self.save.count == 0 or value_ms >= self.save.max_ms:
                self._worst_save_profile = (
                    {**profile, "stages": dict(profile.get("stages", {}))}
                    if profile is not None else None
                )
            session_save = self.session_metrics["save"]
            if session_save.count == 0 or value_ms >= session_save.max_ms:
                self.session_save_profile = self._worst_save_profile
                self.session_save_time = time.strftime("%H:%M:%S")
            session_save.note(value_ms)
            self.save.note(value_ms)

    def note_ui_work(self, value_ms: float) -> None:
        if self.active:
            self.ui_work.note(value_ms)
            self.session_metrics["ui_work"].note(value_ms)

    def note_storage(self, queue_ms: float, work_ms: float) -> None:
        if self.active:
            self.storage_queue.note(queue_ms)
            self.storage_work.note(work_ms)
            self.session_metrics["storage_queue"].note(queue_ms)
            self.session_metrics["storage_work"].note(work_ms)

    def note_saved_to_ack(self, value_ms: float) -> None:
        if self.active:
            self.saved_to_ack.note(value_ms)
            self.session_metrics["saved_to_ack"].note(value_ms)

    def note_ack_write(self, value_ms: float, *, fallback: bool) -> None:
        if not self.active:
            return
        self.ack_write.note(value_ms)
        if fallback:
            self.ack_fallback_count += 1

    def note_ack_metadata(self, value_ms: float) -> None:
        if self.active:
            self.ack_metadata.note(value_ms)

    def note_event_callback(self, value_ms: float) -> None:
        if self.active:
            self.event_callback.note(value_ms)

    def note_log_callback(self, value_ms: float) -> None:
        if self.active:
            self.log_callback.note(value_ms)

    def note_ui_dispatch(self, value_ms: float) -> None:
        if self.active:
            self.ui_dispatch.note(value_ms)
            self.session_metrics["ui"].note(value_ms)

    def note_loop_slip(self, value_ms: float) -> None:
        if self.active:
            self.loop_slip.note(value_ms)
            self.session_metrics["loop"].note(value_ms)

    def periodic_line(self, *, now_ns: int, queue_depth: int) -> str | None:
        if not self.active or now_ns < self.next_log_ns:
            return None
        line = self._line("[ONLINE_HOST_STAT]", "run", queue_depth)
        self.next_log_ns = int(now_ns) + ONLINE_DIAG_INTERVAL_NS
        self._reset_interval()
        return line

    def final_line(self, *, reason: str, queue_depth: int) -> str | None:
        if not self.active or self._final_logged:
            return None
        self._final_logged = True
        summary = " ".join(
            f"all_{name}={metric.percentile_bound(.95):g}/{metric.percentile_bound(.99):g}/{metric.max_ms:.1f}"
            for name, metric in self.session_metrics.items()
        )
        profile = self.session_save_profile or {}
        return (self._line("[ONLINE_HOST_FINAL]", reason, queue_depth)
                + f" all_qmax={self.session_queue_max} all_save_rid={profile.get('record_id', 'NA')} "
                + f"all_save_time={self.session_save_time} "
                + "all_save_steps=" + ",".join(f"{k}:{v:.1f}" for k, v in profile.get("stages", {}).items())
                + f" percentiles=upper_bound_ms {summary}")

    def stop(self) -> None:
        self.active = False

    def _save_detail(self) -> str:
        profile = self._worst_save_profile
        if profile is None:
            return "save_rid=NA save_cpu=NA save_steps=NA"
        steps = dict(profile["stages"])
        steps["other"] = max(0.0, profile["wall_ms"] - sum(steps.values()))
        slowest = max(steps, key=steps.get)
        details = ",".join(f"{name}:{value:.1f}" for name, value in steps.items())
        return (
            f"save_rid={profile['record_id']} save_ok={int(profile['ok'])} "
            f"save_wall={profile['wall_ms']:.1f} save_cpu={profile['cpu_ms']:.1f} "
            f"save_slow={slowest} save_steps={details}"
        )

    def _line(self, prefix: str, reason: str, queue_depth: int) -> str:
        return (
            f"{prefix} sid={self.session_id} why={reason} "
            f"cb={self.callback_count} bytes={self.callback_bytes} "
            f"seq={self.last_sequence} ft=0x{self.last_frame_type:02X} "
            f"rid={self.last_record_id} off={self.last_fragment_offset} "
            f"gap={self.notify_gap.average_ms:.1f}/{self.notify_gap.max_ms:.1f} "
            f"q={max(0, int(queue_depth))}/{self.queue_depth_max}/{self.queue_wait.max_ms:.1f} "
            f"parse={self.parse.max_ms:.1f} save={self.save.max_ms:.1f} "
            f"ack={self.ack_write.max_ms:.1f}/{self.ack_metadata.max_ms:.1f}/{self.ack_fallback_count} "
            f"ui={self.event_callback.max_ms:.1f}/{self.ui_dispatch.max_ms:.1f} "
            f"log={self.log_callback.max_ms:.1f} loop={self.loop_slip.max_ms:.1f} "
            f"storage={self.storage_queue.max_ms:.1f}/{self.storage_work.max_ms:.1f} "
            f"saved_to_ack={self.saved_to_ack.max_ms:.1f} ui_work={self.ui_work.max_ms:.1f} {self._save_detail()}"
        )

    def _reset_interval(self) -> None:
        self._worst_save_profile = None
        self.callback_count = 0
        self.callback_bytes = 0
        self.queue_depth_max = 0
        self.ack_fallback_count = 0
        for metric in (
            self.notify_gap,
            self.queue_wait,
            self.parse,
            self.save,
            self.ack_write,
            self.ack_metadata,
            self.event_callback,
            self.log_callback,
            self.ui_dispatch,
            self.loop_slip,
            self.storage_queue,
            self.storage_work,
            self.saved_to_ack,
            self.ui_work,
        ):
            metric.reset()
