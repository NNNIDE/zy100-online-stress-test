from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable


LogCallback = Callable[[str], None]


class HostCiState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    ACTIVE_IDLE = "ACTIVE_IDLE"
    CAPTURE_RUNTIME = "CAPTURE_RUNTIME"
    ONLINE_HIGH = "ONLINE_HIGH"
    STANDBY = "STANDBY"
    OTHER = "OTHER"


@dataclass(frozen=True, slots=True)
class HostCiTarget:
    ci_units: int
    interval_ms: float
    latency: int


CI_TARGETS: dict[HostCiState, HostCiTarget] = {
    HostCiState.ACTIVE_IDLE: HostCiTarget(48, 60.0, 0),
    HostCiState.CAPTURE_RUNTIME: HostCiTarget(12, 15.0, 0),
    HostCiState.ONLINE_HIGH: HostCiTarget(9, 11.25, 0),
    HostCiState.STANDBY: HostCiTarget(160, 200.0, 5),
}

MISMATCH_LOG_PERIOD_SECONDS = 5.0


class HostCiStateMachine:
    """Mirrors firmware CI intent and independently observes the WinRT link."""

    def __init__(self, log_callback: LogCallback) -> None:
        self._log = log_callback
        self.desired = HostCiState.DISCONNECTED
        self.actual = HostCiState.DISCONNECTED
        self.transition_id = 0
        self.transition_started = time.monotonic()
        self._last_actual: tuple[HostCiState, str, int | None] | None = None
        self._last_observation_log = 0.0

    def transition(
        self,
        target: HostCiState,
        *,
        reason: str,
        snapshot: Any | None = None,
    ) -> None:
        old = self.desired
        if target != old:
            self.transition_id += 1
            self.transition_started = time.monotonic()
            self.desired = target
            target_params = CI_TARGETS.get(target)
            target_text = (
                f"target_ci={target_params.ci_units} "
                f"target_ms={target_params.interval_ms:g} "
                f"target_latency={target_params.latency}"
                if target_params is not None
                else "target_ci=none"
            )
            self._log(
                "[HOST_CI_SM] "
                f"transition id={self.transition_id} from={old.value} "
                f"to={target.value} reason={reason} {target_text} "
                "host_request=none firmware_owns_exact_ci=1"
            )
        if snapshot is not None:
            self.observe(snapshot, source=reason, force=True)

    def observe(self, snapshot: Any, *, source: str, force: bool = False) -> HostCiState:
        ci_text = str(getattr(snapshot, "ci_effective_ms", "unknown"))
        latency = getattr(snapshot, "conn_latency", None)
        try:
            ci_ms = float(ci_text)
        except (TypeError, ValueError):
            ci_ms = None
        actual = self.classify(ci_ms, latency)
        self.actual = actual
        observation = (actual, ci_text, latency)
        target = CI_TARGETS.get(self.desired)
        matched = (
            target is not None
            and ci_ms is not None
            and abs(ci_ms - target.interval_ms) <= 0.26
            and latency == target.latency
        )
        now = time.monotonic()
        mismatch_log_due = (
            not matched
            and (now - self._last_observation_log) >= MISMATCH_LOG_PERIOD_SECONDS
        )
        if force or observation != self._last_actual or mismatch_log_due:
            elapsed_ms = int((now - self.transition_started) * 1000)
            self._log(
                "[HOST_CI_SM] "
                f"observe id={self.transition_id} desired={self.desired.value} "
                f"actual={actual.value} ci_ms={ci_text} latency={latency} "
                f"match={1 if matched else 0} elapsed_ms={elapsed_ms} source={source}"
            )
            self._last_observation_log = now
        self._last_actual = observation
        return actual

    def disconnected(self, *, reason: str) -> None:
        self.transition(HostCiState.DISCONNECTED, reason=reason)
        self.actual = HostCiState.DISCONNECTED
        self._last_actual = None
        self._last_observation_log = 0.0

    @staticmethod
    def classify(ci_ms: float | None, latency: int | None) -> HostCiState:
        if ci_ms is None:
            return HostCiState.OTHER
        for state, target in CI_TARGETS.items():
            if abs(ci_ms - target.interval_ms) <= 0.26 and latency == target.latency:
                return state
        return HostCiState.OTHER
