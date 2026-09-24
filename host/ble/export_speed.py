from __future__ import annotations

import asyncio
import binascii
import struct
import sys
from dataclasses import dataclass, field
from datetime import timedelta
from enum import Enum, IntEnum
from typing import Any, Awaitable, Callable


LogCallback = Callable[[str], None]

ENABLE_BLE_EXPORT_ULTRA_FAST_CI_EXPERIMENT = "ENABLE_BLE_EXPORT_ULTRA_FAST_CI_EXPERIMENT"
ENABLE_WINRT_THROUGHPUT_OPTIMIZED = "ENABLE_WINRT_THROUGHPUT_OPTIMIZED"
CI_TARGET_REACHED_TOLERANCE_MS = 0.25
BLE_CI_UNIT_MIN = 0x0006
BLE_CI_UNIT_MAX = 0x0C80
FIXED_CONN_PARAM_CONTROL = "fixed_no_request"
FIXED_CONN_PARAM_REASON = "fixed_connection_parameters"
WINRT_CONN_PARAM_CONTROL = "winrt_throughput_optimized"
WINRT_CONN_PARAM_REQUESTED = "throughput_optimized"
WINRT_THROUGHPUT_TARGET_MS = 15.0
WINRT_THROUGHPUT_WAIT_TIMEOUT_S = 1.2
WINRT_THROUGHPUT_POLL_INTERVAL_S = 0.1
WINDOWS_CENTRAL_APPLY_TIMEOUT_S = 10.0
WINDOWS_CENTRAL_POLL_INTERVAL_S = 0.5
WINDOWS_CENTRAL_PROFILE_SIGNATURE = 0xB56C5520


class WindowsCentralProfile(IntEnum):
    NONE = 0
    BALANCED = 1
    THROUGHPUT_OPTIMIZED = 2
    POWER_OPTIMIZED = 3


@dataclass(frozen=True, slots=True)
class WindowsCentralProfileSpec:
    minimum_interval: int
    maximum_interval: int
    latency: int
    link_timeout: int


@dataclass(frozen=True, slots=True)
class WindowsCentralApplyResult:
    profile: WindowsCentralProfile
    request_status: int
    accepted: bool
    actual_stable: bool
    actual_ci: int | None
    actual_latency: int | None
    reason: str


def export_ci_mode() -> str:
    return "normal_60ms"


def export_target_ci_ms() -> float:
    return 60.0


def format_ci_ms(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


class ExportBleSpeedMode(str, Enum):
    IDLE = "idle"
    FIXED = "fixed"
    REQUESTING = "requesting"
    ACTIVE = "active"
    DEGRADED = "degraded"
    RESTORING = "restoring"
    UNSUPPORTED = "unsupported"
    FAILED = "failed"


@dataclass(slots=True)
class BleSpeedSnapshot:
    mode: ExportBleSpeedMode = ExportBleSpeedMode.IDLE
    backend: str = "unknown"
    mtu_requested: str = "platform_default"
    mtu_effective: str = "unknown"
    phy_requested: str = "unsupported"
    phy_effective: str = "unknown"
    conn_param_requested: str = "none"
    conn_param_effective: str = "unknown"
    conn_param_control: str = "unsupported"
    ci_mode: str = field(default_factory=export_ci_mode)
    ci_target_ms: str = field(default_factory=lambda: format_ci_ms(export_target_ci_ms()))
    ci_raw: str = "unknown"
    ci_raw_type: str = "unknown"
    ci_effective_ms: str = "unknown"
    conn_latency: int | None = None
    ci_target_reached: str = "unknown"
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ConnectionIntervalObservation:
    ci_raw: str
    ci_raw_type: str
    ci_ms: float | None
    source: str = "unknown"


class WinRtExportSpeedAdapter:
    """Read-only WinRT link observer; firmware owns exact CI requests."""

    def __init__(
        self,
        log_callback: LogCallback,
        *,
        settle_timeout_s: float = WINRT_THROUGHPUT_WAIT_TIMEOUT_S,
        poll_interval_s: float = WINRT_THROUGHPUT_POLL_INTERVAL_S,
    ) -> None:
        self._log = log_callback
        self._settle_timeout_s = max(0.0, float(settle_timeout_s))
        self._poll_interval_s = max(0.0, float(poll_interval_s))
        self.mode = ExportBleSpeedMode.IDLE
        self._request: Any | None = None
        self._requester: Any | None = None
        self._logged_default = False
        self._snapshot = BleSpeedSnapshot()

    @property
    def snapshot(self) -> BleSpeedSnapshot:
        return self._snapshot

    async def enter(self, client: Any, *, reason: str) -> BleSpeedSnapshot:
        self._log_default_once()
        self.mode = ExportBleSpeedMode.FIXED
        self._snapshot = self.read_link_snapshot(client)
        self._snapshot.mode = self.mode
        self._snapshot.reason = "firmware_owned_exact_ci"
        self._snapshot.conn_param_requested = "none"
        self._snapshot.conn_param_control = FIXED_CONN_PARAM_CONTROL
        self._snapshot.ci_mode = "online_high_11p25ms"
        self._snapshot.ci_target_ms = "11.25"
        try:
            ci_ms = float(self._snapshot.ci_effective_ms)
            self._snapshot.ci_target_reached = (
                "1"
                if abs(ci_ms - 11.25) <= CI_TARGET_REACHED_TOLERANCE_MS
                and self._snapshot.conn_latency == 0
                else "0"
            )
        except (TypeError, ValueError):
            self._snapshot.ci_target_reached = "unknown"
        self._snapshot.phy_requested = "unsupported"
        self._log(
            "[HOST_BLE_SPEED] "
            f"winrt_throughput_request=disabled reason={reason} "
            "host_request=none firmware_owns_exact_ci=1"
        )
        self._log_effective_link(self._snapshot)
        return self._snapshot

    async def _wait_for_throughput_link(self, client: Any) -> BleSpeedSnapshot:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self._settle_timeout_s
        while True:
            snapshot = self.read_link_snapshot(client)
            if self._is_throughput_ready(snapshot) or loop.time() >= deadline:
                return snapshot
            await asyncio.sleep(self._poll_interval_s)

    @staticmethod
    def _is_throughput_ready(snapshot: BleSpeedSnapshot) -> bool:
        try:
            ci_ms = float(snapshot.ci_effective_ms)
        except (TypeError, ValueError):
            return False
        return (
            ci_ms <= WINRT_THROUGHPUT_TARGET_MS + CI_TARGET_REACHED_TOLERANCE_MS
            and snapshot.conn_latency == 0
        )

    def _finish_without_request(
        self,
        client: Any,
        *,
        mode: ExportBleSpeedMode,
        control: str,
        reason: str,
    ) -> BleSpeedSnapshot:
        self.mode = mode
        self._snapshot = self.read_link_snapshot(client)
        self._snapshot.mode = mode
        self._snapshot.reason = reason
        self._snapshot.conn_param_requested = "none"
        self._snapshot.conn_param_control = control
        self._snapshot.phy_requested = "unsupported"
        self._log("[HOST_BLE_SPEED] conn_param_requested=none")
        self._log(f"[HOST_BLE_SPEED] conn_param_control={control} reason={reason}")
        self._log_effective_link(self._snapshot)
        return self._snapshot

    async def restore(self, client: Any | None, *, reason: str) -> BleSpeedSnapshot:
        if self.mode in {ExportBleSpeedMode.IDLE, ExportBleSpeedMode.FIXED} and self._request is None:
            if client is not None:
                self._snapshot = self.read_link_snapshot(client)
            self.mode = ExportBleSpeedMode.IDLE
            self._snapshot.mode = self.mode
            return self._snapshot

        self.mode = ExportBleSpeedMode.RESTORING
        self._snapshot.mode = self.mode
        self._log(f"[HOST_BLE_SPEED] mode=restoring reason={reason}")

        request = self._request
        self._request = None
        self._requester = None
        if request is not None:
            try:
                request.close()
                self._log("[HOST_BLE_SPEED] winrt_throughput_restore=ok")
            except Exception as exc:
                self._log(f"[HOST_BLE_SPEED] winrt_throughput_restore=fail reason={exc}")

        self.mode = ExportBleSpeedMode.IDLE
        if client is not None:
            self._snapshot = self.read_link_snapshot(client)
        self._snapshot.mode = self.mode
        return self._snapshot

    def force_release(self, *, reason: str) -> None:
        request = self._request
        self._request = None
        self._requester = None
        self.mode = ExportBleSpeedMode.IDLE
        self._snapshot.mode = self.mode
        if request is None:
            return
        try:
            request.close()
            self._log(f"[HOST_BLE_SPEED] winrt_throughput_release=ok reason={reason}")
        except Exception as exc:
            self._log(f"[HOST_BLE_SPEED] winrt_throughput_release=fail reason={exc}")

    def read_link_snapshot(self, client: Any | None) -> BleSpeedSnapshot:
        snapshot = BleSpeedSnapshot(mode=self.mode)
        snapshot.conn_param_requested = "none"
        snapshot.conn_param_control = FIXED_CONN_PARAM_CONTROL
        if client is None:
            snapshot.reason = "no_client"
            return snapshot

        snapshot.backend = _backend_name(client)

        try:
            snapshot.mtu_effective = str(getattr(client, "mtu_size"))
        except Exception:
            snapshot.mtu_effective = "unknown"

        requester = self._get_winrt_requester(client)
        if requester is None:
            snapshot.reason = "not_winrt_backend"
            return snapshot

        try:
            params = requester.get_connection_parameters()
            ci = observe_connection_interval(getattr(params, "connection_interval", None))
            ci_ms = ci.ci_ms
            snapshot.ci_raw = ci.ci_raw
            snapshot.ci_raw_type = ci.ci_raw_type
            latency = int(getattr(params, "connection_latency"))
            snapshot.conn_latency = latency
            timeout_units = int(getattr(params, "link_timeout"))
            if ci_ms is None:
                snapshot.conn_param_effective = (
                    f"ci_raw={ci.ci_raw} ci_raw_type={ci.ci_raw_type} "
                    f"ci_ms=unknown latency={latency} timeout_ms={timeout_units * 10}"
                )
            else:
                snapshot.ci_effective_ms = format_ci_ms(ci_ms)
                snapshot.ci_target_reached = _ci_target_reached(ci_ms)
                snapshot.conn_param_effective = (
                    f"ci_raw={ci.ci_raw} ci_raw_type={ci.ci_raw_type} "
                    f"ci_ms={ci_ms:.2f} latency={latency} timeout_ms={timeout_units * 10}"
                )
        except Exception:
            snapshot.conn_param_effective = "unknown"

        try:
            phy = requester.get_connection_phy()
            if getattr(phy, "is_uncoded2_m_phy", False):
                snapshot.phy_effective = "2M"
            elif getattr(phy, "is_uncoded1_m_phy", False):
                snapshot.phy_effective = "1M"
            elif getattr(phy, "is_coded_phy", False):
                snapshot.phy_effective = "coded"
        except Exception:
            snapshot.phy_effective = "unknown"

        return snapshot

    def _log_default_once(self) -> None:
        if self._logged_default:
            return
        self._logged_default = True
        self._log(
            "[HOST_BLE_SPEED] "
            "winrt_throughput_default=0 host_request=none firmware_owns_exact_ci=1"
        )

    @staticmethod
    def _get_winrt_requester(client: Any) -> Any | None:
        backend = getattr(client, "_backend", None)
        if backend is None:
            return None
        return getattr(backend, "_requester", None)

    def _log_effective_link(self, snapshot: BleSpeedSnapshot) -> None:
        self._log(
            "[HOST_BLE_SPEED] "
            f"mtu_requested={snapshot.mtu_requested} mtu_effective={snapshot.mtu_effective}"
        )
        self._log(
            "[HOST_BLE_SPEED] "
            f"phy_requested={snapshot.phy_requested} phy_effective={snapshot.phy_effective}"
        )
        self._log(
            "[HOST_BLE_SPEED] "
            f"conn_param_requested={snapshot.conn_param_requested} "
            f"conn_param_effective={snapshot.conn_param_effective}"
        )


class WindowsCentralProfileManager:
    """Owns the single live WinRT preferred-parameter request for a connection."""

    _PROFILE_ATTR = {
        WindowsCentralProfile.BALANCED: "balanced",
        WindowsCentralProfile.THROUGHPUT_OPTIMIZED: "throughput_optimized",
        WindowsCentralProfile.POWER_OPTIMIZED: "power_optimized",
    }
    _EXPECTED = {
        WindowsCentralProfile.BALANCED: WindowsCentralProfileSpec(24, 48, 0, 400),
        WindowsCentralProfile.THROUGHPUT_OPTIMIZED: WindowsCentralProfileSpec(12, 12, 0, 200),
        WindowsCentralProfile.POWER_OPTIMIZED: WindowsCentralProfileSpec(72, 144, 0, 600),
    }

    def __init__(
        self,
        log_callback: LogCallback,
        *,
        apply_timeout_s: float = WINDOWS_CENTRAL_APPLY_TIMEOUT_S,
        poll_interval_s: float = WINDOWS_CENTRAL_POLL_INTERVAL_S,
    ) -> None:
        self._log = log_callback
        self._apply_timeout_s = max(0.0, float(apply_timeout_s))
        self._poll_interval_s = max(0.0, float(poll_interval_s))
        self._lock = asyncio.Lock()
        self._request: Any | None = None
        self._requester: Any | None = None
        self._profile = WindowsCentralProfile.NONE
        self._operation_key: tuple[int, int, int, int] | None = None

    @property
    def active_profile(self) -> WindowsCentralProfile:
        return self._profile

    @staticmethod
    def windows_build() -> int:
        if sys.platform != "win32":
            return 0
        try:
            return int(sys.getwindowsversion().build)
        except Exception:
            return 0

    @classmethod
    def inspect_runtime_profiles(
        cls,
    ) -> tuple[bool, dict[WindowsCentralProfile, WindowsCentralProfileSpec], int, str]:
        if cls.windows_build() < 22000:
            return False, {}, 0, "windows_build_lt_22000"
        try:
            from winrt.windows.devices.bluetooth import (
                BluetoothLEPreferredConnectionParameters,
            )
        except Exception as exc:
            return False, {}, 0, f"winrt_import_failed:{exc}"

        specs: dict[WindowsCentralProfile, WindowsCentralProfileSpec] = {}
        try:
            for profile, attr_name in cls._PROFILE_ATTR.items():
                value = getattr(BluetoothLEPreferredConnectionParameters, attr_name)
                specs[profile] = WindowsCentralProfileSpec(
                    int(value.min_connection_interval),
                    int(value.max_connection_interval),
                    int(value.connection_latency),
                    int(value.link_timeout),
                )
        except Exception as exc:
            return False, {}, 0, f"profile_read_failed:{exc}"
        signature_bytes = b"".join(
            struct.pack(
                "<HHHH",
                spec.minimum_interval,
                spec.maximum_interval,
                spec.latency,
                spec.link_timeout,
            )
            for spec in (
                specs[WindowsCentralProfile.BALANCED],
                specs[WindowsCentralProfile.THROUGHPUT_OPTIMIZED],
                specs[WindowsCentralProfile.POWER_OPTIMIZED],
            )
        )
        signature = binascii.crc32(signature_bytes) & 0xFFFFFFFF
        if specs != cls._EXPECTED or signature != WINDOWS_CENTRAL_PROFILE_SIGNATURE:
            return False, specs, signature, "profile_contract_mismatch"
        return True, specs, signature, "ok"

    async def activate(
        self,
        client: Any,
        profile: WindowsCentralProfile,
        *,
        operation_key: tuple[int, int, int, int] | None = None,
        on_accepted: Callable[[int], Awaitable[None]] | None = None,
    ) -> WindowsCentralApplyResult:
        async with self._lock:
            requester = WinRtExportSpeedAdapter._get_winrt_requester(client)
            if requester is None:
                return WindowsCentralApplyResult(
                    profile, 0, False, False, None, None, "not_winrt_backend"
                )
            valid, _specs, _signature, reason = self.inspect_runtime_profiles()
            if not valid:
                return WindowsCentralApplyResult(
                    profile, 0, False, False, None, None, reason
                )
            self._close_request("profile_switch")
            try:
                from winrt.windows.devices.bluetooth import (
                    BluetoothLEPreferredConnectionParameters,
                    BluetoothLEPreferredConnectionParametersRequestStatus,
                )

                preferred = getattr(
                    BluetoothLEPreferredConnectionParameters,
                    self._PROFILE_ATTR[profile],
                )
                request = requester.request_preferred_connection_parameters(preferred)
                request_status = int(_enum_value(getattr(request, "status", 0)))
                success_status = int(
                    _enum_value(
                        BluetoothLEPreferredConnectionParametersRequestStatus.SUCCESS
                    )
                )
            except Exception as exc:
                return WindowsCentralApplyResult(
                    profile, 0, False, False, None, None, f"request_failed:{exc}"
                )
            self._request = request
            self._requester = requester
            self._profile = profile
            self._operation_key = operation_key
            accepted = request_status == success_status
            try:
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"profile={profile.name} winrt_status={request_status} accepted={int(accepted)}"
                )
                if not accepted:
                    self._close_request_if_owned(operation_key, "request_not_accepted")
                    return WindowsCentralApplyResult(
                        profile, request_status, False, False, None, None, "request_not_accepted"
                    )
                if on_accepted is not None:
                    await on_accepted(request_status)

                loop = asyncio.get_running_loop()
                deadline = loop.time() + self._apply_timeout_s
                stable_count = 0
                last_ci: int | None = None
                last_latency: int | None = None
                while loop.time() <= deadline:
                    last_ci, last_latency = self._read_actual_units(requester)
                    if self._matches(profile, last_ci, last_latency):
                        stable_count += 1
                        if stable_count >= 2:
                            return WindowsCentralApplyResult(
                                profile,
                                request_status,
                                True,
                                True,
                                last_ci,
                                last_latency,
                                "actual_stable",
                            )
                    else:
                        stable_count = 0
                    await asyncio.sleep(self._poll_interval_s)
                self._close_request_if_owned(operation_key, "actual_timeout")
                return WindowsCentralApplyResult(
                    profile,
                    request_status,
                    True,
                    False,
                    last_ci,
                    last_latency,
                    "actual_timeout",
                )
            except asyncio.CancelledError:
                self._close_request_if_owned(operation_key, "profile_cancelled")
                raise

    def force_release(self, *, reason: str) -> None:
        self._close_request(reason)

    async def release_if_owned(
        self,
        *,
        operation_key: tuple[int, int, int, int],
        reason: str,
    ) -> bool:
        """Release only the request owned by the named Central operation."""
        async with self._lock:
            if self._operation_key != operation_key:
                self._log(
                    "[HOST_CI_CENTRAL] release_suppressed=owner_mismatch "
                    f"requested={operation_key} owner={self._operation_key} reason={reason}"
                )
                return False
            self._close_request(reason)
            return True

    async def release_for_standby(
        self,
        *,
        operation_key: tuple[int, int, int, int],
        active_key: tuple[int, int, int, int] | None,
    ) -> bool:
        """Release exactly the request owned by the active Central transaction."""
        async with self._lock:
            if self._operation_key not in {None, active_key}:
                self._log(
                    "[HOST_CI_CENTRAL] release_suppressed=owner_mismatch "
                    f"handoff={operation_key} owner={self._operation_key}"
                )
                return False
            self._close_request("hybrid_standby_release")
            self._log(
                "[HOST_CI_CENTRAL] release_for_standby=1 "
                f"handoff={operation_key}"
            )
            return True

    def read_actual_units(self, client: Any) -> tuple[int | None, int | None]:
        """Read the Windows Central-observed link parameters without a request."""
        requester = WinRtExportSpeedAdapter._get_winrt_requester(client)
        if requester is None:
            return None, None
        return self._read_actual_units(requester)

    def _close_request(self, reason: str) -> None:
        request = self._request
        self._request = None
        self._requester = None
        self._profile = WindowsCentralProfile.NONE
        self._operation_key = None
        if request is None:
            return
        try:
            request.close()
            self._log(f"[HOST_CI_CENTRAL] request_closed reason={reason}")
        except Exception as exc:
            self._log(
                f"[HOST_CI_CENTRAL] request_close_failed reason={reason} error={exc}"
            )

    def _close_request_if_owned(
        self,
        operation_key: tuple[int, int, int, int] | None,
        reason: str,
    ) -> None:
        if self._operation_key != operation_key:
            return
        self._close_request(reason)

    @staticmethod
    def _read_actual_units(requester: Any) -> tuple[int | None, int | None]:
        try:
            params = requester.get_connection_parameters()
            observation = observe_connection_interval(params.connection_interval)
            if observation.ci_ms is None:
                return None, None
            ci = int(round(observation.ci_ms / 1.25))
            return ci, int(params.connection_latency)
        except Exception:
            return None, None

    @classmethod
    def _matches(
        cls,
        profile: WindowsCentralProfile,
        ci: int | None,
        latency: int | None,
    ) -> bool:
        if ci is None or latency != 0:
            return False
        expected = cls._EXPECTED[profile]
        return expected.minimum_interval <= ci <= expected.maximum_interval


def observe_connection_interval(value: Any) -> ConnectionIntervalObservation:
    raw_type = type(value).__name__ if value is not None else "NoneType"
    raw_text = _safe_raw_text(value)
    if value is None or isinstance(value, bool):
        return ConnectionIntervalObservation(raw_text, raw_type, None)

    if isinstance(value, timedelta):
        return ConnectionIntervalObservation(raw_text, raw_type, value.total_seconds() * 1000.0, "timedelta")

    total_seconds = getattr(value, "total_seconds", None)
    if callable(total_seconds):
        try:
            return ConnectionIntervalObservation(
                raw_text,
                raw_type,
                float(total_seconds()) * 1000.0,
                "total_seconds",
            )
        except Exception:
            return ConnectionIntervalObservation(raw_text, raw_type, None)

    for attr_name in ("total_milliseconds", "milliseconds"):
        attr = getattr(value, attr_name, None)
        try:
            attr_value = attr() if callable(attr) else attr
            if attr_value is not None and not isinstance(attr_value, bool):
                return ConnectionIntervalObservation(
                    _safe_raw_text(attr_value),
                    f"{raw_type}.{attr_name}",
                    float(attr_value),
                    attr_name,
                )
        except Exception:
            return ConnectionIntervalObservation(raw_text, raw_type, None)

    for attr_name in ("duration", "ticks"):
        attr = getattr(value, attr_name, None)
        try:
            attr_value = attr() if callable(attr) else attr
            if attr_value is not None and not isinstance(attr_value, bool):
                if isinstance(attr_value, timedelta):
                    return ConnectionIntervalObservation(
                        _safe_raw_text(attr_value),
                        f"{raw_type}.{attr_name}",
                        attr_value.total_seconds() * 1000.0,
                        attr_name,
                    )
                attr_total_seconds = getattr(attr_value, "total_seconds", None)
                if callable(attr_total_seconds):
                    return ConnectionIntervalObservation(
                        _safe_raw_text(attr_value),
                        f"{raw_type}.{attr_name}",
                        float(attr_total_seconds()) * 1000.0,
                        attr_name,
                    )
                return ConnectionIntervalObservation(
                    _safe_raw_text(attr_value),
                    f"{raw_type}.{attr_name}",
                    float(attr_value) / 10_000.0,
                    attr_name,
                )
        except Exception:
            return ConnectionIntervalObservation(raw_text, raw_type, None)

    if isinstance(value, int):
        if BLE_CI_UNIT_MIN <= value <= BLE_CI_UNIT_MAX:
            return ConnectionIntervalObservation(raw_text, raw_type, value * 1.25, "ble_units")
        if 1 <= value <= 4000:
            return ConnectionIntervalObservation(raw_text, raw_type, float(value), "milliseconds")
        return ConnectionIntervalObservation(raw_text, raw_type, None)
    if isinstance(value, float):
        return ConnectionIntervalObservation(raw_text, raw_type, value, "milliseconds")
    return ConnectionIntervalObservation(raw_text, raw_type, None)


def connection_interval_to_ms(value: Any) -> float | None:
    return observe_connection_interval(value).ci_ms


def _safe_raw_text(value: Any) -> str:
    if value is None:
        return "unknown"
    if isinstance(value, (str, int, float)):
        return str(value)
    if isinstance(value, timedelta):
        return str(value)
    try:
        return repr(value)
    except Exception:
        return "unrepresentable"


def _backend_name(client: Any) -> str:
    backend_id = getattr(client, "backend_id", "")
    if backend_id:
        return str(backend_id).lower()
    backend = getattr(client, "_backend", None)
    if backend is None:
        return "unknown"
    module = getattr(type(backend), "__module__", "")
    if module:
        if "winrt" in module.lower():
            return "winrt"
        return module
    return type(backend).__name__


def _ci_target_reached(effective_ms: float | None) -> str:
    if effective_ms is None:
        return "unknown"
    return "1" if effective_ms <= export_target_ci_ms() + CI_TARGET_REACHED_TOLERANCE_MS else "0"


def _enum_value(value: Any) -> Any:
    raw = getattr(value, "value", value)
    try:
        return int(raw)
    except (TypeError, ValueError):
        return raw


def _safe_attr(value: Any, name: str) -> str:
    try:
        return str(getattr(value, name))
    except Exception:
        return "unknown"
