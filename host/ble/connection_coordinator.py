from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import Any, Awaitable, Callable, TypeVar

from .gatt_operation_queue import GattOperationQueue, GattPriority


class ConnectionStage(str, Enum):
    CONNECTING = "connecting"
    LINK_CONNECTED = "link_connected"
    DISCOVERING_GATT = "discovering_gatt"
    READING_DEVICE_INFO = "reading_device_info"
    SUBSCRIBING_ACK = "subscribing_ack"
    BOOTSTRAP_QUERY = "bootstrap_query"
    WAITING_SECURITY = "waiting_security"
    NEGOTIATING_LINK_POLICY = "negotiating_link_policy"
    SYNCING_USER = "syncing_user"
    SYNCING_CALIBRATION = "syncing_calibration"
    SUBSCRIBING_EXPORT = "subscribing_export"
    SYNCING_TIME = "syncing_time"
    CONTROL_READY = "control_ready"
    SYNCING_OFFLINE = "syncing_offline"
    ONLINE_READY = "online_ready"
    SYNCING_FEATURE_CONFIG = "syncing_feature_config"
    BUSINESS_READY = "business_ready"
    RECOVERY_ONLY = "recovery_only"
    FAILED = "failed"
    DISCONNECTED = "disconnected"
    DEFERRED_OFFLINE_CAPTURE = "deferred_offline_capture"  # reserved


@dataclass(frozen=True, slots=True)
class DeviceInfo:
    raw: str
    fields: dict[str, str]
    proto: int | None
    code: int | None
    boot: int | None
    offmeta: int | None
    usrctx: int | None
    fcfg: int | None

    @classmethod
    def parse(cls, raw: str) -> "DeviceInfo":
        fields: dict[str, str] = {}
        for part in raw.split(";"):
            key, separator, value = part.partition("=")
            if separator and key.strip():
                fields[key.strip()] = value.strip()

        def number(name: str) -> int | None:
            value = fields.get(name)
            if value is None:
                return None
            try:
                return int(value, 10)
            except ValueError:
                return None

        return cls(
            raw=raw,
            fields=fields,
            proto=number("proto"),
            code=number("code"),
            boot=number("boot"),
            offmeta=number("offmeta"),
            usrctx=number("usrctx"),
            fcfg=number("fcfg"),
        )


class ProtocolStrategy:
    name = "unsupported"
    host_ci_version: int | None = None
    bootstrap = False


class BootstrapV1Strategy(ProtocolStrategy):
    name = "bootstrap_v1"
    host_ci_version = 3
    bootstrap = True


class BootstrapV2Strategy(BootstrapV1Strategy):
    name = "bootstrap_v2"


class BootstrapV3Strategy(BootstrapV2Strategy):
    name = "bootstrap_v3"


class BootstrapV4Strategy(BootstrapV3Strategy):
    name = "bootstrap_v4"


class LegacyWindowsStrategy(ProtocolStrategy):
    name = "legacy_windows"
    host_ci_version = 1


class LegacyAndroidStrategy(ProtocolStrategy):
    name = "legacy_android"
    host_ci_version = 2


class UnsupportedProtocolError(RuntimeError):
    pass


T = TypeVar("T")


class ConnectionCoordinator:
    """Connection-local ordering, protocol selection and stale callback fence."""

    def __init__(self, emit: Callable[[dict[str, Any]], None], *, platform: str = "windows") -> None:
        self._emit = emit
        self._platform = platform.lower()
        self._token = 0
        self._address = ""
        self._stage = ConnectionStage.DISCONNECTED
        self._device_info: DeviceInfo | None = None
        self._strategy: ProtocolStrategy | None = None
        self._gatt_queue = GattOperationQueue(self.is_current)

    @property
    def token(self) -> int:
        return self._token

    @property
    def strategy(self) -> ProtocolStrategy | None:
        return self._strategy

    @property
    def device_info(self) -> DeviceInfo | None:
        return self._device_info

    def begin(self, address: str) -> int:
        self._token += 1
        self._gatt_queue.cancel_stale(self._token)
        self._address = address
        self._device_info = None
        self._strategy = None
        self.transition(ConnectionStage.CONNECTING)
        return self._token

    def invalidate(self) -> None:
        self._token += 1
        self._gatt_queue.cancel_stale(self._token)
        self._address = ""
        self._strategy = None
        self.transition(ConnectionStage.DISCONNECTED)

    def is_current(self, token: int) -> bool:
        return token == self._token

    def gatt_diagnostic_snapshot(self) -> dict[str, object]:
        return self._gatt_queue.diagnostic_snapshot()

    def guard_callback(self, token: int, callback: Callable[..., Any]) -> Callable[..., Any]:
        def guarded(*args: Any, **kwargs: Any) -> Any:
            if not self.is_current(token):
                return None
            return callback(*args, **kwargs)

        return guarded

    async def gatt(
        self,
        token: int,
        operation: Callable[[], Awaitable[T]],
        *,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "gatt",
    ) -> T:
        return await self._gatt_queue.submit(
            token, operation, priority=priority, name=name
        )

    async def gatt_cleanup(
        self,
        operation: Callable[[], Awaitable[T]],
        *,
        name: str = "gatt_cleanup",
    ) -> T:
        return await self._gatt_queue.submit(
            None, operation, priority=GattPriority.CRITICAL, name=name
        )

    async def read_gatt_char(
        self, token: int, client: object, characteristic: object, *,
        priority: GattPriority = GattPriority.CRITICAL, name: str = "read",
    ) -> object:
        return await self._gatt_queue.read(
            token, client, characteristic, priority=priority, name=name
        )

    async def write_gatt_char(
        self, token: int, client: object, characteristic: object, data: bytes, *,
        response: bool = True,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "write",
        timeout: float | None = None,
    ) -> object:
        return await self._gatt_queue.write(
            token, client, characteristic, data,
            response=response, priority=priority, name=name, timeout=timeout,
        )

    async def start_notify(
        self, token: int, client: object, characteristic: object, callback: object, *,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "start_notify",
    ) -> object:
        return await self._gatt_queue.start_notify(
            token, client, characteristic, callback,
            priority=priority, name=name,
        )

    async def write_gatt_char_with_fallback(
        self, token: int, client: object, characteristic: object, data: bytes, *,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "write_with_fallback",
        guard: Callable[[], bool] | None = None,
    ) -> bool:
        return await self._gatt_queue.write_with_fallback(
            token, client, characteristic, data,
            priority=priority, name=name, guard=guard,
        )

    async def stop_notify(
        self, token: int | None, client: object, characteristic: object, *,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "stop_notify",
    ) -> object:
        return await self._gatt_queue.stop_notify(
            token, client, characteristic, priority=priority, name=name
        )

    def lock_device_info(self, raw: str) -> ProtocolStrategy:
        parsed = DeviceInfo.parse(raw)
        if self._device_info is not None and parsed.raw != self._device_info.raw:
            raise UnsupportedProtocolError("Device Info changed during the active BLE connection")
        self._device_info = parsed
        if parsed.proto != 1:
            raise UnsupportedProtocolError(
                f"Unsupported firmware protocol: proto={parsed.proto!r}, code={parsed.code!r}"
            )
        if parsed.boot == 4:
            if parsed.usrctx != 1 or parsed.fcfg != 1:
                raise UnsupportedProtocolError(
                    "Bootstrap v4 firmware must declare usrctx=1 and fcfg=1"
                )
            strategy = BootstrapV4Strategy()
        elif parsed.boot == 3:
            if parsed.usrctx != 1:
                raise UnsupportedProtocolError(
                    "Bootstrap v3 firmware must declare usrctx=1"
                )
            strategy = BootstrapV3Strategy()
        elif parsed.boot == 2:
            strategy = BootstrapV2Strategy()
        elif parsed.boot == 1:
            strategy: ProtocolStrategy = BootstrapV1Strategy()
        elif parsed.boot is None and parsed.code is not None and 10355 <= parsed.code <= 10386:
            strategy = LegacyAndroidStrategy() if self._platform == "android" else LegacyWindowsStrategy()
        elif parsed.boot is None:
            raise UnsupportedProtocolError(
                f"Untested legacy firmware: proto={parsed.proto!r}, code={parsed.code!r}; OTA upgrade only"
            )
        else:
            raise UnsupportedProtocolError(
                f"Unsupported bootstrap declaration: boot={parsed.boot!r}"
            )
        self._strategy = strategy
        return strategy

    def transition(self, stage: ConnectionStage, *, message: str = "", **extra: Any) -> None:
        self._stage = stage
        payload: dict[str, Any] = {
            "type": "connection_stage",
            "stage": stage.value,
            "token": self._token,
        }
        if message:
            payload["message"] = message
        payload.update(extra)
        self._emit(payload)
        if stage in {
            ConnectionStage.LINK_CONNECTED,
            ConnectionStage.CONTROL_READY,
            ConnectionStage.BUSINESS_READY,
            ConnectionStage.DEFERRED_OFFLINE_CAPTURE,
            ConnectionStage.RECOVERY_ONLY,
            ConnectionStage.FAILED,
            ConnectionStage.DISCONNECTED,
        }:
            event_type = (
                "connection_failed"
                if stage is ConnectionStage.FAILED
                else stage.value
            )
            self._emit({"type": event_type, "token": self._token, **extra})
