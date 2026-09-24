"""Bounded, pairing-preserving connection recovery (no protocol implementation)."""
from __future__ import annotations

import asyncio
import time
from typing import Any


class ConnectionFailure(RuntimeError):
    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


class ConnectionRecovery:
    RETURN_WINDOW = 90.0
    CONNECT_TIMEOUT = 30.0
    RETRY_DELAYS = (1.0, 2.0)

    def __init__(self, manager: Any) -> None:
        self.manager = manager
        self.operation_id = 0
        self.task: asyncio.Task | None = None
        self.confirmation: asyncio.Future | None = None
        self.expected_sn = ""
        self.authorized_address = ""
        self.authorized_endpoint_id = ""
        self.confirmed_endpoint_id = ""
        self.started = 0.0
        self.cancelled = False

    def check_cancelled(self) -> None:
        if self.cancelled:
            raise asyncio.CancelledError

    def respond(self, operation_id: int, approved: bool) -> None:
        if operation_id != self.operation_id or self.task is None:
            return
        if self.confirmation is not None and not self.confirmation.done():
            self.confirmation.set_result(bool(approved))

    def current(self, operation_id: int) -> bool:
        return self.task is not None and self.operation_id == operation_id

    async def cancel(self) -> None:
        self.operation_id += 1
        self.authorized_address = ""
        self.authorized_endpoint_id = ""
        self.confirmed_endpoint_id = ""
        self.cancelled = True
        task = self.task
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self.confirmation is not None and not self.confirmation.done():
            self.confirmation.cancel()

    def progress(self, stage: str, message: str, **extra: Any) -> None:
        m = self.manager
        m._log(f"[BLE_RECOVERY] op={self.operation_id} token={m._connection_token} "
               f"stage={stage} elapsed_ms={int((time.monotonic()-self.started)*1000)} message={message} {extra}")
        m._emit({"type": "connection_recovery_stage", "stage": stage,
                 "message": message, "operation_id": self.operation_id, **extra})

    async def _confirm(self, address: str, name: str, reason: str, endpoint_id: str) -> bool:
        self.confirmed_endpoint_id = endpoint_id
        self.confirmation = asyncio.get_running_loop().create_future()
        self.manager._emit({"type": "pairing_repair_confirmation",
                            "operation_id": self.operation_id, "address": address,
                            "name": name, "sn": self.expected_sn, "reason": reason,
                            "endpoint_id": endpoint_id})
        try:
            return bool(await self.confirmation)
        finally:
            self.confirmation = None

    async def _discover(self, device: Any, address: str, name: str, attempt: int) -> Any:
        from .ble_manager import GATT_DISCOVERY_ALL_SERVICES
        self.progress("discovering_gatt", "正在建立 GATT 会话并完整发现服务", attempt=attempt,
                      cache="uncached", discovery_mode=GATT_DISCOVERY_ALL_SERVICES)
        try:
            return await asyncio.wait_for(
                self.manager._connect_and_discover_once(
                    device, address, name, attempt=attempt, total_attempts=3,
                    discovery_mode=GATT_DISCOVERY_ALL_SERVICES, use_cached_services=False),
                timeout=self.CONNECT_TIMEOUT)
        except asyncio.CancelledError:
            raise
        except ConnectionFailure:
            raise
        except Exception as exc:
            kind = "security" if self.manager._is_pairing_failure(exc) else "gatt_discovery"
            raise ConnectionFailure(kind, f"{kind}: {type(exc).__name__}: {exc}") from exc

    async def run(self, address: str, name: str, user_id: int, training_id: int,
                  *, ota: bool) -> None:
        if self.task is not None:
            self.manager._log("[BLE_RECOVERY] ignored duplicate connection request")
            return
        m = self.manager
        if not address or user_id <= 0:
            m._emit({"type": "error", "message": "连接需要目标地址和非零用户 ID"})
            return
        self.operation_id += 1
        self.task = asyncio.current_task()
        self.started = time.monotonic()
        self.cancelled = False
        self.authorized_address = ""
        trust = m._gatt_trust_store.get(address) or {}
        info = m._connection_coordinator.device_info
        self.expected_sn = str(trust.get("sn") or "")
        if m._connected_address == address and info is not None:
            self.expected_sn = info.fields.get("sn", "") or self.expected_sn
        unpaired = False
        success = False
        preserve_business = False
        missing_count = 0
        security_seen = False
        terminal_event = None
        failure = ConnectionFailure("unreachable", "未发现返回的设备")
        try:
            await m._safe_disconnect(emit=False)
            m._gatt_trust_store.invalidate(address)
            m._connection_started_monotonic = time.monotonic()
            m._central_enable_reconnect_used = True  # one retry owner, no recursive connect
            prepared = None
            device = m._devices.get(address) or m._make_winrt_cached_device(address, name)
            first_failure = None
            if ota:
                deadline = time.monotonic() + self.RETURN_WINDOW
                while time.monotonic() < deadline:
                    self.progress("waiting_for_device", "等待升级后的设备返回（保留配对）")
                    remaining = deadline - time.monotonic()
                    scanned = await m._scan_for_device_by_address(address, timeout=min(4.0, remaining))
                    device = scanned or m._make_winrt_cached_device(address, name)
                    if device is not None:
                        try:
                            # A successful GATT session is also evidence of return when not advertising.
                            if scanned is not None:
                                prepared = await self._discover(device, address, name, 1)
                            else:
                                remaining = max(0.001, deadline - time.monotonic())
                                prepared = await asyncio.wait_for(
                                    self._discover(device, address, name, 1), timeout=remaining)
                            break
                        except asyncio.TimeoutError:
                            raise ConnectionFailure("unreachable", "设备返回等待超时；未删除配对")
                        except ConnectionFailure as exc:
                            failure = exc
                            if exc.kind in {"access_denied", "cleanup"}:
                                raise
                            if exc.kind == "security" or scanned is not None:
                                security_seen = exc.kind == "security"
                                first_failure = exc
                                break
                    await asyncio.sleep(min(2.0, max(0.0, deadline-time.monotonic())))
                else:
                    raise ConnectionFailure("unreachable", "设备返回等待超时；未删除配对")
            if device is None:
                raise ConnectionFailure("unreachable", "无法定位目标设备，请重新扫描")
            name = getattr(device, "name", "") or name or address
            m._current_device_name = name

            for repair_round in range(2):
                for attempt in range(1, 4):
                    self.check_cancelled()
                    if attempt > 1:
                        await asyncio.sleep(self.RETRY_DELAYS[attempt-2])
                    client = None
                    try:
                        if first_failure is not None:
                            failed_probe = first_failure
                            first_failure = None
                            raise failed_probe
                        if prepared is not None:
                            client, status = prepared
                            prepared = None
                        else:
                            client, status = await self._discover(device, address, name, attempt)
                        self.check_cancelled()
                        if not m._has_required_gatt():
                            missing_count += 1
                            missing = ", ".join(m._required_gatt_missing())
                            raise ConnectionFailure("gatt_missing", f"必需 GATT 缺失: {missing}")
                        self.progress("initializing", "正在进行安全验证与协议初始化", attempt=attempt)
                        m._gatt_fast_path_active = False
                        m._gatt_ready_monotonic = time.monotonic()
                        await m._initialize_connection(client, status, device, address, name,
                                                       user_id=user_id, training_id=training_id)
                        self.check_cancelled()
                        if m._connection_business_ready:
                            success = True
                            self.progress("business_ready", "设备业务连接已就绪")
                            if ota:
                                m._emit({"type": "ota_reconnect_completed", "address": address,
                                         "name": name, "unpair_attempted": unpaired})
                            return
                        # Existing observer/offline recovery owns subsequent business progression.
                        preserve_business = m._offline_observer_active or m._recovery_only
                        raise ConnectionFailure("business", "业务初始化未完成；保持现有数据与业务门禁")
                    except asyncio.CancelledError:
                        raise
                    except Exception as exc:
                        from .connection_coordinator import UnsupportedProtocolError
                        if isinstance(exc, ConnectionFailure):
                            failure = exc
                        elif isinstance(exc, UnsupportedProtocolError):
                            failure = ConnectionFailure("protocol", str(exc))
                        else:
                            kind = "security" if m._is_pairing_failure(exc) else "initialization"
                            failure = ConnectionFailure(kind, f"{type(exc).__name__}: {exc}")
                        phase = getattr(m._connection_coordinator, "_stage", None)
                        if getattr(phase, "value", "") in {
                            "syncing_user", "syncing_calibration", "syncing_offline",
                            "syncing_time", "syncing_feature_config", "online_ready", "recovery_only",
                        }:
                            preserve_business = True
                            m._enter_recovery_only(str(failure))
                            failure = ConnectionFailure("business", str(failure))
                        security_seen |= failure.kind == "security"
                        self.progress(failure.kind, str(failure), attempt=attempt)
                        if preserve_business:
                            raise failure
                        if client is not None:
                            await m._disconnect_for_gatt_retry(client)
                        if failure.kind in {"identity", "protocol", "host_ci", "business", "access_denied", "cleanup"}:
                            raise failure
                eligible = missing_count >= 2 or security_seen
                if repair_round or not eligible:
                    raise failure
                target = await m._resolve_windows_pairing_target(address)
                self.check_cancelled()
                self.progress("pairing_state", f"目标配对状态：{target.state}", endpoint_id=target.endpoint_id,
                              query_error=target.error)
                if target.state != "paired":
                    if target.state == "unknown":
                        raise ConnectionFailure("pairing_unknown", f"{failure}；无法确认目标配对状态，未删除配对")
                    raise failure
                if not await self._confirm(address, name, str(failure), target.endpoint_id):
                    raise ConnectionFailure("repair_declined", "已保留配对，可手动重试连接")
                self.check_cancelled()
                self.authorized_address = address.upper()
                self.authorized_endpoint_id = target.endpoint_id
                try:
                    if not await m._unpair_windows_device(address):
                        raise ConnectionFailure("unpair_failed", "目标设备取消配对失败")
                finally:
                    self.authorized_address = ""
                    self.authorized_endpoint_id = ""
                unpaired = True
                m._devices.pop(address, None)
                m._gatt_trust_store.invalidate(address)
                await asyncio.sleep(2.0)
                device = await m._scan_for_device_by_address(address, timeout=4.0)
                if device is None:
                    raise ConnectionFailure("unreachable", "配对已移除，但未扫描到目标设备")
        except ConnectionFailure as exc:
            self.progress(exc.kind, str(exc))
            terminal_event = {"type": "ota_reconnect_failed" if ota else "connection_failed",
                     "message": str(exc), "reason": exc.kind, "address": address,
                     "operation_id": self.operation_id, "unpair_attempted": unpaired}
        except asyncio.CancelledError:
            self.progress("cancelled", "连接恢复已取消")
            raise
        except Exception as exc:
            self.progress("failed", f"连接恢复失败: {exc}")
            terminal_event = {"type": "ota_reconnect_failed" if ota else "connection_failed",
                     "message": f"连接恢复失败: {exc}", "address": address}
        finally:
            if not success and not preserve_business:
                try:
                    await m._safe_disconnect(emit=False)
                except Exception as exc:
                    terminal_event = {"type": "ota_reconnect_failed" if ota else "connection_failed",
                                      "message": str(exc), "reason": "cleanup", "address": address,
                                      "operation_id": self.operation_id}
            self.expected_sn = ""
            self.authorized_address = ""
            self.authorized_endpoint_id = ""
            self.confirmed_endpoint_id = ""
            self.task = None
            self.confirmation = None
            if terminal_event is not None:
                m._emit(terminal_event)
