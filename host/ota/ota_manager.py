from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .dfu_protocol import DfuProtocolConfig
from .image_info import OtaImageInfo
from .ota_state import OtaState
from .realtek_dfu import DFU_CONTROL_POINT_UUID, DFU_DATA_UUID, DFU_SERVICE_UUID, hex_bytes, load_realtek_app_image
from .runtime_paths import get_firmware_directory, list_firmware_images


LogCallback = Callable[[str], None]


@dataclass(slots=True)
class OtaServiceDiscoveryResult:
    total_service_count: int = 0
    total_char_count: int = 0
    candidate_service_count: int = 0
    candidate_char_count: int = 0
    candidate_service_lines: list[str] = field(default_factory=list)
    candidate_char_lines: list[str] = field(default_factory=list)

    def has_candidate(self) -> bool:
        return (self.candidate_service_count + self.candidate_char_count) > 0

    def to_lines(self) -> list[str]:
        lines = [
            f"服务总数：{self.total_service_count}",
            f"特征总数：{self.total_char_count}",
            f"候选服务：{self.candidate_service_count}",
            f"候选特征：{self.candidate_char_count}",
        ]
        if not self.candidate_service_lines and not self.candidate_char_lines:
            lines.append("未发现名称包含 OTA/DFU 的候选服务或特征。")
            lines.append("说明：本结果仅关键字提示，不等于最终协议确认。")
            return lines

        if self.candidate_service_lines:
            lines.append("")
            lines.append("候选服务：")
            lines.extend(self.candidate_service_lines)
        if self.candidate_char_lines:
            lines.append("")
            lines.append("候选特征：")
            lines.extend(self.candidate_char_lines)
        return lines


@dataclass(slots=True)
class OtaPreflightResult:
    ok: bool
    issues: list[str]


@dataclass(slots=True)
class OtaStartResult:
    ok: bool
    state: OtaState
    message: str
    issues: list[str] = field(default_factory=list)


class OtaManager:
    """OTA UI state holder for the RTL8762D DFU workflow."""

    def __init__(self, log_callback: LogCallback) -> None:
        self._log_callback = log_callback
        self._state = OtaState.IDLE
        self._connected = False
        self._image_info: OtaImageInfo | None = None
        self._services: list[dict[str, Any]] = []
        self._service_result = OtaServiceDiscoveryResult()
        self._protocol_config = DfuProtocolConfig()
        self._dfu_running = False
        self._dfu_progress_summary = "等待开始"
        self._dfu_completion_message = ""

    @property
    def state(self) -> OtaState:
        return self._state

    @property
    def image_info(self) -> OtaImageInfo | None:
        return self._image_info

    @property
    def service_result(self) -> OtaServiceDiscoveryResult:
        return self._service_result

    @property
    def protocol_config(self) -> DfuProtocolConfig:
        return self._protocol_config

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def dfu_running(self) -> bool:
        return self._dfu_running

    @property
    def dfu_progress_summary(self) -> str:
        return self._dfu_progress_summary

    @property
    def dfu_completion_message(self) -> str:
        return self._dfu_completion_message

    def get_firmware_directory(self) -> str | None:
        directory = get_firmware_directory()
        if directory is None:
            return None
        return str(directory)

    def list_builtin_images(self) -> list[str]:
        return [path.name for path in list_firmware_images()]

    def select_builtin_image(self, file_name: str) -> OtaImageInfo:
        wanted = file_name.strip().lower()
        if not wanted:
            raise ValueError("Please select a built-in firmware image first.")

        for path in list_firmware_images():
            if path.name.lower() == wanted:
                return self.select_image(str(path))

        raise ValueError(f"Built-in firmware not found: {file_name}")

    def select_image(self, path: str) -> OtaImageInfo:
        lowered = path.lower()
        if not lowered.endswith(".bin"):
            self._state = OtaState.ERROR
            raise ValueError("请选择 .bin 固件文件")

        try:
            dfu_image = load_realtek_app_image(path)
        except ValueError:
            self._state = OtaState.ERROR
            raise

        info = OtaImageInfo(
            path=dfu_image.path,
            file_name=dfu_image.file_name,
            size_bytes=dfu_image.size_bytes,
            sha256=dfu_image.sha256,
        )
        self._image_info = info
        self._dfu_completion_message = ""
        if not self._dfu_running:
            self._state = OtaState.FILE_SELECTED
        self._recalculate_state()
        self._log(f"固件已选择：{info.file_name} ({info.size_bytes} bytes)")
        self._log("DFU 离线解析完成：")
        self._log(f"file_size = {dfu_image.size_bytes}")
        self._log(f"image_base = 0x{dfu_image.image_base:X}")
        self._log(f"wrapper_size = {dfu_image.wrapper_size}")
        if dfu_image.wrapper_size:
            self._log("wrapper skipped, not sent by DFU")
        self._log(f"dfu_size_bytes = {dfu_image.dfu_size_bytes}")
        self._log(f"ctrl_header = file[0x{dfu_image.image_base:X}:0x{dfu_image.ctrl_header_end:X}]")
        self._log(f"Data start = file[0x{dfu_image.data_start:X}:]")
        self._log(
            "APP image ctrl header: "
            f"ic_type=0x{dfu_image.ic_type:02X}, secure_version=0x{dfu_image.secure_version:02X}, "
            f"ctrl_flag=0x{dfu_image.ctrl_flag:04X}, image_id=0x{dfu_image.image_id:04X}, "
            f"crc16=0x{dfu_image.crc16:04X}, payload_len={dfu_image.payload_len} "
            f"(0x{dfu_image.payload_len:08X})"
        )
        self._log(f"APP image ctrl header hex: {hex_bytes(dfu_image.ctrl_header)}")
        return info

    def clear_image(self) -> None:
        self._image_info = None
        self._dfu_completion_message = ""
        self._dfu_progress_summary = "等待开始"
        if not self._dfu_running:
            self._state = OtaState.IDLE
        self._recalculate_state()
        self._log("已清除 OTA 固件文件选择")

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            self._services = []
            self._service_result = OtaServiceDiscoveryResult()
        self._recalculate_state()
        self._log("OTA 上下文：设备已连接" if connected else "OTA 上下文：设备未连接")

    def update_gatt_services(self, services: list[dict[str, Any]]) -> OtaServiceDiscoveryResult:
        self._services = list(services or [])
        self._service_result = self._detect_candidates(self._services)
        self._recalculate_state()
        self._log(
            "OTA 服务检测完成："
            f"服务 {self._service_result.total_service_count} / 特征 {self._service_result.total_char_count} / "
            f"候选服务 {self._service_result.candidate_service_count} / 候选特征 {self._service_result.candidate_char_count}"
        )
        return self._service_result

    def preflight_check(self) -> OtaPreflightResult:
        issues: list[str] = []
        if self._image_info is None:
            issues.append("未选择固件文件")
        if self._dfu_running:
            issues.append("OTA 正在进行中")
        return OtaPreflightResult(ok=len(issues) == 0, issues=issues)

    def start_enter_dfu_command(self) -> OtaStartResult:
        preflight = self.preflight_check()
        if not preflight.ok:
            self._state = OtaState.ERROR
            message = "Start OTA 前置检查失败"
            self._log(f"{message}：{'；'.join(preflight.issues)}")
            return OtaStartResult(
                ok=False,
                state=self._state,
                message=message,
                issues=preflight.issues,
            )

        self._dfu_running = True
        self._state = OtaState.ENTERING_DFU
        self._dfu_progress_summary = "正在通过普通 BLE 发送进入 DFU 指令"
        self._dfu_completion_message = ""
        self._log("OTA 开始：通过普通 BLE 指令让设备进入 DFU 模式")
        return OtaStartResult(ok=True, state=self._state, message="正在让设备进入 DFU")

    def update_enter_dfu_progress(self, message: str) -> None:
        if message:
            self._dfu_progress_summary = message
            self._log(message)

    def fail_enter_dfu_command(self, message: str) -> None:
        self._dfu_running = False
        self._state = OtaState.ERROR
        self._dfu_progress_summary = message or "进入 DFU 指令失败"
        self._log(self._dfu_progress_summary)

    def start_dfu_upgrade(self, *, after_entry_command: bool = False) -> OtaStartResult:
        if self._dfu_running and not (after_entry_command and self._state == OtaState.ENTERING_DFU):
            return OtaStartResult(
                ok=False,
                state=self._state,
                message="OTA 正在进行中",
                issues=["请等待当前 OTA 完成"],
            )
        if self._image_info is None:
            self._state = OtaState.ERROR
            return OtaStartResult(
                ok=False,
                state=self._state,
                message="Start OTA 前置检查失败",
                issues=["未选择固件文件"],
            )

        self._dfu_running = True
        self._connected = False
        self._state = OtaState.SCANNING
        self._dfu_progress_summary = "正在扫描 ZY100_OTA / DFU Service"
        self._dfu_completion_message = ""
        if after_entry_command:
            self._log("进入 DFU 指令已完成：开始扫描 ZY100_OTA / DFU Service")
        else:
            self._log("RTL8762D DFU OTA 开始：扫描 DFU 设备；若设备未自动进入 DFU，可使用实体按钮入口")
        return OtaStartResult(ok=True, state=self._state, message="DFU OTA 已启动")

    def handle_dfu_event(self, event: dict[str, Any]) -> None:
        event_type = str(event.get("type", ""))
        if event_type == "ota_status":
            state_name = str(event.get("state", ""))
            if state_name in OtaState.__members__:
                self._state = OtaState[state_name]
            if state_name == "CONNECTED":
                self._connected = True
            self._dfu_progress_summary = str(event.get("message", self._dfu_progress_summary))
            return

        if event_type == "ota_progress":
            self._state = OtaState.TRANSFERRING
            sent = int(event.get("sent") or 0)
            total = int(event.get("total") or 0)
            percent = float(event.get("percent") or 0.0)
            phase = str(event.get("phase", "Receive FW Image"))
            speed = event.get("speed")
            eta = event.get("eta")
            suffix = ""
            if isinstance(speed, (int, float)) and isinstance(eta, (int, float)):
                suffix = f"，{speed:.0f} B/s，ETA {eta:.1f}s"
            self._dfu_progress_summary = f"{phase}：{sent}/{total} bytes（{percent:.1f}%）{suffix}"
            return

        if event_type == "ota_completed":
            self._dfu_running = False
            self._connected = False
            self._state = OtaState.COMPLETED
            self._dfu_progress_summary = "OTA 完成，设备已重启"
            self._dfu_completion_message = str(event.get("message", "OTA 已完成"))
            return

        if event_type == "ota_error":
            self._dfu_running = False
            self._connected = False
            self._state = OtaState.COMPLETED if event.get("validated") and event.get("activated") else OtaState.ERROR
            self._dfu_progress_summary = str(event.get("message", "OTA 失败"))
            return

    def start_ota_skeleton(self) -> OtaStartResult:
        preflight = self.preflight_check()
        if not preflight.ok:
            self._state = OtaState.ERROR
            message = "Start OTA 前置检查失败"
            self._log(f"{message}：{'；'.join(preflight.issues)}")
            return OtaStartResult(
                ok=False,
                state=self._state,
                message=message,
                issues=preflight.issues,
            )

        pending = self._protocol_config.pending_items()
        self._state = OtaState.PENDING_PROTOCOL
        message = "协议参数待补充，当前不会发送任何 BLE OTA 命令"
        self._log(message)
        if pending:
            self._log("待确认协议项：" + ", ".join(pending))
        return OtaStartResult(
            ok=False,
            state=self._state,
            message=message,
            issues=pending,
        )

    def _recalculate_state(self) -> None:
        if self._dfu_running:
            return
        if self._state in {
            OtaState.PENDING_PROTOCOL,
            OtaState.ENTERING_DFU,
            OtaState.SCANNING,
            OtaState.CONNECTING,
            OtaState.TRANSFERRING,
            OtaState.VALIDATING,
            OtaState.ACTIVATING,
            OtaState.COMPLETED,
        }:
            return
        if not self._connected and self._image_info is None:
            self._state = OtaState.IDLE
            return
        if self._connected and self._services:
            self._state = OtaState.SERVICE_CHECKED
            return
        if self._connected:
            self._state = OtaState.CONNECTED
            return
        if self._image_info is not None:
            self._state = OtaState.FILE_SELECTED
            return
        self._state = OtaState.IDLE

    def _detect_candidates(self, services: list[dict[str, Any]]) -> OtaServiceDiscoveryResult:
        result = OtaServiceDiscoveryResult()
        keywords = ("ota", "dfu")

        for service in services:
            service_uuid = str(service.get("uuid", ""))
            service_desc = str(service.get("description", "")).strip()
            description_lower = service_desc.lower()
            chars = list(service.get("characteristics", []) or [])
            result.total_service_count += 1
            result.total_char_count += len(chars)

            if service_uuid.lower() == DFU_SERVICE_UUID or any(key in description_lower for key in keywords):
                result.candidate_service_count += 1
                result.candidate_service_lines.append(
                    f"- {service_uuid} | {service_desc or '<无描述>'}"
                )

            for char in chars:
                char_uuid = str(char.get("uuid", ""))
                char_desc = str(char.get("description", "")).strip()
                char_desc_lower = char_desc.lower()
                if char_uuid.lower() in {DFU_DATA_UUID, DFU_CONTROL_POINT_UUID} or any(key in char_desc_lower for key in keywords):
                    result.candidate_char_count += 1
                    props = ",".join(char.get("properties", []) or [])
                    result.candidate_char_lines.append(
                        f"- {char_uuid} | {char_desc or '<无描述>'} | {props or '<无属性>'}"
                    )

        return result

    def _log(self, message: str) -> None:
        self._log_callback(message)
