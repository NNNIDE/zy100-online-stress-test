from __future__ import annotations

import tkinter as tk
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from .mailbox import UiMailbox
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk
from typing import Any

from ble.ble_manager import (
    BATTERY_LEVEL_UUID,
    BATTERY_SERVICE_UUID,
    LEGACY_OFFLINE_INTENT_FIX_CODE,
    BleManager,
    WINDOWS_GATT_CACHE_HINT,
)
from ble.calibration_protocol import (
    CALIBRATION_SERVICE_UUID,
    calibration_completion_detail_zh,
    calibration_failure_detail_zh,
    calibration_status_zh,
)
from ble.feature_config import (
    EFFECT_BLINK,
    EFFECT_BREATH,
    EFFECT_MARQUEE,
    EFFECT_SOLID,
    SPEED_CAPTURE_DEFAULT,
    SPEED_FAST,
    SPEED_SLOW,
    SPEED_STANDARD,
    TARGET_ALL,
    TARGET_LOGO,
    TARGET_NOTIFY,
    FeatureConfig,
)
from ble.online_store import OnlineSessionStore
from ble.offline_v2_store import OfflineV2SessionStore
from ble.zy100_protocol import (
    CMD_CLEAR_FLASH,
    CMD_ENTER_SHIPPING,
    CMD_OFFLINE_CAPTURE_START,
    CMD_OFFLINE_CAPTURE_STOP,
    CMD_PAUSE_CAPTURE,
    CMD_PING,
    CMD_STATE_NOTIFY,
    CMD_START_CAPTURE,
    CMD_TIME_SYNC,
    DEVICE_STATE_OFFLINE_CAPTURING,
    DEVICE_STATE_OFFLINE_FINALIZING,
    DEVICE_STATE_OFFLINE_INTENT_READY,
    DEVICE_STATE_WAIT_START,
    PREFERRED_DEVICE_NAME,
    ZY100_ACK_UUID,
    ZY100_COMMAND_UUID,
    ZY100_CONTROL_SERVICE_UUID,
    ZY100_DEVICE_INFO_UUID,
    ZY100_EXPORT_DATA_UUID,
    ZY100_MAX_PAIRED_CENTRALS,
    ZY100_MAX_SIMULTANEOUS_LINKS,
    command_name,
)
from ota import OtaManager, OtaState, RealtekDfuOtaClient


APP_VERSION_NAME = "v1.1.108"
DEVICE_INFO_VERSION_PLACEHOLDER = "Device Info: code=- fw=-"
DEVICE_CHANNEL_PLACEHOLDER = "Channel: -"

FEATURE_COLOR_PRESETS: tuple[tuple[str, tuple[int, int, int]], ...] = (
    ("蓝色", (0, 0, 255)),
    ("红色", (255, 0, 0)),
    ("绿色", (0, 255, 0)),
    ("黄色", (255, 255, 0)),
    ("青色", (0, 255, 255)),
    ("紫色", (128, 0, 255)),
    ("橙色", (255, 128, 0)),
    ("白色", (255, 255, 255)),
)
FEATURE_COLOR_RGB_BY_NAME = dict(FEATURE_COLOR_PRESETS)
FEATURE_COLOR_NAME_BY_RGB = {rgb: name for name, rgb in FEATURE_COLOR_PRESETS}
FEATURE_COLOR_CUSTOM_KEY = "__current_profile_color__"


class MainWindow:
    _SCAN_SECONDS = 3.0

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(f"ZY100 BLE 9ECA 采集与导出工具 {APP_VERSION_NAME}")
        self.root.geometry("1360x900")
        self.root.minsize(1120, 760)

        self.devices: list[dict[str, Any]] = []
        self._is_scanning = False
        self._is_closing = False
        self._mailbox = UiMailbox()
        self._online_list_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="online-list")
        self._online_list_future = None
        self._online_file_future = None
        self._online_file_operation = ""
        self._online_list_revision = 0
        self._online_list_requested_revision = 0
        self._online_list_refresh_pending = False
        self._close_thread = None
        self._current_view = "connect"
        self._command_found = False
        self._ack_subscribed = False
        self._export_subscribed = False
        self._rtc_sync_ok = False
        self._connection_user_id: int | None = None
        self._gatt_mismatch_active = False
        self._gatt_mismatch_blocked_addresses: set[str] = set()
        self._shown_gatt_mismatch_keys: set[str] = set()
        self._ota_reconnect_context: dict[str, Any] | None = None
        self._ota_reconnect_active = False
        self._ota_operation_active = False
        self._pending_ota_completion_message = ""
        self._export_blocking_commands = False
        self._offline_priority_active = False
        self._offline_priority_phase = "idle"
        self._business_ready = False
        self._feature_config_start_ready = False
        self._feature_config_supported = False
        self._feature_config_sync_active = False
        self._offline_control_supported = False
        self._offline_stop_any_supported = False
        self._connection_user_synced = False
        self._offline_start_supported = False
        self._last_device_state: int | None = None
        self._device_serial = ""
        self._device_serial_source = ""
        self._device_channel_short_received = False
        self._device_channel_short_valid = False
        self._start_command_pending = False
        self._offline_start_command_pending = False
        self._calibration_available = False
        self._calibration_gate_ready = False
        self._calibration_gate_state = "unknown"
        self._calibration_active = False
        self._last_calibration_record: dict[str, Any] | None = None
        self._last_calibration_diagnostics: dict[str, Any] | None = None
        self._calibration_operation_transaction: int | None = None
        self._calibration_started_monotonic: float | None = None
        self._calibration_last_progress = 0
        self._calibration_last_progress_changed_monotonic: float | None = None
        self._calibration_device_success_pending = False
        self._calibration_terminal_result: str | None = None
        self._calibration_popup_keys: set[tuple[int, str]] = set()
        self._sessions: list[dict[str, Any]] = []
        self._selected_session_key: str | None = None
        self._selected_session_keys: list[str] = []
        self.session_store = OfflineV2SessionStore()
        self._online_sessions: list[dict[str, Any]] = []
        self._selected_online_session_key: str | None = None
        self._selected_online_session_keys: list[str] = []
        self._online_last_list_refresh_ms = 0
        self.online_session_store = OnlineSessionStore()

        self.status_var = tk.StringVar(value="未连接")
        self.timeout_reason_var = tk.StringVar(value="")
        self._timeout_notice_key: tuple[int, int, int] | None = None
        self.connect_hint_var = tk.StringVar(value="扫描 ZP- 开头设备后，选择设备并手动连接")
        self.ack_subscription_var = tk.StringVar(value="ACK Notify：未订阅")
        self.rtc_sync_var = tk.StringVar(value="RTC Sync: not synced")
        self.control_status_var = tk.StringVar(value="ZY100 Control Service：未发现")
        self.last_sent_var = tk.StringVar(value="最近发送：-")
        self.command_result_var = tk.StringVar(value="指令状态：等待发送")
        self.device_version_var = tk.StringVar(value=DEVICE_INFO_VERSION_PLACEHOLDER)
        self.device_channel_var = tk.StringVar(value=DEVICE_CHANNEL_PLACEHOLDER)
        self.battery_level_var = tk.StringVar(value="设备电量：-")
        self.user_id_var = tk.StringVar(value="1")
        self.training_id_var = tk.StringVar(value="1")
        self.device_time_var = tk.StringVar(value="")
        self.calibration_capability_var = tk.StringVar(value="地磁校准服务：未发现")
        self.calibration_result_var = tk.StringVar(value="等待连接后开始地磁校准")
        self.calibration_status_var = tk.StringVar(value="校准状态：等待连接")
        self.calibration_info_var = tk.StringVar(value="记录：无数据")
        self.calibration_transfer_var = tk.StringVar(value="传输：-")
        self.calibration_diagnostics_status_var = tk.StringVar(
            value="诊断摘要：尚未读取"
        )
        self.calibration_diagnostics_result_var = tk.StringVar(
            value="等待校准完成后自动读取诊断摘要"
        )
        self.calibration_progress_var = tk.DoubleVar(value=0.0)

        self._gatt_vars = {
            "service_count": tk.StringVar(value="服务数量：0"),
            "char_count": tk.StringVar(value="特征数量：0"),
            "control": tk.StringVar(value="Control Service：未发现"),
            "command": tk.StringVar(value="Command：未发现"),
            "ack": tk.StringVar(value="ACK / Status：未发现"),
            "device_info": tk.StringVar(value="Device Info：未发现"),
            "battery": tk.StringVar(value="Battery Level：未发现/未订阅"),
            "notify": tk.StringVar(value="ACK Notify：未订阅"),
            "export": tk.StringVar(value="Export Data：未发现"),
            "calibration": tk.StringVar(value="Calibration Service：未发现"),
        }
        self._gatt_export_text = "当前暂无 GATT 数据。"

        self.transfer_status_var = tk.StringVar(value="传输状态：等待连接或历史操作")
        self.transfer_progress_var = tk.StringVar(value="当前传输：-")
        self.transfer_detail_var = tk.StringVar(value="本地 session：0 条")
        self.online_status_var = tk.StringVar(value="Online: idle")
        self.online_counts_var = tk.StringVar(value="800Hz 16B包数：0 | Record：0")
        self.online_bytes_var = tk.StringVar(value="Bytes: RAW 0 / EVENT 0 / SUMMARY 0")
        self.online_ack_var = tk.StringVar(value="ACK: RAW 0 / EVENT 0 / SUMMARY 0 / END 0")
        self.online_rate_var = tk.StringVar(value="Rate: 0 B/s")
        self.online_detail_var = tk.StringVar(value="Session: -")
        self.online_sessions_var = tk.StringVar(value="Online sessions: 0")
        self._online_chart_values: dict[str, int] = {"800Hz DATA": 0}
        self._online_last_redraw_ms = 0
        self._online_ui_active = False
        self._online_ui_diag_last_note_ns = 0

        self.ota_state_var = tk.StringVar(value="OTA状态：IDLE")
        self.ota_connection_var = tk.StringVar(value="DFU连接：未连接")
        self.ota_file_var = tk.StringVar(value="固件文件：未选择")
        self.ota_size_var = tk.StringVar(value="文件大小：-")
        self.ota_sha_var = tk.StringVar(value="SHA256：-")
        self.ota_preflight_var = tk.StringVar(value="前置检查：未开始")
        self.ota_protocol_var = tk.StringVar(value="协议状态：普通 BLE 进入 DFU + RTL8762D legacy DFU")
        self.ota_dfu_progress_var = tk.StringVar(value="DFU进度：等待开始")
        self.ota_progress_var = tk.DoubleVar(value=0.0)

        self.ota_manager = OtaManager(log_callback=self._post_ota_log)
        self.dfu_ota_client = RealtekDfuOtaClient(
            event_callback=self._post_ota_event,
            log_callback=self._post_ota_log,
        )
        self.ble_manager = BleManager(
            event_callback=self._post_event,
            log_callback=self._post_log,
        )

        self._command_buttons: list[ttk.Button] = []
        self._command_buttons_by_cmd: dict[int, ttk.Button] = {}

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._log(f"Application started {APP_VERSION_NAME}")
        self._set_gatt_placeholder()
        self._refresh_session_list()
        self._refresh_online_session_list()
        self._refresh_ota_panel()
        self._show_connect_view("请手动扫描并连接 ZP- 开头设备")
        self.root.after(20, self._pump_mailbox)
        self.root.after(1000, self._update_calibration_motion_hint)

    def _build_widgets(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=7)
        self.root.rowconfigure(2, weight=2)

        self._build_top_bar()
        self._build_content_views()
        self._build_log_panel()

    def _build_top_bar(self) -> None:
        top = ttk.Frame(self.root)
        top.grid(row=0, column=0, sticky="ew", padx=12, pady=8)
        top.columnconfigure(12, weight=1)

        ttk.Label(top, text="ZY100 BLE 9ECA 采集与导出", font=("Microsoft YaHei UI", 14, "bold")).grid(
            row=0, column=0, padx=(0, 12), sticky="w"
        )
        self.scan_button = ttk.Button(top, text="扫描 ZP 设备", command=lambda: self._start_scan("手动扫描"))
        self.scan_button.grid(row=0, column=1, padx=4)
        self.connect_button = ttk.Button(top, text="连接", state="disabled", command=self._on_connect)
        self.connect_button.grid(row=0, column=2, padx=4)
        self.disconnect_button = ttk.Button(top, text="断开", state="disabled", command=self._on_disconnect)
        self.disconnect_button.grid(row=0, column=3, padx=4)

        ttk.Label(top, text="连接状态：").grid(row=0, column=4, padx=(14, 4))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=5, sticky="w")
        ttk.Label(top, textvariable=self.timeout_reason_var, foreground="#975400",
                  font=("Microsoft YaHei UI", 10, "bold"), wraplength=1100).grid(
                      row=1, column=0, columnspan=13, sticky="ew", pady=(4, 0))
        ttk.Label(top, textvariable=self.ack_subscription_var).grid(row=0, column=6, padx=(14, 0), sticky="w")

    def _build_content_views(self) -> None:
        self.content_container = ttk.Frame(self.root)
        self.content_container.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 8))
        self.content_container.columnconfigure(0, weight=1)
        self.content_container.rowconfigure(0, weight=1)

        self.main_notebook = ttk.Notebook(self.content_container)
        self.main_notebook.grid(row=0, column=0, sticky="nsew")
        self.main_notebook.bind("<<NotebookTabChanged>>", self._on_main_tab_changed)

        self.device_view = ttk.Frame(self.main_notebook)
        self._build_device_view()

        self.command_view = ttk.Frame(self.main_notebook)
        self._build_command_view()

        self.transfer_view = ttk.Frame(self.main_notebook)
        self._build_transfer_view()

        self.online_view = ttk.Frame(self.main_notebook)
        self._build_online_view()
        from .online_stress_view import OnlineStressView
        self.stress_view = OnlineStressView(self.main_notebook, self.ble_manager,
                                            self._stress_identity, APP_VERSION_NAME)

        self.calibration_view = ttk.Frame(self.main_notebook)
        self._build_calibration_view()

        self.gatt_view = ttk.Frame(self.main_notebook)
        self._build_gatt_view()

        self.ota_view = ttk.Frame(self.main_notebook)
        self._build_ota_view()

        self.main_notebook.add(self.device_view, text="设备连接")
        self.feature_config_view = ttk.Frame(self.main_notebook)
        self._build_feature_config_view()
        self.main_notebook.add(self.feature_config_view, text="功能配置")
        self.main_notebook.add(self.command_view, text="离线采集")
        self.main_notebook.add(self.transfer_view, text="数据传输")
        self.main_notebook.add(self.online_view, text="在线采集")
        self.main_notebook.add(self.stress_view, text="链路压测")
        self.main_notebook.add(self.calibration_view, text="地磁校准")
        self.main_notebook.add(self.gatt_view, text="GATT 状态")
        self.main_notebook.add(self.ota_view, text="OTA 升级")
        self.main_notebook.tab(self.command_view, state="normal")
        self.main_notebook.tab(self.gatt_view, state="normal")

    def _build_device_view(self) -> None:
        self.device_view.columnconfigure(0, weight=1)
        self.device_view.rowconfigure(2, weight=1)

        hint_frame = tk.Frame(self.device_view, relief="groove", borderwidth=1)
        hint_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        hint_frame.columnconfigure(0, weight=1)
        tk.Label(
            hint_frame,
            textvariable=self.connect_hint_var,
            font=("Microsoft YaHei UI", 12, "bold"),
            anchor="center",
        ).grid(row=0, column=0, sticky="ew", padx=12, pady=10)

        connection_user_frame = ttk.Frame(self.device_view)
        connection_user_frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        connection_user_frame.columnconfigure(3, weight=1)
        ttk.Label(connection_user_frame, text="本次连接 user_id").grid(
            row=0, column=0, padx=(0, 6), sticky="w"
        )
        self.user_id_entry = ttk.Entry(
            connection_user_frame,
            width=16,
            textvariable=self.user_id_var,
        )
        self.user_id_entry.grid(row=0, column=1, padx=(0, 12), sticky="w")
        self.user_id_entry.bind("<FocusOut>", self._on_feature_config_user_changed)
        ttk.Label(
            connection_user_frame,
            text="连接前可修改，范围 1–0xFFFFFFFF；连接中修改仅在下次重连生效",
            foreground="#666666",
        ).grid(row=0, column=2, sticky="w")

        device_panel, device_body = self._create_panel(self.device_view, "ZP 设备列表", [])
        device_panel.grid(row=2, column=0, sticky="nsew")
        device_body.columnconfigure(0, weight=1)
        device_body.rowconfigure(0, weight=1)

        self.device_tree = ttk.Treeview(
            device_body,
            columns=("name", "address", "rssi"),
            show="headings",
            selectmode="browse",
        )
        self.device_tree.heading("name", text="设备名")
        self.device_tree.heading("address", text="地址")
        self.device_tree.heading("rssi", text="RSSI")
        self.device_tree.column("name", width=220, anchor="w")
        self.device_tree.column("address", width=360, anchor="w")
        self.device_tree.column("rssi", width=100, anchor="center")
        self.device_tree.grid(row=0, column=0, sticky="nsew")
        self.device_tree.bind("<<TreeviewSelect>>", self._on_device_selected)

        dev_y = ttk.Scrollbar(device_body, orient="vertical", command=self.device_tree.yview)
        dev_y.grid(row=0, column=1, sticky="ns")
        self.device_tree.configure(yscrollcommand=dev_y.set)

    def _build_feature_config_view(self) -> None:
        self.feature_config_view.columnconfigure(0, weight=1)
        panel, body = self._create_panel(
            self.feature_config_view,
            "按 user_id 保存的采集功能配置",
            [],
        )
        panel.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)

        self.feature_auto_capture_var = tk.BooleanVar(value=False)
        self.feature_training_led_var = tk.BooleanVar(value=True)
        self.feature_target_var = tk.StringVar(value="Notify 灯 (D7-D8)")
        self.feature_effect_var = tk.StringVar(value="呼吸")
        self.feature_color_var = tk.StringVar(value="蓝色")
        self._feature_custom_rgb: tuple[int, int, int] | None = None
        self.feature_brightness_var = tk.StringVar(value="100")
        self.feature_speed_var = tk.StringVar(value="采集默认")
        self.feature_reverse_var = tk.BooleanVar(value=False)
        self.feature_profile_var = tk.StringVar(value="本地档案：user_id=1（默认值）")
        self.feature_sync_var = tk.StringVar(value="设备同步：等待连接")
        self._feature_config_apply_failure = ""
        self.feature_pending_var = tk.StringVar(value="待同步参数：无")
        self.feature_confirmed_var = tk.StringVar(value="设备最后确认：暂无记录")
        self.feature_crc_var = tk.StringVar(value="CRC32：-")
        self.feature_generation_var = tk.StringVar(value="FTL generation：-")

        row = 0
        ttk.Label(
            body,
            text="自动采集与灯效按用户 ID 保存；设备断联后沿用最后成功写入的整套配置。\n运动自动采集开关由 Production10523 起支持；Production10524 起，成功确认包含硬件应用完成。",
            foreground="#666666",
        ).grid(row=row, column=0, columnspan=4, sticky="w", pady=(0, 10))
        row += 1
        self.feature_auto_capture_check = ttk.Checkbutton(
            body,
            text="运动唤醒并自动离线采集",
            variable=self.feature_auto_capture_var,
        )
        self.feature_auto_capture_check.grid(row=row, column=0, sticky="w", pady=4)
        self.feature_training_led_check = ttk.Checkbutton(
            body,
            text="训练灯",
            variable=self.feature_training_led_var,
            command=self._on_feature_config_constraint_changed,
        )
        self.feature_training_led_check.grid(row=row, column=1, sticky="w", pady=4)
        row += 1

        ttk.Label(body, text="灯区").grid(row=row, column=0, sticky="w", pady=4)
        self.feature_target_combo = ttk.Combobox(
            body,
            textvariable=self.feature_target_var,
            values=("Notify 灯 (D7-D8)", "Logo 灯 (D1-D6)", "全部灯 (D1-D8)"),
            state="readonly",
            width=24,
        )
        self.feature_target_combo.grid(row=row, column=1, sticky="w", pady=4)
        self.feature_target_combo.bind("<<ComboboxSelected>>", self._on_feature_config_constraint_changed)
        ttk.Label(body, text="灯效").grid(row=row, column=2, sticky="w", padx=(16, 6), pady=4)
        self.feature_effect_combo = ttk.Combobox(
            body,
            textvariable=self.feature_effect_var,
            values=("常亮", "闪烁", "呼吸", "跑马"),
            state="readonly",
            width=16,
        )
        self.feature_effect_combo.grid(row=row, column=3, sticky="w", pady=4)
        self.feature_effect_combo.bind("<<ComboboxSelected>>", self._on_feature_config_constraint_changed)
        row += 1

        ttk.Label(body, text="颜色").grid(row=row, column=0, sticky="nw", pady=4)
        palette = ttk.Frame(body)
        palette.grid(row=row, column=1, columnspan=3, sticky="w", pady=4)
        self.feature_color_buttons: list[tk.Button] = []
        self.feature_color_button_by_name: dict[str, tk.Button] = {}
        for idx, (name, color_rgb) in enumerate(FEATURE_COLOR_PRESETS):
            color_hex = self._feature_color_hex(color_rgb)
            button = tk.Button(
                palette,
                text=name,
                width=7,
                height=2,
                background=color_hex,
                activebackground=color_hex,
                foreground=self._feature_color_foreground(color_rgb),
                activeforeground=self._feature_color_foreground(color_rgb),
                command=lambda selected=name: self._select_feature_color(selected),
            )
            button.grid(row=idx // 4, column=idx % 4, padx=(0, 8), pady=(0, 6))
            self.feature_color_buttons.append(button)
            self.feature_color_button_by_name[name] = button
        self.feature_custom_color_button = tk.Button(
            palette,
            text="当前档案颜色",
            width=12,
            height=2,
            command=lambda: self._select_feature_color(FEATURE_COLOR_CUSTOM_KEY),
        )
        self.feature_custom_color_button.grid(row=2, column=0, columnspan=2, sticky="w")
        self.feature_custom_color_button.grid_remove()
        self.feature_color_buttons.append(self.feature_custom_color_button)
        row += 1

        ttk.Label(body, text="亮度").grid(row=row, column=0, sticky="w", pady=4)
        brightness = ttk.Frame(body)
        brightness.grid(row=row, column=1, sticky="w", pady=4)
        self.feature_brightness_spinbox = ttk.Spinbox(
            brightness,
            from_=1,
            to=100,
            width=6,
            textvariable=self.feature_brightness_var,
        )
        self.feature_brightness_spinbox.grid(row=0, column=0)
        ttk.Label(brightness, text="%（固件按安全上限 76 等比例换算）").grid(row=0, column=1, padx=(4, 0))
        ttk.Label(body, text="速度").grid(row=row, column=2, sticky="w", padx=(16, 6), pady=4)
        self.feature_speed_combo = ttk.Combobox(
            body,
            textvariable=self.feature_speed_var,
            values=("采集默认", "慢", "标准", "快"),
            state="readonly",
            width=24,
        )
        self.feature_speed_combo.grid(row=row, column=3, sticky="w", pady=4)
        row += 1
        self.feature_reverse_check = ttk.Checkbutton(
            body,
            text="跑马反向",
            variable=self.feature_reverse_var,
        )
        self.feature_reverse_check.grid(row=row, column=1, columnspan=3, sticky="w", pady=4)
        row += 1

        ttk.Separator(body).grid(row=row, column=0, columnspan=4, sticky="ew", pady=10)
        row += 1
        ttk.Label(body, textvariable=self.feature_profile_var).grid(row=row, column=0, columnspan=4, sticky="w", pady=2)
        row += 1
        ttk.Label(body, textvariable=self.feature_sync_var).grid(row=row, column=0, columnspan=4, sticky="w", pady=2)
        row += 1
        ttk.Label(body, textvariable=self.feature_pending_var, wraplength=760).grid(
            row=row, column=0, columnspan=4, sticky="w", pady=2)
        row += 1
        ttk.Label(body, textvariable=self.feature_confirmed_var, wraplength=760).grid(
            row=row, column=0, columnspan=4, sticky="w", pady=2)
        row += 1
        status = ttk.Frame(body)
        status.grid(row=row, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(status, textvariable=self.feature_crc_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.feature_generation_var).grid(row=0, column=1, sticky="w", padx=(24, 0))
        row += 1

        buttons = ttk.Frame(body)
        buttons.grid(row=row, column=0, columnspan=4, sticky="w", pady=(12, 0))
        self.feature_restore_button = ttk.Button(
            buttons, text="恢复默认", command=self._on_feature_config_restore_default
        )
        self.feature_restore_button.grid(row=0, column=0, padx=(0, 8))
        self.feature_save_button = ttk.Button(
            buttons, text="保存", command=lambda: self._on_feature_config_save(False)
        )
        self.feature_save_button.grid(row=0, column=1, padx=(0, 8))
        self.feature_save_sync_button = ttk.Button(
            buttons, text="保存并同步", command=lambda: self._on_feature_config_save(True)
        )
        self.feature_save_sync_button.grid(row=0, column=2)
        self.feature_reapply_button = ttk.Button(
            buttons, text="同值重新应用（WOM 保持开启）",
            command=self._on_feature_config_reapply,
        )
        self.feature_reapply_button.grid(row=0, column=3, padx=(12, 0))
        ttk.Label(
            body, wraplength=760,
            text="同值重新应用使用本次连接中设备已确认的配置，忽略未保存表单。仅 WOM 已开启且连接空闲时可用。",
        ).grid(row=row + 1, column=0, columnspan=4, sticky="w", pady=(10, 0))

        ttk.Label(
            body,
            text=(
                "默认：自动采集关、训练灯开、Notify 蓝色呼吸、100%、采集默认；"
                "600 ms 三角呼吸后约 4 s 全灭并关闭 LED_PWR。"
            ),
            foreground="#365F91",
        ).grid(row=row + 2, column=0, columnspan=4, sticky="w", pady=(14, 0))
        self._load_feature_config_for_current_user(show_warning=False)
        self._refresh_feature_config_controls()

    def _build_command_view(self) -> None:
        self.command_view.columnconfigure(0, weight=1)
        self.command_view.rowconfigure(0, weight=1)

        command_panel, command_body = self._create_panel(
            self.command_view, "Offline V2 离线采集控制", []
        )
        command_panel.grid(row=0, column=0, sticky="nsew")
        command_body.columnconfigure(0, weight=1)
        command_body.rowconfigure(3, weight=1)
        command_body.rowconfigure(4, weight=1)

        state_box = tk.Frame(command_body, relief="solid", borderwidth=1, background="#F7F7F7")
        state_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        state_box.columnconfigure(0, weight=1)
        for row, var in enumerate(
            (
                self.control_status_var,
                self.device_version_var,
                self.device_channel_var,
                self.battery_level_var,
                self.ack_subscription_var,
                self.rtc_sync_var,
                self.last_sent_var,
                self.command_result_var,
            )
        ):
            tk.Label(
                state_box,
                textvariable=var,
                anchor="w",
                justify="left",
                background="#F7F7F7",
                font=("Consolas", 10),
            ).grid(row=row, column=0, sticky="ew", padx=8, pady=(4, 0))

        input_row = ttk.Frame(command_body)
        input_row.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        input_row.columnconfigure(5, weight=1)
        ttk.Label(input_row, text="training_id").grid(
            row=0, column=0, padx=(0, 4), sticky="w"
        )
        ttk.Entry(input_row, width=12, textvariable=self.training_id_var).grid(row=0, column=1, padx=(0, 12), sticky="w")
        ttk.Label(input_row, text="device_time_ms").grid(row=0, column=2, padx=(0, 4), sticky="w")
        ttk.Entry(input_row, width=22, textvariable=self.device_time_var).grid(row=0, column=3, padx=(0, 8), sticky="w")
        ttk.Label(
            input_row,
            text="离线开始/停止自动使用当前连接 user ID，training/time 固定为 0",
            foreground="#666666",
        ).grid(
            row=0, column=4, sticky="w"
        )

        button_row = ttk.Frame(command_body)
        button_row.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        commands = (
            ("PING", CMD_PING),
            ("离线开始", CMD_OFFLINE_CAPTURE_START),
            ("离线停止", CMD_OFFLINE_CAPTURE_STOP),
            ("全量清除采集数据", CMD_CLEAR_FLASH),
            ("进入船运模式", CMD_ENTER_SHIPPING),
        )
        for col, (label, cmd) in enumerate(commands):
            button = ttk.Button(button_row, text=label, command=lambda cmd=cmd: self._on_send_command(cmd))
            button.grid(row=0, column=col, padx=(0, 8), sticky="w")
            self._command_buttons.append(button)
            self._command_buttons_by_cmd[cmd] = button
        self._set_command_buttons_enabled(True)

        ack_panel, ack_body = self._create_panel(command_body, "最近 ACK", [])
        ack_panel.grid(row=3, column=0, sticky="nsew", pady=(0, 8))
        ack_body.columnconfigure(0, weight=1)
        ack_body.rowconfigure(0, weight=1)
        self.ack_detail_text = tk.Text(ack_body, wrap="word", font=("Consolas", 10), height=10)
        self.ack_detail_text.grid(row=0, column=0, sticky="nsew")
        self.ack_detail_text.configure(state="disabled")
        ack_y = ttk.Scrollbar(ack_body, orient="vertical", command=self.ack_detail_text.yview)
        ack_y.grid(row=0, column=1, sticky="ns")
        self.ack_detail_text.configure(yscrollcommand=ack_y.set)

        history_panel, history_body = self._create_panel(
            command_body,
            "指令 / ACK 历史",
            [("清除", lambda: self._clear_text_widget(self.command_history_text))],
        )
        history_panel.grid(row=4, column=0, sticky="nsew")
        history_body.columnconfigure(0, weight=1)
        history_body.rowconfigure(0, weight=1)
        self.command_history_text = tk.Text(history_body, wrap="none", font=("Consolas", 9), height=10)
        self.command_history_text.grid(row=0, column=0, sticky="nsew")
        self.command_history_text.configure(state="disabled")
        hist_y = ttk.Scrollbar(history_body, orient="vertical", command=self.command_history_text.yview)
        hist_y.grid(row=0, column=1, sticky="ns")
        hist_x = ttk.Scrollbar(history_body, orient="horizontal", command=self.command_history_text.xview)
        hist_x.grid(row=1, column=0, sticky="ew")
        self.command_history_text.configure(yscrollcommand=hist_y.set, xscrollcommand=hist_x.set)

        self._set_text_widget(self.ack_detail_text, "等待 ACK Notify。")

    def _build_transfer_view(self) -> None:
        self.transfer_view.columnconfigure(0, weight=1)
        self.transfer_view.rowconfigure(0, weight=1)

        panel, body = self._create_panel(
            self.transfer_view,
            "BLE 数据传输 / 本地 Session",
            [
                ("刷新", self._refresh_session_list),
                ("下载", self._on_download_session),
                ("删除选中", self._on_delete_session),
            ],
        )
        panel.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        status_box = tk.Frame(body, relief="solid", borderwidth=1, background="#F7F7F7")
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        status_box.columnconfigure(0, weight=1)
        for row, var in enumerate((self.transfer_status_var, self.transfer_progress_var, self.transfer_detail_var)):
            tk.Label(
                status_box,
                textvariable=var,
                anchor="w",
                justify="left",
                background="#F7F7F7",
                font=("Consolas", 10),
            ).grid(row=row, column=0, sticky="ew", padx=8, pady=(4, 0))

        table_box = ttk.Frame(body)
        table_box.grid(row=1, column=0, sticky="nsew")
        table_box.columnconfigure(0, weight=1)
        table_box.rowconfigure(0, weight=1)
        columns = (
            "session_id",
            "generation",
            "owner_user_id",
            "start_time",
            "duration",
            "event_count",
            "bytes",
            "crc",
            "local_status",
            "device_status",
        )
        self.session_tree = ttk.Treeview(table_box, columns=columns, show="headings", selectmode="extended")
        headings = {
            "session_id": "session",
            "generation": "generation",
            "owner_user_id": "用户 ID",
            "start_time": "开始时间",
            "duration": "时长",
            "event_count": "事件数",
            "bytes": "逻辑字节",
            "crc": "CRC32",
            "local_status": "本机状态",
            "device_status": "设备状态",
        }
        widths = {
            "session_id": 90,
            "generation": 90,
            "owner_user_id": 120,
            "start_time": 170,
            "duration": 90,
            "event_count": 90,
            "bytes": 90,
            "crc": 100,
            "local_status": 110,
            "device_status": 120,
        }
        for col in columns:
            self.session_tree.heading(col, text=headings[col])
            self.session_tree.column(col, width=widths[col], anchor="w")
        self.session_tree.grid(row=0, column=0, sticky="nsew")
        self.session_tree.bind("<<TreeviewSelect>>", self._on_session_selected)
        y_scroll = ttk.Scrollbar(table_box, orient="vertical", command=self.session_tree.yview)
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll = ttk.Scrollbar(table_box, orient="horizontal", command=self.session_tree.xview)
        x_scroll.grid(row=1, column=0, sticky="ew")
        self.session_tree.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)

    def _build_online_view(self) -> None:
        self.online_view.columnconfigure(0, weight=1)
        self.online_view.rowconfigure(0, weight=1)

        panel, body = self._create_panel(
            self.online_view,
            "800Hz 在线连续采集",
            [
                ("START", lambda: self._on_send_command(CMD_START_CAPTURE)),
                ("STOP", lambda: self._on_send_command(CMD_PAUSE_CAPTURE)),
                ("Refresh", self._refresh_online_session_list),
                ("下载选中", self._download_online_session),
                ("删除选中", self._delete_online_session),
            ],
        )
        online_actions = getattr(panel, "action_buttons", {})
        self._online_device_action_buttons = [
            button
            for key in ("START", "STOP")
            if (button := online_actions.get(key)) is not None
        ]
        panel.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=2)
        body.rowconfigure(2, weight=3)

        status_box = tk.Frame(body, relief="solid", borderwidth=1, background="#F7F7F7")
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        status_box.columnconfigure(0, weight=1)
        for row, var in enumerate(
            (
                self.online_status_var,
                self.online_counts_var,
                self.online_bytes_var,
                self.online_ack_var,
                self.online_rate_var,
                self.online_detail_var,
                self.online_sessions_var,
            )
        ):
            tk.Label(
                status_box,
                textvariable=var,
                anchor="w",
                justify="left",
                background="#F7F7F7",
                font=("Consolas", 10),
            ).grid(row=row, column=0, sticky="ew", padx=8, pady=(4, 0))

        chart_box = ttk.Frame(body)
        chart_box.grid(row=1, column=0, sticky="nsew")
        chart_box.columnconfigure(0, weight=1)
        chart_box.rowconfigure(0, weight=1)
        self.online_chart_canvas = tk.Canvas(chart_box, height=220, background="#FFFFFF", highlightthickness=1)
        self.online_chart_canvas.grid(row=0, column=0, sticky="nsew")
        self.online_chart_canvas.bind("<Configure>", lambda _event: self._draw_online_chart())
        self._draw_online_chart()

        table_box = ttk.Frame(body)
        table_box.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        table_box.columnconfigure(0, weight=1)
        table_box.rowconfigure(0, weight=1)
        columns = (
            "session_id",
            "user_id",
            "training_id",
            "started_at",
            "status",
            "records",
            "bytes",
        )
        self.online_session_tree = ttk.Treeview(table_box, columns=columns, show="headings", selectmode="extended")
        headings = {
            "session_id": "session",
            "user_id": "user",
            "training_id": "training",
            "started_at": "设备Unix开始",
            "status": "status",
            "records": "16B包数",
            "bytes": "采样字节",
        }
        widths = {
            "session_id": 110,
            "user_id": 70,
            "training_id": 80,
            "started_at": 180,
            "status": 90,
            "records": 140,
            "bytes": 150,
        }
        for col in columns:
            self.online_session_tree.heading(col, text=headings[col])
            self.online_session_tree.column(col, width=widths[col], anchor="w")
        self.online_session_tree.grid(row=0, column=0, sticky="nsew")
        self.online_session_tree.bind("<<TreeviewSelect>>", self._on_online_session_selected)
        online_y = ttk.Scrollbar(table_box, orient="vertical", command=self.online_session_tree.yview)
        online_y.grid(row=0, column=1, sticky="ns")
        online_x = ttk.Scrollbar(table_box, orient="horizontal", command=self.online_session_tree.xview)
        online_x.grid(row=1, column=0, sticky="ew")
        self.online_session_tree.configure(yscrollcommand=online_y.set, xscrollcommand=online_x.set)

    def _build_calibration_view(self) -> None:
        self.calibration_view.columnconfigure(0, weight=1)
        self.calibration_view.rowconfigure(0, weight=1)

        panel, body = self._create_panel(self.calibration_view, "设备端地磁校准", [])
        panel.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(3, weight=1)
        body.rowconfigure(4, weight=1)

        action_row = ttk.Frame(body)
        action_row.grid(row=0, column=0, sticky="ew", pady=(2, 8))
        self.mag_cal_start_button = ttk.Button(
            action_row,
            text="开始地磁校准",
            command=self._on_start_mag_calibration,
            state="disabled",
        )
        self.mag_cal_start_button.grid(row=0, column=0, padx=(0, 8))
        self.mag_cal_request_button = ttk.Button(
            action_row,
            text="重新同步完整校准记录",
            command=self._on_request_calibration_record,
            state="disabled",
        )
        self.mag_cal_request_button.grid(row=0, column=1, padx=(0, 8))
        self.mag_cal_diagnostics_button = ttk.Button(
            action_row,
            text="重新读取诊断",
            command=self._on_request_mag_calibration_diagnostics,
            state="disabled",
        )
        self.mag_cal_diagnostics_button.grid(row=0, column=2, padx=(0, 8))

        status_box = tk.Frame(body, relief="solid", borderwidth=1, background="#F7F7F7")
        status_box.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        status_box.columnconfigure(0, weight=1)
        self.calibration_result_banner = tk.Label(
            status_box,
            textvariable=self.calibration_result_var,
            anchor="w",
            justify="left",
            background="#E5E7EB",
            foreground="#1F2937",
            font=("Microsoft YaHei UI", 11, "bold"),
            padx=10,
            pady=8,
        )
        self.calibration_result_banner.grid(row=0, column=0, sticky="ew")
        for row, var in enumerate(
            (
                self.calibration_capability_var,
                self.calibration_status_var,
                self.calibration_info_var,
                self.calibration_transfer_var,
                self.calibration_diagnostics_status_var,
            ),
            start=1,
        ):
            tk.Label(
                status_box,
                textvariable=var,
                anchor="w",
                justify="left",
                background="#F7F7F7",
                font=("Consolas", 10),
            ).grid(row=row, column=0, sticky="ew", padx=8, pady=(4, 0))
        self.mag_cal_progress = ttk.Progressbar(
            status_box,
            orient="horizontal",
            mode="determinate",
            variable=self.calibration_progress_var,
            maximum=100.0,
        )
        self.mag_cal_progress.grid(row=6, column=0, sticky="ew", padx=8, pady=(6, 8))

        guide = (
            "操作要求：整机远离手机、磁铁、电机、充电器和大块金属。连续缓慢转动球拍，"
            "可以一边滚转，一边改变俯仰和偏航；无需固定顺序或标准六面停留。\n"
            "动作进度达到 100% 即可停止。静止或重复同一旋转维度不会增加进度；"
            "30 秒内未达到 100% 才判定校准未完成，失败不会覆盖设备原有参数。"
        )
        tk.Label(
            body,
            text=guide,
            anchor="w",
            justify="left",
            wraplength=1180,
            foreground="#444444",
        ).grid(row=2, column=0, sticky="ew", pady=(0, 8))

        record_box = tk.Frame(body, relief="solid", borderwidth=1, background="#FBFBFB")
        record_box.grid(row=3, column=0, sticky="nsew")
        record_box.columnconfigure(0, weight=1)
        record_box.rowconfigure(0, weight=1)
        self.calibration_record_text = tk.Text(
            record_box, wrap="none", font=("Consolas", 10), height=16
        )
        self.calibration_record_text.grid(row=0, column=0, sticky="nsew")
        self.calibration_record_text.configure(state="disabled")
        cal_y = ttk.Scrollbar(record_box, orient="vertical", command=self.calibration_record_text.yview)
        cal_y.grid(row=0, column=1, sticky="ns")
        cal_x = ttk.Scrollbar(record_box, orient="horizontal", command=self.calibration_record_text.xview)
        cal_x.grid(row=1, column=0, sticky="ew")
        self.calibration_record_text.configure(yscrollcommand=cal_y.set, xscrollcommand=cal_x.set)
        self._set_text_widget(self.calibration_record_text, "等待设备校准记录。")

        diagnostics_box = tk.Frame(
            body, relief="solid", borderwidth=1, background="#FBFBFB"
        )
        diagnostics_box.grid(row=4, column=0, sticky="nsew", pady=(8, 0))
        diagnostics_box.columnconfigure(0, weight=1)
        diagnostics_box.rowconfigure(1, weight=1)
        self.calibration_diagnostics_banner = tk.Label(
            diagnostics_box,
            textvariable=self.calibration_diagnostics_result_var,
            anchor="w",
            justify="left",
            background="#E5E7EB",
            foreground="#1F2937",
            font=("Microsoft YaHei UI", 10, "bold"),
            padx=8,
            pady=6,
        )
        self.calibration_diagnostics_banner.grid(
            row=0, column=0, columnspan=2, sticky="ew"
        )
        self.calibration_diagnostics_text = tk.Text(
            diagnostics_box, wrap="none", font=("Consolas", 10), height=10
        )
        self.calibration_diagnostics_text.grid(
            row=1, column=0, sticky="nsew"
        )
        self.calibration_diagnostics_text.configure(state="disabled")
        diag_y = ttk.Scrollbar(
            diagnostics_box,
            orient="vertical",
            command=self.calibration_diagnostics_text.yview,
        )
        diag_y.grid(row=1, column=1, sticky="ns")
        self.calibration_diagnostics_text.configure(yscrollcommand=diag_y.set)
        self._set_text_widget(
            self.calibration_diagnostics_text, "等待设备诊断摘要。"
        )

    def _build_gatt_view(self) -> None:
        self.gatt_view.columnconfigure(0, weight=1)
        self.gatt_view.rowconfigure(0, weight=1)

        self.gatt_panel, gatt_body = self._create_panel(
            self.gatt_view,
            "GATT 服务与 ZY100 Control Service 检查",
            [("重新发现", self._on_gatt_rediscover_hint), ("下载", self._download_gatt_summary)],
        )
        self.gatt_panel.grid(row=0, column=0, sticky="nsew")
        gatt_body.columnconfigure(0, weight=1)
        gatt_body.rowconfigure(1, weight=1)

        info_box = tk.Frame(gatt_body, relief="solid", borderwidth=1, background="#F8F8F8")
        info_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        info_box.columnconfigure(0, weight=1)
        for row, key in enumerate(
            ("service_count", "char_count", "control", "command", "ack", "device_info", "battery", "export", "calibration", "notify")
        ):
            tk.Label(
                info_box,
                textvariable=self._gatt_vars[key],
                anchor="w",
                justify="left",
                background="#F8F8F8",
                font=("Consolas", 10, "bold"),
            ).grid(row=row, column=0, sticky="ew", padx=8, pady=(3, 0))

        list_box = tk.Frame(gatt_body, relief="solid", borderwidth=1, background="#F3F3F3")
        list_box.grid(row=1, column=0, sticky="nsew")
        list_box.columnconfigure(0, weight=1)
        list_box.rowconfigure(0, weight=1)
        self.gatt_list_text = tk.Text(list_box, wrap="none", relief="flat", borderwidth=0, font=("Consolas", 9))
        self.gatt_list_text.grid(row=0, column=0, sticky="nsew", padx=(8, 0), pady=(8, 6))
        self.gatt_list_text.configure(state="disabled")
        gatt_y = ttk.Scrollbar(list_box, orient="vertical", command=self.gatt_list_text.yview)
        gatt_y.grid(row=0, column=1, sticky="ns", pady=(8, 6))
        gatt_x = ttk.Scrollbar(list_box, orient="horizontal", command=self.gatt_list_text.xview)
        gatt_x.grid(row=1, column=0, sticky="ew", padx=(8, 0))
        self.gatt_list_text.configure(yscrollcommand=gatt_y.set, xscrollcommand=gatt_x.set)

    def _build_ota_view(self) -> None:
        self.ota_view.columnconfigure(0, weight=1)
        self.ota_view.rowconfigure(0, weight=1)

        self.ota_panel, ota_body = self._create_panel(self.ota_view, "RTL8762D DFU OTA", [])
        self.ota_panel.grid(row=0, column=0, sticky="nsew")
        ota_body.columnconfigure(0, weight=1)
        ota_body.rowconfigure(2, weight=1)
        ota_body.rowconfigure(3, weight=1)

        action_row = ttk.Frame(ota_body)
        action_row.grid(row=0, column=0, sticky="ew", pady=(2, 6))
        action_row.columnconfigure(5, weight=1)
        self.ota_select_button = ttk.Button(action_row, text="选择固件", command=self._on_select_ota_image)
        self.ota_select_button.grid(row=0, column=0, padx=(0, 6))
        self.ota_clear_button = ttk.Button(action_row, text="清除固件", command=self._on_clear_ota_image)
        self.ota_clear_button.grid(row=0, column=1, padx=(0, 6))
        self.ota_start_button = ttk.Button(action_row, text="进入 DFU 并升级", command=self._on_start_ota)
        self.ota_start_button.grid(row=0, column=2)
        ttk.Label(action_row, textvariable=self.ota_state_var).grid(row=0, column=3, padx=(14, 0), sticky="w")

        info_box = tk.Frame(ota_body, relief="solid", borderwidth=1, background="#F7F7F7")
        info_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        info_box.columnconfigure(1, weight=1)
        info_rows = [
            self.ota_connection_var,
            self.ota_file_var,
            self.ota_size_var,
            self.ota_sha_var,
            self.ota_preflight_var,
            self.ota_protocol_var,
            self.ota_dfu_progress_var,
        ]
        for idx, var in enumerate(info_rows):
            tk.Label(
                info_box,
                textvariable=var,
                anchor="w",
                justify="left",
                background="#F7F7F7",
                font=("Consolas", 10),
            ).grid(row=idx, column=0, sticky="w", padx=8, pady=(3, 0))

        progress_row = len(info_rows)
        ttk.Label(info_box, text="进度：").grid(row=progress_row, column=0, sticky="w", padx=8, pady=(4, 6))
        self.ota_progress = ttk.Progressbar(
            info_box,
            orient="horizontal",
            mode="determinate",
            variable=self.ota_progress_var,
            maximum=100.0,
        )
        self.ota_progress.grid(row=progress_row, column=1, sticky="ew", padx=(0, 8), pady=(4, 6))

        svc_card = tk.Frame(ota_body, relief="solid", borderwidth=1, background="#F3F3F3")
        svc_card.grid(row=2, column=0, sticky="nsew", pady=(0, 6))
        svc_card.columnconfigure(0, weight=1)
        svc_card.rowconfigure(1, weight=1)
        tk.Label(
            svc_card,
            text="DFU 服务 / 扫描详情",
            anchor="w",
            background="#F3F3F3",
            font=("Microsoft YaHei UI", 9),
        ).grid(row=0, column=0, sticky="w", padx=8, pady=(6, 2))
        self.ota_service_text = tk.Text(
            svc_card,
            wrap="none",
            relief="flat",
            borderwidth=0,
            background="#F3F3F3",
            font=("Consolas", 9),
        )
        self.ota_service_text.grid(row=1, column=0, sticky="nsew", padx=(8, 0), pady=(2, 6))
        self.ota_service_text.configure(state="disabled")
        svc_y = ttk.Scrollbar(svc_card, orient="vertical", command=self.ota_service_text.yview)
        svc_y.grid(row=1, column=1, sticky="ns", pady=(2, 6))
        self.ota_service_text.configure(yscrollcommand=svc_y.set)

        log_card = tk.Frame(ota_body, relief="solid", borderwidth=1, background="#FBFBFB")
        log_card.grid(row=3, column=0, sticky="nsew")
        log_card.columnconfigure(0, weight=1)
        log_card.rowconfigure(1, weight=1)
        tk.Label(
            log_card,
            text="OTA 日志",
            anchor="w",
            background="#FBFBFB",
            font=("Microsoft YaHei UI", 9),
        ).grid(row=0, column=0, sticky="w", padx=8, pady=(6, 2))
        self.ota_log_text = tk.Text(
            log_card,
            wrap="word",
            relief="flat",
            borderwidth=0,
            background="#FBFBFB",
            font=("Consolas", 9),
        )
        self.ota_log_text.grid(row=1, column=0, sticky="nsew", padx=(8, 0), pady=(2, 6))
        self.ota_log_text.configure(state="disabled")
        log_y = ttk.Scrollbar(log_card, orient="vertical", command=self.ota_log_text.yview)
        log_y.grid(row=1, column=1, sticky="ns", pady=(2, 6))
        self.ota_log_text.configure(yscrollcommand=log_y.set)

        self._set_text_widget(self.ota_service_text, "等待连接并发现服务后显示检测结果。")

    def _build_log_panel(self) -> None:
        self.log_panel, log_body = self._create_panel(
            self.root,
            "日志",
            [
                ("清除", lambda: self._clear_text_widget(self.log_text)),
                ("下载", lambda: self._download_text_widget(self.log_text, "导出日志", "ble_tool_logs.txt")),
            ],
        )
        self.log_panel.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 12))
        log_body.columnconfigure(0, weight=1)
        log_body.rowconfigure(0, weight=1)
        log_body.rowconfigure(1, weight=0)

        self.log_text = tk.Text(log_body, wrap="word", font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state="disabled")
        log_y = ttk.Scrollbar(log_body, orient="vertical", command=self.log_text.yview)
        log_y.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_y.set)

        ttk.Label(
            log_body,
            text=APP_VERSION_NAME,
            foreground="#666666",
            font=("Consolas", 9),
        ).grid(row=1, column=0, columnspan=2, sticky="e", padx=(0, 4), pady=(2, 0))

    def _create_panel(
        self,
        parent: tk.Widget,
        title: str,
        actions: list[tuple[str, Any]],
    ) -> tuple[tk.Frame, ttk.Frame]:
        panel = tk.Frame(parent, relief="groove", borderwidth=1)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)

        header = ttk.Frame(panel)
        header.grid(row=0, column=0, sticky="ew")
        ttk.Label(header, text=title, font=("Microsoft YaHei UI", 10, "bold")).grid(
            row=0, column=0, sticky="w", padx=(8, 4), pady=6
        )
        action_buttons: dict[str, ttk.Button] = {}
        for idx, (text, callback) in enumerate(actions, start=1):
            button = ttk.Button(header, text=text, width=8, command=callback)
            button.grid(
                row=0, column=idx, sticky="w", padx=(0, 4), pady=4
            )
            action_buttons[text] = button

        panel.action_buttons = action_buttons  # type: ignore[attr-defined]

        body = ttk.Frame(panel)
        body.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0, 4))
        return panel, body

    def _on_main_tab_changed(self, _: tk.Event[Any]) -> None:
        selected = self._get_selected_main_tab()
        if selected is getattr(self, "feature_config_view", None):
            self._load_feature_config_for_current_user(show_warning=True)
        if (
            selected is self.command_view
            or selected is self.gatt_view
            or selected is self.transfer_view
            or selected is self.online_view
            or selected is getattr(self, "feature_config_view", None)
            or selected is getattr(self, "calibration_view", None)
        ):
            self._current_view = "connected"
        elif selected is self.device_view:
            self._current_view = "connect" if self.disconnect_button.cget("state") == "disabled" else "connected"

    def _get_selected_main_tab(self) -> tk.Widget | None:
        try:
            selected_id = self.main_notebook.select()
            if not selected_id:
                return None
            return self.main_notebook.nametowidget(selected_id)
        except tk.TclError:
            return None

    def _feature_config_user_id(self) -> int:
        user_id = self._parse_uint(
            self.user_id_var.get(), "user_id", max_value=0xFFFFFFFF
        )
        if user_id == 0:
            raise ValueError("user_id 必须为 1–0xFFFFFFFF，不能为 0。")
        return user_id

    def _on_feature_config_user_changed(self, _: tk.Event[Any]) -> None:
        self._load_feature_config_for_current_user(show_warning=True)

    @staticmethod
    def _feature_color_hex(color_rgb: tuple[int, int, int]) -> str:
        return "#{:02X}{:02X}{:02X}".format(*color_rgb)

    @staticmethod
    def _feature_color_foreground(color_rgb: tuple[int, int, int]) -> str:
        red, green, blue = color_rgb
        luminance = red * 299 + green * 587 + blue * 114
        return "#000000" if luminance >= 128000 else "#FFFFFF"

    def _feature_color_rgb(self) -> tuple[int, int, int]:
        selected = self.feature_color_var.get()
        if selected in FEATURE_COLOR_RGB_BY_NAME:
            return FEATURE_COLOR_RGB_BY_NAME[selected]
        if selected == FEATURE_COLOR_CUSTOM_KEY and self._feature_custom_rgb is not None:
            return self._feature_custom_rgb
        raise ValueError("请选择一个灯效颜色")

    def _select_feature_color(self, selected: str) -> None:
        if selected not in FEATURE_COLOR_RGB_BY_NAME and selected != FEATURE_COLOR_CUSTOM_KEY:
            return
        if selected == FEATURE_COLOR_CUSTOM_KEY and self._feature_custom_rgb is None:
            return
        self.feature_color_var.set(selected)
        if selected in FEATURE_COLOR_RGB_BY_NAME and hasattr(
            self, "feature_custom_color_button"
        ):
            self.feature_custom_color_button.grid_remove()
            self._feature_custom_rgb = None
        self._refresh_feature_config_controls()

    def _render_feature_color(self, color_rgb: tuple[int, int, int]) -> None:
        preset_name = FEATURE_COLOR_NAME_BY_RGB.get(color_rgb)
        if preset_name is not None:
            self.feature_color_var.set(preset_name)
            self._feature_custom_rgb = None
            if hasattr(self, "feature_custom_color_button"):
                self.feature_custom_color_button.grid_remove()
            return
        self.feature_color_var.set(FEATURE_COLOR_CUSTOM_KEY)
        self._feature_custom_rgb = color_rgb
        if hasattr(self, "feature_custom_color_button"):
            color_hex = self._feature_color_hex(color_rgb)
            self.feature_custom_color_button.configure(
                background=color_hex,
                activebackground=color_hex,
                foreground=self._feature_color_foreground(color_rgb),
                activeforeground=self._feature_color_foreground(color_rgb),
            )
            self.feature_custom_color_button.grid()

    def _feature_config_from_form(self) -> FeatureConfig:
        target_map = {
            "Notify 灯 (D7-D8)": TARGET_NOTIFY,
            "Logo 灯 (D1-D6)": TARGET_LOGO,
            "全部灯 (D1-D8)": TARGET_ALL,
        }
        effect_map = {
            "常亮": EFFECT_SOLID,
            "闪烁": EFFECT_BLINK,
            "呼吸": EFFECT_BREATH,
            "跑马": EFFECT_MARQUEE,
        }
        speed_map = {
            "采集默认": SPEED_CAPTURE_DEFAULT,
            "慢": SPEED_SLOW,
            "标准": SPEED_STANDARD,
            "快": SPEED_FAST,
        }
        try:
            red, green, blue = self._feature_color_rgb()
            config = FeatureConfig(
                auto_capture=bool(self.feature_auto_capture_var.get()),
                training_led=bool(self.feature_training_led_var.get()),
                target=target_map[self.feature_target_var.get()],
                effect=effect_map[self.feature_effect_var.get()],
                red=red,
                green=green,
                blue=blue,
                brightness=self._parse_uint(
                    self.feature_brightness_var.get(), "亮度", max_value=100
                ),
                speed=speed_map[self.feature_speed_var.get()],
                reverse=bool(self.feature_reverse_var.get()),
            )
        except KeyError as exc:
            raise ValueError(f"未知配置选项：{exc}") from exc
        config.validate()
        return config

    def _render_feature_config(self, config: FeatureConfig) -> None:
        target_names = {
            TARGET_NOTIFY: "Notify 灯 (D7-D8)",
            TARGET_LOGO: "Logo 灯 (D1-D6)",
            TARGET_ALL: "全部灯 (D1-D8)",
        }
        effect_names = {
            EFFECT_SOLID: "常亮",
            EFFECT_BLINK: "闪烁",
            EFFECT_BREATH: "呼吸",
            EFFECT_MARQUEE: "跑马",
        }
        speed_names = {
            SPEED_CAPTURE_DEFAULT: "采集默认",
            SPEED_SLOW: "慢",
            SPEED_STANDARD: "标准",
            SPEED_FAST: "快",
        }
        self.feature_auto_capture_var.set(config.auto_capture)
        self.feature_training_led_var.set(config.training_led)
        self.feature_target_var.set(target_names[config.target])
        self.feature_effect_var.set(effect_names[config.effect])
        self._render_feature_color((config.red, config.green, config.blue))
        self.feature_brightness_var.set(str(config.brightness))
        self.feature_speed_var.set(speed_names[config.speed])
        self.feature_reverse_var.set(config.reverse)
        self._on_feature_config_constraint_changed()

    def _load_feature_config_for_current_user(self, *, show_warning: bool) -> None:
        if not hasattr(self, "feature_profile_var"):
            return
        try:
            user_id = self._feature_config_user_id()
            config, warning = self.ble_manager.load_feature_config_profile(user_id)
        except (ValueError, OSError) as exc:
            self.feature_profile_var.set(f"本地档案：无法载入（{exc}）")
            return
        self._render_feature_config(config)
        self.feature_profile_var.set(f"本地档案：user_id={user_id} 已载入")
        self.feature_crc_var.set(f"CRC32：0x{config.crc32(user_id):08X}")
        if warning:
            self.feature_profile_var.set(f"本地档案：{warning}")
            self._log(f"[FEATURE_CFG][WARN] user={user_id} {warning}")
            if show_warning:
                messagebox.showwarning("功能配置档案已回退默认", warning)

    def _on_feature_config_constraint_changed(self, _: tk.Event[Any] | None = None) -> None:
        if not hasattr(self, "feature_effect_var"):
            return
        effect = self.feature_effect_var.get()
        target = self.feature_target_var.get()
        if target == "Notify 灯 (D7-D8)":
            self.feature_effect_combo.configure(values=("常亮", "闪烁", "呼吸"))
            if effect == "跑马":
                self.feature_effect_var.set("呼吸")
                effect = "呼吸"
        else:
            self.feature_effect_combo.configure(values=("常亮", "闪烁", "呼吸", "跑马"))
        if effect == "常亮":
            self.feature_speed_var.set("采集默认")
        elif effect in {"闪烁", "跑马"}:
            if self.feature_speed_var.get() == "采集默认":
                self.feature_speed_var.set("标准")
        if effect != "跑马":
            self.feature_reverse_var.set(False)
        self._refresh_feature_config_controls()

    def _feature_config_edit_gate(self) -> tuple[bool, str]:
        if getattr(self, "_feature_config_sync_active", False):
            return False, "功能配置同步进行中"
        if getattr(self, "_start_command_pending", False) or getattr(
            self, "_offline_start_command_pending", False
        ):
            return False, "采集启动操作进行中"
        if getattr(self, "_calibration_active", False):
            return False, "地磁校准进行中"
        if getattr(self, "_ota_operation_active", False) or getattr(
            self, "_ota_reconnect_active", False
        ):
            return False, "OTA 操作进行中"
        try:
            user_id = self._feature_config_user_id()
        except ValueError as exc:
            return False, str(exc)
        gate = getattr(self.ble_manager, "feature_config_manual_gate", None)
        if not callable(gate):
            return False, "功能配置门禁不可用"
        return gate(user_id)

    def _refresh_feature_config_controls(self) -> None:
        if not hasattr(self, "feature_effect_combo"):
            return
        editable, _reason = self._feature_config_edit_gate()
        common_state = "normal" if editable else "disabled"
        combo_state = "readonly" if editable else "disabled"
        for widget in (
            self.feature_auto_capture_check,
            self.feature_training_led_check,
            *self.feature_color_buttons,
            self.feature_brightness_spinbox,
            self.feature_restore_button,
            self.feature_save_button,
            self.feature_save_sync_button,
        ):
            widget.configure(state=common_state)
        if hasattr(self, "feature_reapply_button"):
            reapply_allowed = False
            if editable:
                reapply_allowed, _ = self.ble_manager.feature_config_reapply_gate(
                    self._feature_config_user_id())
            self.feature_reapply_button.configure(
                state="normal" if reapply_allowed else "disabled")
        selected_color = self.feature_color_var.get()
        for name, button in self.feature_color_button_by_name.items():
            button.configure(
                relief="sunken" if name == selected_color else "raised",
                borderwidth=3 if name == selected_color else 1,
            )
        if hasattr(self, "feature_custom_color_button"):
            self.feature_custom_color_button.configure(
                relief=(
                    "sunken"
                    if selected_color == FEATURE_COLOR_CUSTOM_KEY
                    else "raised"
                ),
                borderwidth=(3 if selected_color == FEATURE_COLOR_CUSTOM_KEY else 1),
            )
        self.feature_target_combo.configure(state=combo_state)
        self.feature_effect_combo.configure(state=combo_state)
        effect = self.feature_effect_var.get()
        self.feature_speed_combo.configure(
            state="readonly" if editable and effect != "常亮" else "disabled"
        )
        self.feature_reverse_check.configure(
            state="normal" if editable and effect == "跑马" else "disabled"
        )

    def _on_feature_config_restore_default(self) -> None:
        allowed, reason = self._feature_config_edit_gate()
        if not allowed:
            self.feature_sync_var.set(f"设备同步：{reason}，当前页面只读")
            return
        self._render_feature_config(FeatureConfig())
        self.feature_sync_var.set("设备同步：已恢复表单默认值，尚未保存")

    def _on_feature_config_reapply(self) -> None:
        allowed, reason = self._feature_config_edit_gate()
        if not allowed:
            self.feature_sync_var.set(f"设备同步：{reason}")
            return
        previous_ready = self._feature_config_start_ready
        try:
            user_id = self._feature_config_user_id()
            self._feature_config_sync_active = True
            self._feature_config_start_ready = False
            self._refresh_command_button_state()
            self.ble_manager.reapply_confirmed_feature_config(user_id)
        except ValueError as exc:
            self._feature_config_sync_active = False
            self._feature_config_start_ready = previous_ready
            self._refresh_command_button_state()
            self.feature_sync_var.set(f"设备同步：{exc}")
            return
        self.feature_sync_var.set("设备同步：已提交同值重新应用，等待设备完成回执")

    def _on_feature_config_save(self, sync: bool) -> None:
        allowed, reason = self._feature_config_edit_gate()
        if not allowed:
            self.feature_sync_var.set(f"设备同步：{reason}，请回到连接空闲态后重试")
            messagebox.showinfo("功能配置门禁", reason)
            return
        try:
            user_id = self._feature_config_user_id()
            config = self._feature_config_from_form()
            if sync:
                self._feature_config_sync_active = True
                self._feature_config_start_ready = False
                self._refresh_command_button_state()
            path = self.ble_manager.save_feature_config_profile(
                user_id, config, sync=sync
            )
        except (ValueError, OSError) as exc:
            self._feature_config_sync_active = False
            self._refresh_command_button_state()
            messagebox.showerror("功能配置无效", str(exc))
            return
        self.feature_crc_var.set(f"CRC32：0x{config.crc32(user_id):08X}")
        if sync:
            self.feature_sync_var.set("设备同步：已提交；设备确认成功后保存本地档案")
            self.feature_profile_var.set(
                f"本地档案：user_id={user_id} 等待设备确认，目标路径 {path}"
            )
        else:
            self.feature_profile_var.set(f"本地档案：user_id={user_id} 已原子保存到 {path}")
            self.feature_sync_var.set("设备同步：仅保存本地档案")

    def _show_connect_view(self, hint: str) -> None:
        self._current_view = "connect"
        self.connect_hint_var.set(hint)
        selected = self._get_selected_main_tab()
        if (
            selected is not self.ota_view
            and selected is not self.transfer_view
            and selected is not getattr(self, "online_view", None)
            and selected is not getattr(self, "calibration_view", None)
        ):
            self.main_notebook.select(self.device_view)
        self.main_notebook.tab(self.command_view, state="normal")
        self.main_notebook.tab(self.gatt_view, state="normal")
        self.scan_button.configure(
            state=(
                "normal"
                if (not self.ota_manager.dfu_running and not getattr(self, "_is_scanning", False))
                else "disabled"
            )
        )
        self.connect_button.configure(
            state="normal" if (self._get_selected_index() is not None and not self._is_scanning) else "disabled"
        )
        self.disconnect_button.configure(state="disabled")
        self._set_command_buttons_enabled(True)

    def _show_connected_view(self) -> None:
        self._current_view = "connected"
        self.main_notebook.tab(self.command_view, state="normal")
        self.main_notebook.tab(self.gatt_view, state="normal")
        selected = self._get_selected_main_tab()
        if (
            selected is not self.ota_view
            and selected is not self.transfer_view
            and selected is not getattr(self, "online_view", None)
            and selected is not getattr(self, "calibration_view", None)
        ):
            self.main_notebook.select(self.command_view)
        self.scan_button.configure(state="normal" if not self.ota_manager.dfu_running else "disabled")
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self._refresh_command_button_state()

    def _start_scan(self, reason: str) -> None:
        if self.ota_manager.dfu_running:
            self._log("普通扫描已忽略：RTL8762D DFU OTA 正在进行中")
            return
        if self._is_scanning or self._is_closing:
            return
        disconnect_before_scan = self.disconnect_button.cget("state") == "normal" or bool(
            getattr(self.ota_manager, "connected", False)
        )
        if disconnect_before_scan:
            self._prepare_for_rescan_disconnect()
        self._is_scanning = True
        self.status_var.set(f"扫描中（{int(self._SCAN_SECONDS)}秒）")
        self.connect_hint_var.set("正在扫描 ZP- 开头设备")
        self.scan_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self._log(f"UI action: {reason}{'，扫描前断开当前连接' if disconnect_before_scan else ''}")
        self.ble_manager.scan(timeout=self._SCAN_SECONDS)

    def _prepare_for_rescan_disconnect(self) -> None:
        self._command_found = False
        self._ack_subscribed = False
        self._export_subscribed = False
        self._rtc_sync_ok = False
        self._last_device_state = None
        self._start_command_pending = False
        self._calibration_available = False
        self._calibration_active = False
        self._gatt_mismatch_active = False
        self._export_blocking_commands = False
        self.control_status_var.set("ZY100 Control Service：未发现")
        self.ack_subscription_var.set("ACK Notify：未订阅")
        self.rtc_sync_var.set("RTC Sync: not synced")
        self.command_result_var.set("指令状态：等待发送")
        self._reset_device_info_summary()
        self._reset_battery_summary()
        self.last_sent_var.set("最近发送：-")
        self.transfer_status_var.set("传输状态：未连接")
        self.transfer_progress_var.set("当前传输：-")
        self._reset_online_stream_status()
        self._reset_calibration_status(
            "等待重新连接",
            preserve_result=bool(
                getattr(self, "_calibration_terminal_result", None) is not None
                or getattr(self, "_calibration_device_success_pending", False)
            ),
        )
        self.ota_manager.set_connected(False)
        self._refresh_ota_panel()
        self._set_gatt_placeholder()
        self._refresh_command_button_state()
        self.disconnect_button.configure(state="disabled")

    def _on_connect(self) -> None:
        idx = self._get_selected_index()
        if idx is None:
            messagebox.showinfo("提示", "请先选择一个 ZP 设备。")
            return
        self._connect_device_at_index(idx, reason="手动连接")

    def _connect_device_at_index(self, idx: int, *, reason: str) -> None:
        if not (0 <= idx < len(self.devices)):
            messagebox.showinfo("提示", "请选择有效的 ZP 设备。")
            return
        try:
            user_id = self._parse_uint(self.user_id_var.get(), "user_id", max_value=0xFFFFFFFF)
            training_id = self._parse_uint(self.training_id_var.get(), "training_id", max_value=0xFFFFFFFF)
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        if user_id == 0:
            messagebox.showerror("参数错误", "user_id 必须为 1–0xFFFFFFFF，不能为 0。")
            return
        self._command_found = False
        self._ack_subscribed = False
        self._export_subscribed = False
        self._gatt_mismatch_active = False
        self._rtc_sync_ok = False
        self._feature_config_start_ready = False
        self._calibration_available = False
        self._calibration_active = False
        self.rtc_sync_var.set("RTC Sync: not synced")
        self.ack_subscription_var.set("ACK Notify：未订阅")
        self.control_status_var.set("ZY100 Control Service：未发现")
        self._reset_device_info_summary()
        self._reset_battery_summary()
        self._reset_calibration_status(
            "正在连接并发现 Calibration GATT",
            preserve_result=bool(
                getattr(self, "_calibration_terminal_result", None) is not None
                or getattr(self, "_calibration_device_success_pending", False)
            ),
        )
        self._set_gatt_placeholder()
        device = self.devices[idx]
        name = str(device.get("name", ""))
        address = str(device.get("address", ""))
        self.status_var.set("Connecting")
        self.connect_hint_var.set(f"{reason}：{name} {address}")
        self.connect_button.configure(state="disabled")
        self.scan_button.configure(state="disabled")
        if training_id <= 0:
            training_id = 1
        self._connection_user_id = user_id
        self._log(f"UI action: {reason} name={name} address={address}")
        self.ble_manager.connect(address, user_id=user_id, training_id=training_id)

    def _on_disconnect(self) -> None:
        self.status_var.set("断开中")
        self.ble_manager.disconnect()

    def _on_gatt_rediscover_hint(self) -> None:
        self._command_found = False
        self._ack_subscribed = False
        self._export_subscribed = False
        self._gatt_mismatch_active = True
        self._rtc_sync_ok = False
        self.status_var.set("GATT_MISMATCH")
        self.connect_hint_var.set("请按提示处理 Windows BLE GATT 缓存后重新扫描/连接。")
        self.control_status_var.set("ZY100 Control Service：等待重新发现")
        self.ack_subscription_var.set("ACK Notify：未订阅")
        self.rtc_sync_var.set("RTC Sync: not synced")
        self.command_result_var.set(f"GATT_MISMATCH：{WINDOWS_GATT_CACHE_HINT}")
        self._reset_battery_summary()
        self._set_command_buttons_enabled(True)
        self._set_gatt_placeholder()
        self._log(f"[BLE_GATT] rediscovery requested: {WINDOWS_GATT_CACHE_HINT}")
        self.ble_manager.disconnect()
        messagebox.showinfo("重新发现 GATT", WINDOWS_GATT_CACHE_HINT)

    def _stress_identity(self):
        user = self._parse_uint(self.user_id_var.get(), "user_id", max_value=0xFFFFFFFF)
        training = self._parse_uint(self.training_id_var.get(), "training_id", max_value=0xFFFFFFFF)
        if user <= 0 or training <= 0:
            raise ValueError("user_id 和 training_id 必须大于零")
        return user, training, self.device_version_var.get()

    def _on_send_command(self, cmd: int) -> None:
        if (
            getattr(self, "_offline_priority_active", False)
            and cmd not in {CMD_PING, CMD_OFFLINE_CAPTURE_STOP}
        ):
            self.command_result_var.set(
                "离线采集或自动上传进行中，当前设备操作已暂停"
            )
            return
        if cmd in {CMD_OFFLINE_CAPTURE_START, CMD_OFFLINE_CAPTURE_STOP} and not getattr(
            self, "_offline_control_supported", False
        ):
            self.command_result_var.set("当前设备需升级至声明 offctl=1 的固件")
            return
        if cmd == CMD_OFFLINE_CAPTURE_START and not getattr(
            self, "_offline_start_supported", False
        ):
            self.command_result_var.set(
                f"旧固件存在 0x0E 卡死风险，BLE 离线开始仅支持 {LEGACY_OFFLINE_INTENT_FIX_CODE} 及以上；请使用实体双击"
            )
            return
        if cmd == CMD_OFFLINE_CAPTURE_START and (
            not getattr(self, "_business_ready", False)
            or getattr(self, "_last_device_state", None) != DEVICE_STATE_WAIT_START
            or getattr(self, "_offline_start_command_pending", False)
        ):
            self.command_result_var.set(
                "离线开始等待 Business Ready / WAIT_START，且不能有 pending 任务"
            )
            return
        if cmd == CMD_OFFLINE_CAPTURE_STOP and not (
            getattr(self, "_offline_stop_any_supported", False)
            or getattr(self, "_connection_user_synced", False)
        ):
            self.command_result_var.set("旧固件尚未确认用户身份，无法暂停；请等待结束或升级固件")
            return
        if cmd == CMD_OFFLINE_CAPTURE_STOP and (
            getattr(self, "_last_device_state", None)
            not in {
                DEVICE_STATE_OFFLINE_INTENT_READY,
                DEVICE_STATE_OFFLINE_CAPTURING,
                DEVICE_STATE_OFFLINE_FINALIZING,
            }
        ):
            self.command_result_var.set(
                "离线停止仅在 OFFLINE_INTENT_READY/OFFLINE_CAPTURING/FINALIZING 状态开放"
            )
            return
        shipping_state = getattr(self, "_last_device_state", None)
        if cmd == CMD_ENTER_SHIPPING and (
            shipping_state not in {None, 0, DEVICE_STATE_WAIT_START}
            or getattr(self, "_pending_command_active", False)
        ):
            self._log_diagnostic(
                "[SHIPPING_GUARD] blocked "
                f"state={shipping_state if shipping_state is not None else 'unknown'} "
                f"pending={int(bool(getattr(self, '_pending_command_active', False)))} "
                f"serial_source={getattr(self, '_device_serial_source', '') or 'none'}"
            )
            self.command_result_var.set("船运模式仅允许在空闲/Standby 且无 pending 任务时执行")
            return
        if self._calibration_active and cmd != CMD_PING:
            self.command_result_var.set("地磁校准进行中，仅允许 PING")
            return
        if getattr(self, "_feature_config_sync_active", False) and cmd != CMD_PING:
            self.command_result_var.set("功能配置同步进行中，仅允许 PING")
            return
        if getattr(self, "_ota_operation_active", False) and cmd != CMD_PING:
            self.command_result_var.set("OTA 操作进行中，仅允许 PING")
            return
        if cmd == CMD_START_CAPTURE and not getattr(
            self, "_calibration_gate_ready", False
        ):
            self.command_result_var.set("校准摘要或完整记录尚未确认，START_CAPTURE 已锁定")
            return
        if cmd == CMD_START_CAPTURE and not getattr(
            self, "_feature_config_start_ready", True
        ):
            self.command_result_var.set("功能配置尚未确认，START_CAPTURE 已锁定")
            return
        if cmd == CMD_START_CAPTURE and (
            getattr(self, "_last_device_state", None) != DEVICE_STATE_WAIT_START
            or getattr(self, "_start_command_pending", False)
        ):
            self.command_result_var.set("START waiting for device WAIT_START")
            return
        if cmd == CMD_START_CAPTURE and getattr(self, "_online_file_future", None) is not None:
            self.command_result_var.set("后台文件处理中，完成后可开始在线采集")
            return
        if (
            cmd == CMD_PAUSE_CAPTURE
            and getattr(self, "online_view", None) is not None
            and self._get_selected_main_tab() is self.online_view
            and not getattr(self, "_online_ui_active", False)
        ):
            self.command_result_var.set("ONLINE STOP ignored: stream inactive")
            self._log("[ONLINE_STATE] stop ignored inactive_ui")
            return

        try:
            if cmd in {CMD_OFFLINE_CAPTURE_START, CMD_OFFLINE_CAPTURE_STOP}:
                training_id = 0
                device_time_ms = 0
            elif cmd == CMD_ENTER_SHIPPING:
                training_id = 0
                device_time_ms = None
            else:
                training_id = self._parse_uint(
                    self.training_id_var.get(), "training_id", max_value=0xFFFFFFFF
                )
                time_text = self.device_time_var.get().strip()
                device_time_ms = None if not time_text else self._parse_uint(
                    time_text,
                    "device_time_ms",
                    max_value=0xFFFFFFFFFFFFFFFF,
                )
            user_id = self._connection_user_id
            if user_id is None:
                user_id = self._parse_uint(
                    self.user_id_var.get(),
                    "user_id",
                    max_value=0xFFFFFFFF,
                )
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        if cmd == CMD_CLEAR_FLASH:
            serial = self._device_serial.strip()
            if not serial:
                messagebox.showerror(
                    "CLEAR_FLASH",
                    "未读取到当前设备完整 SN，禁止执行全量 Flash 清理。请重新连接并确认 MFG Info。",
                )
                return
            typed = simpledialog.askstring(
                "CLEAR_FLASH 全量采集数据清理",
                "此操作不可撤销。\n\n"
                "将清除：\n"
                "- 外置 Flash 0x000000..0x3F7FFF（Offline V2、Journal A/B、Online Spool）\n"
                "- 内置 USER_DATA1 0x008DE000..0x008FBFFF（离线/旧采集 scratch）\n"
                "- 重新建立 0x008DC000/0x008DD000 READY marker\n\n"
                "将保留：\n"
                "- 外置 Flash 0x3F8000..0x3FFFFF\n"
                "- System Info/配对 0x008FC000..0x008FDFFF\n"
                "- MFG 0x008FE000..0x008FFFFF、校准、OTA 和固件区\n\n"
                f"请输入当前设备完整 SN 以确认：{serial}",
                parent=self.root,
            )
            if typed is None:
                return
            if typed.strip() != serial:
                messagebox.showerror("CLEAR_FLASH", "SN 不匹配，未发送清理指令。")
                return

        if cmd == CMD_ENTER_SHIPPING:
            serial = self._device_serial.strip()
            self._log_diagnostic(
                "[SHIPPING_GUARD] serial_check "
                f"value={serial or '<empty>'} "
                f"source={getattr(self, '_device_serial_source', '') or 'none'}"
            )
            if not serial:
                messagebox.showerror(
                    "ENTER_SHIPPING",
                    "未读取到当前设备完整 SN，禁止执行船运模式。请重新连接并确认 MFG Info。",
                )
                return
            typed = simpledialog.askstring(
                "ENTER_SHIPPING 进入船运模式",
                "该操作会让设备进入低功耗船运模式，设备随后断开连接。\n\n"
                "恢复方式：插入 VIN 后设备会自动开机进入正常 ZP-* 运行态，\n"
                "主机不会自动重连。\n\n"
                f"请输入当前设备完整 SN 以确认：{serial}",
                parent=self.root,
            )
            if typed is None:
                return
            if typed.strip() != serial:
                messagebox.showerror("ENTER_SHIPPING", "SN 不匹配，未发送船运指令。")
                return

        self.command_result_var.set(f"指令状态：正在发送 {command_name(cmd)}")
        if user_id <= 0:
            messagebox.showerror("参数错误", "user_id 必须为 1–0xFFFFFFFF，不能为 0。")
            return
        if training_id <= 0 and cmd not in {
            CMD_OFFLINE_CAPTURE_START,
            CMD_OFFLINE_CAPTURE_STOP,
            CMD_ENTER_SHIPPING,
        }:
            training_id = 1
        if cmd == CMD_START_CAPTURE and not self._rtc_sync_ok:
            self.command_result_var.set("START needs RTC sync; syncing before START_CAPTURE")
        if cmd == CMD_START_CAPTURE:
            self._start_command_pending = True
            self._refresh_command_button_state()
        elif cmd == CMD_OFFLINE_CAPTURE_START:
            self._offline_start_command_pending = True
            self._refresh_command_button_state()
        self.ble_manager.send_zy100_command(
            cmd=cmd,
            user_id=user_id,
            training_id=training_id,
            device_time_ms=device_time_ms,
        )

    def _on_start_mag_calibration(self) -> None:
        if getattr(self, "_offline_priority_active", False):
            messagebox.showinfo(
                "地磁校准",
                "离线采集或自动上传进行中，地磁校准已暂停。",
            )
            return
        if not getattr(self, "_calibration_gate_ready", False):
            messagebox.showinfo(
                "地磁校准",
                "校准摘要或完整记录尚未确认，当前入口已锁定。请重新同步或重新连接。",
            )
            return
        if (
            not getattr(self, "_business_ready", False)
            or getattr(self, "_feature_config_sync_active", False)
            or getattr(self, "_ota_operation_active", False)
            or getattr(self, "_start_command_pending", False)
            or getattr(self, "_offline_start_command_pending", False)
        ):
            messagebox.showinfo(
                "地磁校准",
                "当前存在采集、配置或 OTA 独占操作，地磁校准仅允许在连接空闲态启动。",
            )
            return
        if not self._calibration_available:
            messagebox.showinfo("地磁校准", "当前设备未提供 Calibration GATT 服务。")
            return
        if getattr(self, "_calibration_active", False):
            return
        if self._last_device_state != DEVICE_STATE_WAIT_START:
            messagebox.showinfo("地磁校准", "设备必须处于 WAIT_START 空闲态，不能在采集或导出期间启动。")
            return
        if not messagebox.askyesno(
            "开始地磁校准",
            "请确认设备已远离手机、磁铁、电机、充电器和大块金属。\n\n"
            "开始后可一边滚转，一边改变俯仰和偏航；动作进度达到 100% 即可停止。是否继续？",
        ):
            return
        self._calibration_active = True
        self._calibration_operation_transaction = None
        self._calibration_started_monotonic = time.monotonic()
        self._calibration_last_progress = 0
        self._calibration_last_progress_changed_monotonic = time.monotonic()
        self._calibration_device_success_pending = False
        self._calibration_terminal_result = None
        self._last_calibration_diagnostics = None
        self.calibration_progress_var.set(0.0)
        self.calibration_diagnostics_status_var.set(
            "诊断摘要：等待本次校准结束"
        )
        self._set_calibration_diagnostics_banner(
            "等待本次校准完成后自动读取诊断摘要", "idle"
        )
        if hasattr(self, "calibration_diagnostics_text"):
            self._set_text_widget(
                self.calibration_diagnostics_text, "等待本次校准诊断摘要。"
            )
        self.calibration_status_var.set("校准状态：命令发送中，等待设备接受")
        self._set_calibration_banner("校准命令发送中，请准备开始三维 8 字运动", "working")
        self._refresh_calibration_controls()
        self._refresh_command_button_state()
        self._set_ota_controls_enabled(False)
        self.ble_manager.start_mag_calibration()

    def _on_request_calibration_record(self) -> None:
        if getattr(self, "_offline_priority_active", False):
            self.calibration_transfer_var.set(
                "传输：离线采集或自动上传进行中，校准记录同步已暂停"
            )
            return
        if not self._calibration_available:
            return
        self.calibration_transfer_var.set("传输：正在同步并确认完整校准记录")
        self.ble_manager.request_calibration_record()

    def _on_request_mag_calibration_diagnostics(self) -> None:
        if getattr(self, "_offline_priority_active", False):
            self.calibration_diagnostics_status_var.set(
                "诊断摘要：离线采集或自动上传进行中，诊断已暂停"
            )
            return
        if not self._calibration_available or self._calibration_active:
            return
        self.calibration_diagnostics_status_var.set("诊断摘要：正在请求")
        self.ble_manager.request_mag_calibration_diagnostics()

    @staticmethod
    def _parse_uint(raw: str, field: str, *, max_value: int) -> int:
        text = raw.strip()
        if not text:
            raise ValueError(f"{field} 不能为空")
        value = int(text, 16) if text.lower().startswith("0x") else int(text, 10)
        if not (0 <= value <= max_value):
            raise ValueError(f"{field} 超出范围：0..{max_value}")
        return value

    def _set_command_buttons_enabled(self, enabled: bool) -> None:
        self._refresh_online_device_action_buttons()
        buttons = list(getattr(self, "_command_buttons", []))
        if not buttons:
            buttons = list(getattr(self, "_command_buttons_by_cmd", {}).values())
        if getattr(self, "_ota_reconnect_active", False):
            for button in buttons:
                button.configure(state="disabled")
            return
        if getattr(self, "_offline_priority_active", False):
            for cmd, button in getattr(self, "_command_buttons_by_cmd", {}).items():
                allow = (
                    cmd == CMD_OFFLINE_CAPTURE_STOP
                    and getattr(self, "_offline_control_supported", False)
                    and (getattr(self, "_offline_stop_any_supported", False)
                         or getattr(self, "_connection_user_synced", False))
                    and getattr(self, "_last_device_state", None) in {
                        DEVICE_STATE_OFFLINE_INTENT_READY, DEVICE_STATE_OFFLINE_CAPTURING,
                        DEVICE_STATE_OFFLINE_FINALIZING,
                    }
                )
                button.configure(state="normal" if allow else "disabled")
            return
        for button in buttons:
            button.configure(state="normal")
        if getattr(self, "_calibration_active", False):
            for cmd, button in getattr(self, "_command_buttons_by_cmd", {}).items():
                if cmd != CMD_PING:
                    button.configure(state="disabled")
        if getattr(self, "_feature_config_sync_active", False):
            for cmd, button in getattr(self, "_command_buttons_by_cmd", {}).items():
                if cmd != CMD_PING:
                    button.configure(state="disabled")
        start_button = getattr(self, "_command_buttons_by_cmd", {}).get(
            CMD_START_CAPTURE
        )
        if start_button is not None and (
            not getattr(self, "_business_ready", False)
            or not getattr(self, "_calibration_gate_ready", False)
            or not getattr(self, "_feature_config_start_ready", True)
            or getattr(self, "_last_device_state", None) != DEVICE_STATE_WAIT_START
            or getattr(self, "_start_command_pending", False)
            or getattr(self, "_online_file_future", None) is not None
        ):
            start_button.configure(state="disabled")
        offline_start_button = getattr(self, "_command_buttons_by_cmd", {}).get(
            CMD_OFFLINE_CAPTURE_START
        )
        if offline_start_button is not None:
            offline_start_button.configure(
                state="normal"
                if getattr(self, "_offline_start_supported", False)
                and getattr(self, "_business_ready", False)
                and getattr(self, "_feature_config_start_ready", False)
                and getattr(self, "_last_device_state", None)
                == DEVICE_STATE_WAIT_START
                and not getattr(self, "_offline_start_command_pending", False)
                and not getattr(self, "_pending_command_active", False)
                else "disabled"
            )
        offline_stop_button = getattr(self, "_command_buttons_by_cmd", {}).get(
            CMD_OFFLINE_CAPTURE_STOP
        )
        if offline_stop_button is not None:
            offline_stop_button.configure(
                state="normal"
                if getattr(self, "_offline_control_supported", False)
                and (getattr(self, "_offline_stop_any_supported", False)
                     or getattr(self, "_connection_user_synced", False))
                and getattr(self, "_last_device_state", None) in {
                    DEVICE_STATE_OFFLINE_INTENT_READY, DEVICE_STATE_OFFLINE_CAPTURING,
                    DEVICE_STATE_OFFLINE_FINALIZING,
                }
                else "disabled"
            )
        shipping_button = getattr(self, "_command_buttons_by_cmd", {}).get(
            CMD_ENTER_SHIPPING
        )
        if shipping_button is not None:
            shipping_state = getattr(self, "_last_device_state", None)
            shipping_button.configure(
                state=(
                    "normal"
                    if getattr(self, "_business_ready", False)
                    and shipping_state in {None, 0, DEVICE_STATE_WAIT_START}
                    and not getattr(self, "_pending_command_active", False)
                    and not getattr(self, "_shipping_operation_active", False)
                    and not getattr(self, "_start_command_pending", False)
                    and not getattr(self, "_offline_start_command_pending", False)
                    else "disabled"
                )
            )

    def _refresh_online_device_action_buttons(self) -> None:
        blocked = bool(
            getattr(self, "_offline_priority_active", False)
            or getattr(self, "_ota_reconnect_active", False)
        )
        for button in getattr(self, "_online_device_action_buttons", []):
            button.configure(state="disabled" if blocked else "normal")

    def _refresh_command_button_state(self) -> None:
        self._set_command_buttons_enabled(True)
        self._refresh_feature_config_controls()

    def _refresh_calibration_controls(self) -> None:
        available = getattr(self, "_calibration_available", False)
        active = getattr(self, "_calibration_active", False)
        offline_priority = getattr(self, "_offline_priority_active", False)
        idle = getattr(self, "_last_device_state", None) == DEVICE_STATE_WAIT_START
        if hasattr(self, "mag_cal_start_button"):
            self.mag_cal_start_button.configure(
                state="normal"
                if available
                and getattr(self, "_calibration_gate_ready", False)
                and getattr(self, "_business_ready", False)
                and idle
                and not active
                and not offline_priority
                and not getattr(self, "_feature_config_sync_active", False)
                and not getattr(self, "_ota_operation_active", False)
                and not getattr(self, "_start_command_pending", False)
                and not getattr(self, "_offline_start_command_pending", False)
                else "disabled"
            )
        if hasattr(self, "mag_cal_request_button"):
            self.mag_cal_request_button.configure(
                state="normal"
                if available and not active and not offline_priority
                else "disabled"
            )
        if hasattr(self, "mag_cal_diagnostics_button"):
            self.mag_cal_diagnostics_button.configure(
                state="normal"
                if available and not active and not offline_priority
                else "disabled"
            )

    def _on_select_ota_image(self) -> None:
        dialog_kwargs: dict[str, Any] = {
            "title": "选择 OTA 固件",
            "filetypes": [("固件文件", "*.bin"), ("所有文件", "*.*")],
        }
        firmware_dir = self.ota_manager.get_firmware_directory()
        if firmware_dir:
            dialog_kwargs["initialdir"] = firmware_dir

        path = filedialog.askopenfilename(**dialog_kwargs)
        if not path:
            return
        try:
            self.ota_manager.select_image(path)
        except ValueError as exc:
            messagebox.showerror("错误", str(exc))
        finally:
            self._refresh_ota_panel()

    def _on_clear_ota_image(self) -> None:
        if self.ota_manager.dfu_running:
            messagebox.showinfo("OTA", "OTA 正在进行中，不能清除固件。")
            return
        self.ota_manager.clear_image()
        self.ota_progress_var.set(0.0)
        self._refresh_ota_panel()

    def _on_start_ota(self) -> None:
        if getattr(self, "_offline_priority_active", False):
            messagebox.showinfo(
                "OTA",
                "离线采集或自动上传进行中，OTA 已暂停。",
            )
            return
        if getattr(self, "_calibration_active", False):
            messagebox.showinfo("OTA", "地磁校准进行中，不能进入 OTA。")
            return
        if self._is_scanning:
            messagebox.showinfo("OTA", "普通蓝牙扫描正在进行中，请等待扫描结束后再启动 DFU OTA。")
            return
        if self._can_send_ota_dfu_entry_command():
            if (
                not getattr(self, "_business_ready", False)
                or getattr(self, "_last_device_state", None)
                != DEVICE_STATE_WAIT_START
                or getattr(self, "_feature_config_sync_active", False)
                or getattr(self, "_start_command_pending", False)
                or getattr(self, "_offline_start_command_pending", False)
            ):
                messagebox.showinfo(
                    "OTA",
                    "OTA 仅允许在普通蓝牙 Business Ready / WAIT_START 且无采集、校准或配置任务时启动。",
                )
                return
            self._remember_ota_reconnect_context()
            result = self.ota_manager.start_enter_dfu_command()
            self.ota_progress_var.set(0.0)
            self._refresh_ota_panel()
            if not result.ok:
                details = "\n".join(result.issues) if result.issues else "未知原因"
                messagebox.showwarning("OTA", f"{result.message}\n\n{details}")
                return

            self._ota_operation_active = True
            self._set_ota_controls_enabled(False)
            self.scan_button.configure(state="disabled")
            self.connect_button.configure(state="disabled")
            self.disconnect_button.configure(state="disabled")
            self._set_command_buttons_enabled(True)
            self._refresh_feature_config_controls()
            self._refresh_calibration_controls()
            self.ble_manager.enter_ota_dfu_mode()
            return

        self._start_dfu_upgrade_current_image(after_entry_command=False)

    def _remember_ota_reconnect_context(self) -> None:
        identity: dict[str, Any] = {}
        get_identity = getattr(self.ble_manager, "get_connection_identity", None)
        if callable(get_identity):
            identity = dict(get_identity() or {})

        idx = self._get_selected_index() if hasattr(self, "device_tree") else None
        devices = list(getattr(self, "devices", []))
        if idx is not None and 0 <= idx < len(devices):
            selected = devices[idx]
            identity.setdefault("address", str(selected.get("address") or ""))
            identity.setdefault("name", str(selected.get("name") or ""))

        try:
            user_id = self._parse_uint(self.user_id_var.get(), "user_id", max_value=0xFFFFFFFF)
            training_id = self._parse_uint(self.training_id_var.get(), "training_id", max_value=0xFFFFFFFF)
        except (AttributeError, ValueError):
            user_id = getattr(self, "_connection_user_id", None) or 1
            training_id = 1

        user_id = getattr(self, "_connection_user_id", None) or user_id

        address = str(identity.get("address") or "")
        if not address:
            self._ota_reconnect_context = None
            if hasattr(self, "log_text"):
                self._log("[BLE_OTA_RECOVER] target context unavailable before OTA")
            return
        self._ota_reconnect_context = {
            "address": address,
            "name": str(identity.get("name") or ""),
            "user_id": user_id,
            "training_id": max(1, training_id),
        }
        if hasattr(self, "log_text"):
            self._log_diagnostic(
                "[BLE_OTA_RECOVER] target_saved "
                f"name={self._ota_reconnect_context['name']} address={address} "
                f"user_id={self._ota_reconnect_context['user_id']} "
                f"training_id={self._ota_reconnect_context['training_id']}"
            )

    def _can_send_ota_dfu_entry_command(self) -> bool:
        return bool(
            getattr(self.ota_manager, "connected", False)
            and self._command_found
            and not self._gatt_mismatch_active
        )

    def _start_dfu_upgrade_current_image(self, *, after_entry_command: bool) -> None:
        result = self.ota_manager.start_dfu_upgrade(after_entry_command=after_entry_command)
        self.ota_progress_var.set(0.0)
        self._refresh_ota_panel()

        if not result.ok:
            details = "\n".join(result.issues) if result.issues else "未知原因"
            messagebox.showwarning("OTA", f"{result.message}\n\n{details}")
            self._refresh_ota_panel()
            return

        image = self.ota_manager.image_info
        if image is None:
            messagebox.showwarning("OTA", "请先选择 OTA 固件文件。")
            self._refresh_ota_panel()
            return

        self._set_ota_controls_enabled(False)
        self.scan_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="disabled")
        self._set_command_buttons_enabled(True)
        if not self.dfu_ota_client.start(image.path):
            self._handle_ota_event({"type": "ota_error", "message": "OTA 正在进行中"})

    def _on_device_selected(self, _: Any) -> None:
        if self.disconnect_button.cget("state") == "normal":
            return
        self.connect_button.configure(
            state="normal" if (self._get_selected_index() is not None and not self._is_scanning) else "disabled"
        )

    def _get_selected_index(self) -> int | None:
        selection = self.device_tree.selection()
        if not selection:
            return None
        try:
            return int(selection[0])
        except ValueError:
            return None

    def _post_event(self, event: dict[str, Any]) -> None:
        self._mailbox.post("event", event)

    def _post_log(self, message: str) -> None:
        self._mailbox.post("log", (datetime.now().strftime("%H:%M:%S"), message))

    def _post_ota_event(self, event: dict[str, Any]) -> None:
        self._mailbox.post("ota_event", event)

    def _post_ota_log(self, message: str) -> None:
        self._mailbox.post("ota_log", message)

    def _pump_mailbox(self) -> None:
        if self._is_closing:
            if self._close_thread is not None and not self._close_thread.is_alive():
                if getattr(self, "_close_error", ""):
                    self._is_closing = False
                    messagebox.showwarning("文件收尾尚未完成", self._close_error)
                    self.root.after(20, self._pump_mailbox)
                    return
                self._mailbox.close()
                self.root.destroy()
                return
            self.root.after(20, self._pump_mailbox)
            return
        items, overflow, dropped = self._mailbox.take()
        try:
            if overflow:
                self._log(f"[UI_MAILBOX][ERR] critical_overflow={overflow}; disconnecting")
                self.ble_manager.disconnect()
                messagebox.showerror("接收界面异常", "界面事件队列已满，正在断开连接。请保存日志后重新连接。")
            if dropped:
                self._log(f"[UI_MAILBOX] log_display_dropped={dropped}")
            log_lines = []
            for kind, payload, posted_ns in items:
                if kind == "event":
                    work_started_ns = time.perf_counter_ns()
                    if payload.get("type") == "online_stream_status":
                        self.ble_manager.note_online_ui_dispatch((time.perf_counter_ns() - posted_ns) / 1e6)
                        self._online_list_revision += 1
                        self._online_list_snapshot = payload.get("session_metadata")
                    self._handle_event(payload)
                    if payload.get("type") == "online_stream_status":
                        self.ble_manager.note_online_ui_work((time.perf_counter_ns() - work_started_ns) / 1e6)
                elif kind == "ota_event":
                    self._handle_ota_event(payload)
                elif kind == "log":
                    stamp, message = payload
                    log_lines.append(f"[{stamp}] {message}\n")
                else:
                    self._on_ota_log(payload)
            if log_lines:
                self.log_text.configure(state="normal")
                self.log_text.insert(tk.END, "".join(log_lines))
                self.log_text.see(tk.END)
                self.log_text.configure(state="disabled")
            self._poll_online_session_list()
            self._poll_online_file_job()
        finally:
            self.root.after(20, self._pump_mailbox)

    def _handle_event(self, event: dict[str, Any]) -> None:
        et = event.get("type")
        if et == "stress_status":
            self.stress_view.handle(event)
            return
        if et == "online_storage_settled":
            if event.get("stress_report"):
                self.stress_view.report_ready(event)
            self._refresh_online_session_list()
            return

        if et == "timeout_action":
            if not self.ble_manager.timeout_action_is_current(event):
                return
            key = (event["connection_token"], event["generation"], event["token"])
            if key != self._timeout_notice_key:
                self._timeout_notice_key = key
                self.timeout_reason_var.set(
                    "连续30分钟没有保存到事件，即将自动停止采集" if event["reason"] == 1
                    else "待机已满30分钟，即将自动关机")
            self.ble_manager.confirm_timeout_action(event)
            return
        if et == "timeout_action_state":
            key = (event["connection_token"], event["generation"], event["token"])
            if key == self._timeout_notice_key:
                self.timeout_reason_var.set(event["message"])
            return
        if et in {"timeout_action_clear", "link_connected"}:
            self._timeout_notice_key = None
            if hasattr(self, "timeout_reason_var"):
                self.timeout_reason_var.set("")
            if et == "timeout_action_clear":
                return

        if et == "scan_result":
            self._is_scanning = False
            self.devices = list(event.get("devices", []) or [])
            self._render_devices(self.devices)
            count = len(self.devices)
            self.status_var.set(f"扫描完成：{count} 个 ZP 设备")
            self.connect_hint_var.set(
                f"已发现 {count} 个 ZP 设备。优先设备 {PREFERRED_DEVICE_NAME} 会排在第一位；"
                f"当前固件同一时间仅支持 {ZY100_MAX_SIMULTANEOUS_LINKS} 台连接，最多保留 "
                f"{ZY100_MAX_PAIRED_CENTRALS} 台已配对主机。"
                if count
                else "未发现 ZP- 开头设备，请唤醒设备后重新扫描"
            )
            if self.disconnect_button.cget("state") != "normal":
                self.scan_button.configure(state="normal")
                self.connect_button.configure(state="normal" if self._get_selected_index() is not None else "disabled")
            return

        if et == "connection_stage":
            message = str(event.get("message", ""))
            if message:
                self.status_var.set(message)
            return

        if et == "feature_config_status":
            if not self.ble_manager.feature_config_event_current(event):
                return
            status = str(event.get("status") or "unknown")
            message = str(event.get("message") or "")
            supported = bool(event.get("supported"))
            synced = bool(event.get("synced"))
            pending = bool(event.get("pending"))
            user_id = int(event.get("user_id") or 0)
            crc32 = int(event.get("crc32") or 0)
            generation = int(event.get("generation") or 0)
            self._feature_config_supported = supported
            self._feature_config_sync_active = bool(
                pending or status in {"saved_pending", "syncing"}
            )
            self._feature_config_start_ready = synced or status == "unsupported"
            if hasattr(self, "feature_sync_var"):
                if status == "unsupported":
                    self.feature_sync_var.set(
                        "设备同步：固件不支持配置同步，设备使用固件默认值"
                    )
                else:
                    suffix = "（待同步）" if pending else ""
                    self.feature_sync_var.set(
                        f"设备同步：{message or status}{suffix}；user_id={user_id}"
                    )
                self.feature_crc_var.set(f"CRC32：0x{crc32:08X}")
                self.feature_generation_var.set(
                    f"FTL generation：{generation if supported else '-'}"
                )
            if status == "profile_locked":
                self._feature_config_start_ready = False
            candidate = event.get("candidate")
            if candidate and hasattr(self, "feature_pending_var"):
                value = FeatureConfig.from_dict(candidate)
                self.feature_pending_var.set(
                    f"待同步参数：user_id={user_id}，自动采集={'开启' if value.auto_capture else '关闭'}，"
                    f"训练灯={'开启' if value.training_led else '关闭'}；CRC32=0x{value.crc32(user_id):08X}"
                )
            if status in {"matched", "synced", "profile_save_failed"} and hasattr(self, "feature_pending_var"):
                self.feature_pending_var.set("待同步参数：无（本次设备配置记录已确认）")
            if status == "apply_failed":
                self._feature_config_apply_failure = message
            elif status in {"profile_locked", "matched", "synced"}:
                self._feature_config_apply_failure = ""
            confirmed = event.get("confirmed")
            if not confirmed and hasattr(self, "feature_confirmed_var"):
                self.feature_confirmed_var.set("设备最后确认：当前连接暂无确认记录")
            if confirmed and hasattr(self, "feature_confirmed_var"):
                config = confirmed["config"]
                auto = "开启" if config["auto_capture"] else "关闭"
                led = "开启" if config["training_led"] else "关闭"
                runtime = ("运动自动采集开关受此设置控制" if event.get("auto_capture_runtime_supported")
                           else "当前固件未确认支持运动自动采集开关")
                self.feature_confirmed_var.set(
                    f"设备最后确认：user_id={confirmed['user_id']}，自动采集位={auto}，训练灯={led}；"
                    f"CRC32=0x{confirmed['crc32']:08X}；{runtime}"
                    + ("；硬件配置已完成" if event.get("hardware_apply_supported") and synced else "")
                )
            if status in {"matched", "synced"}:
                self.feature_profile_var.set(f"本地档案：user_id={user_id} 与设备确认结果一致")
            elif status == "profile_save_failed":
                self.feature_profile_var.set(f"本地档案：保存失败；{message}")
            self._refresh_command_button_state()
            self._refresh_calibration_controls()
            self._refresh_ota_panel()
            self._log(
                f"[FEATURE_CFG] status={status} supported={int(supported)} "
                f"synced={int(synced)} pending={int(pending)} user={user_id} "
                f"crc=0x{crc32:08X} generation={generation} message={message}"
            )
            return

        if et == "link_connected":
            self.status_var.set("BLE 物理链路已连接，正在完成安全与协议初始化")
            return

        if et == "control_ready":
            self.status_var.set("BLE 控制通道已就绪，正在同步离线数据")
            return

        if et == "business_ready":
            self._business_ready = True
            self.status_var.set("BLE 业务连接已就绪")
            self._refresh_command_button_state()
            return

        if et == "recovery_only":
            self._business_ready = False
            self.status_var.set("BLE 处于受限恢复模式；采集与数据操作保持禁用")
            self._refresh_command_button_state()
            return

        if et == "deferred_offline_capture":
            self._business_ready = False
            self.status_var.set("正在离线采集，用户同步将在结束后进行")
            self._refresh_command_button_state()
            return

        if et == "connection_user_sync":
            self._connection_user_synced = event.get("status") == "ok"
            if event.get("status") == "waiting_capture":
                self.status_var.set(str(event["message"]))
            self._refresh_command_button_state()
            return

        if et == "offline_capture_observer":
            observer_active = bool(event.get("active", True))
            observer_phase = str(event.get("phase", ""))
            if event.get("device_state") is not None:
                self._last_device_state = int(event["device_state"])
            self._business_ready = bool(
                not observer_active
                and observer_phase == "ready"
                and getattr(self, "_last_device_state", None)
                == DEVICE_STATE_WAIT_START
            )
            self.status_var.set(str(event.get("message") or "正在离线采集中"))
            self._refresh_command_button_state()
            self._refresh_calibration_controls()
            self._refresh_ota_panel()
            return

        if et == "connection_failed":
            message = str(event.get("message") or event.get("reason") or "BLE 连接初始化失败")
            self.status_var.set(message)
            self.connect_hint_var.set(message)
            self._refresh_command_button_state()
            return

        if et == "pairing_repair_confirmation":
            self._confirm_pairing_repair(event)
            return

        if et == "connection_recovery_stage":
            self.status_var.set(str(event.get("message") or "正在恢复连接"))
            if event.get("stage") == "cancelled":
                self._ota_reconnect_active = False
                self._ota_operation_active = False
            return

        if et == "ota_reconnect_stage":
            self._render_ota_reconnect_stage(event)
            return

        if et == "ota_reconnect_completed":
            self._render_ota_reconnect_completed(event)
            return

        if et == "ota_reconnect_failed":
            self._render_ota_reconnect_failed(event)
            return

        if et == "connected":
            self._is_scanning = False
            self._gatt_mismatch_active = False
            connected_address = str(event.get("address") or "")
            if connected_address:
                self._gatt_mismatch_blocked_addresses.discard(connected_address)
            self._rtc_sync_ok = False
            self.status_var.set(f"BLE 业务连接已就绪：{event.get('name', '')} {event.get('address', '')}")
            self.ota_manager.set_connected(True)
            self.ota_manager.update_gatt_services(self.ble_manager.get_last_services_snapshot())
            self._refresh_ota_panel()
            self._show_connected_view()
            return

        if et == "services":
            self._render_services(event)
            self.ota_manager.update_gatt_services(list(event.get("services", []) or []))
            self._refresh_ota_panel()
            return

        if et == "zy100_control_status":
            self._render_control_status(event)
            return

        if et == "device_info":
            self._render_device_info(event)
            return

        if et == "legacy_user_isolation_warning":
            message = str(event.get("message") or "旧固件管理员兼容模式")
            self.status_var.set("旧固件管理员兼容模式")
            self.command_result_var.set(message)
            self._log(f"[USER_CTX][LEGACY] {message}")
            messagebox.showwarning("用户隔离兼容提示", message)
            return

        if et == "foreign_offline_data_prompt":
            count = int(event.get("count") or 0)
            confirm = messagebox.askyesno(
                "清理其他用户离线数据",
                "检测到 Flash 中存在不属于您的数据，是否清理？\n\n"
                f"其他用户或旧版未知数据场次数：{count}\n\n"
                "确认：仅擦除不属于当前用户的session extent（每次4 KiB）。\n"
                "当前用户session、空白区和Online Spool不会访问。\n"
                "取消：保留数据并继续连接；已擦除空洞不会重新使用，可用容量会减少。",
            )
            self.ble_manager.respond_foreign_offline_cleanup(confirm)
            return

        if et == "foreign_offline_data_retained":
            message = str(event.get("message") or "其他用户数据已保留")
            self.status_var.set(message)
            self.command_result_var.set(message)
            if event.get("upgrade_required"):
                messagebox.showwarning("需要升级设备固件", message)
            return

        if et == "foreign_offline_data_clear_failed":
            message = str(event.get("message") or "其他用户数据清理失败")
            self.status_var.set(message)
            messagebox.showwarning("清理失败", message)
            return

        if et == "foreign_offline_data_purge_progress":
            message = str(event.get("message") or "其他用户离线数据定向清理")
            self.status_var.set(message)
            self.command_result_var.set(message)
            self._log(f"[OFFLINE_FOREIGN_PURGE] {message}")
            return

        if et == "mfg_info":
            self._render_mfg_info(event)
            return

        if et == "device_channel":
            self._render_device_channel(event)
            return

        if et == "ack_subscription":
            self._render_ack_subscription(event)
            return

        if et == "export_subscription":
            self._render_export_subscription(event)
            return

        if et == "calibration_capability":
            self._render_calibration_capability(event)
            return

        if et == "calibration_subscription":
            self._render_calibration_subscription(event)
            return

        if et == "calibration_info":
            self._render_calibration_info(event)
            return

        if et == "calibration_gate":
            self._render_calibration_gate(event)
            return

        if et == "calibration_command":
            self._render_calibration_command(event)
            return

        if et == "calibration_status":
            self._render_calibration_status(event)
            return

        if et == "calibration_transfer":
            self._render_calibration_transfer(event)
            return

        if et == "calibration_record":
            self._render_calibration_record(event)
            return

        if et == "calibration_diagnostics_command":
            self._render_calibration_diagnostics_command(event)
            return

        if et == "calibration_diagnostics":
            self._render_calibration_diagnostics(event)
            return

        if et == "calibration_diagnostics_error":
            self._render_calibration_diagnostics_error(event)
            return

        if et == "calibration_error":
            self._render_calibration_error(event)
            return

        if et == "battery_level":
            self._render_battery_level(event)
            return

        if et == "battery_subscription":
            self._render_battery_subscription(event)
            return

        if et == "gatt_mismatch":
            self._render_gatt_mismatch(event)
            return

        if et == "rtc_sync":
            self._render_rtc_sync(event)
            return

        if et == "command_write_result":
            self._render_command_write_result(event)
            return

        if et == "shipping_status":
            self._render_shipping_status(event)
            return

        if et == "ota_dfu_entry_status":
            self._render_ota_dfu_entry_status(event)
            return

        if et == "ota_dfu_entry_write_result":
            self._render_ota_dfu_entry_write_result(event)
            return

        if et == "ota_dfu_entry_ack":
            self._render_ota_dfu_entry_ack(event)
            return

        if et == "ota_dfu_entry_result":
            self._handle_ota_dfu_entry_result(event)
            return

        if et == "ota_warning":
            self._handle_ota_event(event)
            return

        if et == "zy100_ack":
            self._render_ack(event)
            return

        if et == "zy100_state_notify":
            self._render_ack(event)
            return

        if et == "ci_mode":
            mode = str(event.get("mode", "PERIPHERAL_EXACT"))
            reason = str(event.get("reason", ""))
            self.command_result_var.set(
                f"CI 协商模式：{mode}" + (f"（{reason}）" if reason else "")
            )
            self._log(
                f"[CI_MODE] mode={mode} session={event.get('session_id', 0)} "
                f"generation={event.get('generation', 0)} reason={reason}"
            )
            return

        if et == "zy100_link_state":
            ack = dict(event.get("ack") or {})
            detail = int(ack.get("detail") or 0)
            training = int(ack.get("training_id") or 0)
            self.command_result_var.set(
                "Central CI："
                f"state={ack.get('device_state')} profile={ack.get('exec_mode')} "
                f"phase={ack.get('status')} CI={detail & 0xFFFF} "
                f"latency={(detail >> 16) & 0xFFFF}"
            )
            self._log(
                "[CI_LINK] "
                f"generation={training & 0xFFFF} transition={training >> 16} "
                f"business_state={ack.get('device_state')} "
                f"profile={ack.get('exec_mode')} phase={ack.get('status')} "
                f"actual_ci={detail & 0xFFFF} latency={(detail >> 16) & 0xFFFF}"
            )
            return

        if et == "zy100_ack_error":
            self._render_ack_error(event)
            return

        if et == "transfer_status":
            self._render_transfer_status(event)
            return

        if et == "offline_v2_status":
            self._render_offline_v2_status(event)
            return

        if et == "offline_priority_gate":
            self._offline_priority_active = bool(event.get("active"))
            self._offline_priority_phase = str(
                event.get("phase") or ("syncing" if self._offline_priority_active else "ready")
            )
            message = str(event.get("message") or "")
            if message:
                self.status_var.set(message)
                self.command_result_var.set(message)
            self._refresh_command_button_state()
            self._refresh_calibration_controls()
            self._refresh_ota_panel()
            return

        if et == "online_stream_status":
            self._render_online_stream_status(event)
            self._maybe_refresh_online_session_list(str(event.get("status") or ""))
            return

        if et == "sessions_changed":
            self._refresh_session_list()
            return

        if et == "offline_v2_sessions_changed":
            self._refresh_session_list()
            return

        if et == "disconnected":
            calibration_was_active = self._calibration_active
            preserve_calibration_result = bool(
                self._calibration_terminal_result is not None
                or self._calibration_device_success_pending
                or calibration_was_active
            )
            self._command_found = False
            self._ack_subscribed = False
            self._export_subscribed = False
            self._rtc_sync_ok = False
            self._connection_user_id = None
            self._last_device_state = None
            self._start_command_pending = False
            self._offline_start_command_pending = False
            self._calibration_available = False
            self._calibration_gate_ready = False
            self._calibration_gate_state = "unknown"
            self._calibration_active = False
            self._gatt_mismatch_active = False
            self._export_blocking_commands = False
            self._offline_priority_active = False
            self._offline_priority_phase = "idle"
            self._business_ready = False
            self._feature_config_start_ready = False
            self._feature_config_supported = False
            self._feature_config_sync_active = False
            self._offline_control_supported = False
            self._offline_stop_any_supported = False
            self._connection_user_synced = False
            self.status_var.set("连接已断开")
            self.control_status_var.set("ZY100 Control Service：未发现")
            self.ack_subscription_var.set("ACK Notify：未订阅")
            self.rtc_sync_var.set("RTC Sync: not synced")
            if hasattr(self, "feature_sync_var"):
                self.feature_sync_var.set(
                    getattr(self, "_feature_config_apply_failure", "") or
                    "设备同步：连接已断开；设备保留上次成功配置")
            self.command_result_var.set("指令状态：等待发送")
            self._reset_device_info_summary()
            self._reset_battery_summary()
            self.last_sent_var.set("最近发送：-")
            self._set_text_widget(self.ack_detail_text, "等待 ACK Notify。")
            self._set_gatt_placeholder()
            self.transfer_status_var.set("传输状态：未连接")
            self.transfer_progress_var.set("当前传输：-")
            self._reset_online_stream_status()
            self._reset_calibration_status(
                "连接已断开；若设备已接受校准，设备端仍会继续执行",
                preserve_result=preserve_calibration_result,
            )
            if self._calibration_device_success_pending:
                self._set_calibration_banner(
                    "设备端校准已成功，连接已断开；重连后读取并确认记录",
                    "pending",
                )
            elif calibration_was_active:
                self._calibration_terminal_result = "pending"
                self._set_calibration_banner(
                    "BLE 已断开，设备端仍在继续校准；请重连查看最终结果",
                    "pending",
                )
            self.ota_manager.set_connected(False)
            self._refresh_ota_panel()
            self._refresh_command_button_state()
            self._show_connect_view("连接已断开，请重新扫描/连接 ZP 设备")
            return

        if et == "error":
            self._is_scanning = False
            message = str(event.get("message", "未知错误"))
            self.status_var.set("错误")
            self._log(f"ERROR: {message}")
            if self.disconnect_button.cget("state") != "normal":
                self.scan_button.configure(state="normal")
                self.connect_button.configure(state="normal" if self._get_selected_index() is not None else "disabled")
            return

    def _handle_ota_event(self, event: dict[str, Any]) -> None:
        event_type = str(event.get("type", ""))
        self.ota_manager.handle_dfu_event(event)

        if event_type == "ota_status":
            message = str(event.get("message", ""))
            if message:
                self.status_var.set(f"DFU OTA：{message}")
            self._refresh_ota_panel()
            return

        if event_type == "ota_dfu_services":
            lines = list(event.get("lines", []) or [])
            if lines:
                self._set_text_widget(self.ota_service_text, "\n".join(str(line) for line in lines))
            self._refresh_ota_panel()
            return

        if event_type == "ota_progress":
            self.ota_progress_var.set(float(event.get("percent") or 0.0))
            self._refresh_ota_panel()
            return

        if event_type == "ota_warning":
            message = str(event.get("message", ""))
            if message:
                self._on_ota_log(f"WARNING: {message}")
            self._refresh_ota_panel()
            return

        if event_type == "ota_completed":
            self.ota_progress_var.set(100.0)
            self.status_var.set("DFU OTA：固件写入完成，等待设备重启")
            self._refresh_ota_panel()
            self._pending_ota_completion_message = str(event.get("message", "OTA 已完成"))
            context = self._ota_reconnect_context
            reconnect = getattr(self.ble_manager, "reconnect_after_ota", None)
            if context and callable(reconnect):
                self._ota_reconnect_active = True
                self.connect_hint_var.set("OTA 固件写入完成，正在等待设备重启并恢复普通 BLE 连接。")
                self.scan_button.configure(state="disabled")
                self.connect_button.configure(state="disabled")
                self.disconnect_button.configure(state="disabled")
                reconnect(
                    context["address"],
                    context["name"],
                    user_id=context["user_id"],
                    training_id=context["training_id"],
                )
            else:
                self._ota_reconnect_active = False
                self._show_connect_view("OTA 已完成，但未保存普通 BLE 地址；请扫描并重新连接设备。")
                messagebox.showwarning(
                    "OTA",
                    f"{self._pending_ota_completion_message}\n\n未保存升级前的设备地址，请在上位机内重新扫描并连接。",
                )
            return

        if event_type == "ota_error":
            self._ota_operation_active = False
            message = str(event.get("message", "OTA 失败"))
            image_written = bool(event.get("validated"))
            self.status_var.set("镜像已写入，连接恢复失败" if image_written else "DFU OTA：失败")
            self._refresh_ota_panel()
            self._show_connect_view(message if image_written else "DFU OTA 失败，请确认设备仍在 DFU 模式后重试。")
            messagebox.showerror("OTA", message)
            return

    def _render_ota_reconnect_stage(self, event: dict[str, Any]) -> None:
        self._ota_reconnect_active = True
        stage = str(event.get("stage") or "")
        message = str(event.get("message") or "正在恢复普通 BLE 连接")
        self.status_var.set(f"OTA 后重连：{message}")
        self.connect_hint_var.set(message)
        self.scan_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="disabled")
        for button in getattr(self, "_command_buttons", []):
            button.configure(state="disabled")
        self._log(f"[BLE_OTA_RECOVER][UI] stage={stage} message={message}")

    def _render_ota_reconnect_completed(self, event: dict[str, Any]) -> None:
        self._ota_reconnect_active = False
        self._ota_operation_active = False
        self._gatt_mismatch_active = False
        self._gatt_mismatch_blocked_addresses.discard(str(event.get("address") or ""))
        self.status_var.set("Ready")
        self.connect_hint_var.set(
            f"OTA 已完成并重新连接：{event.get('name', '')} {event.get('address', '')}"
        )
        self.ota_manager.set_connected(True)
        self._refresh_ota_panel()
        self._show_connected_view()
        self._refresh_command_button_state()
        completion_message = self._pending_ota_completion_message or "OTA 已完成"
        self._pending_ota_completion_message = ""
        self._ota_reconnect_context = None
        messagebox.showinfo("OTA", f"{completion_message}\n\n普通 BLE 已恢复连接，可以继续操作。")

    def _confirm_pairing_repair(self, event: dict[str, Any]) -> None:
        operation_id = int(event["operation_id"])
        if not self.ble_manager.pairing_repair_is_current(operation_id):
            return
        approved = messagebox.askyesno(
            "确认修复目标设备配对",
            f"保留配对的自动恢复未成功。\n设备：{event.get('name') or '未知'}\n"
            f"地址：{event.get('address')}\nSN：{event.get('sn') or '未知'}\n"
            f"原因：{event.get('reason')}\n\n"
            "是否删除此设备的 Windows 配对并重新连接？\n"
            "仅影响这个设备，需要重新配对；不擦除设备数据。\n"
            "选择“否”将保留配对，不进行删除。",
            default=messagebox.NO,
        )
        self.ble_manager.respond_pairing_repair(operation_id, approved)

    def _render_ota_reconnect_failed(self, event: dict[str, Any]) -> None:
        self._ota_reconnect_active = False
        self._ota_operation_active = False
        self._command_found = False
        self._ack_subscribed = False
        self._export_subscribed = False
        self._rtc_sync_ok = False
        message = str(event.get("message") or "设备未能恢复普通 BLE 连接")
        self.status_var.set("OTA 后自动重连失败")
        self.ota_manager.set_connected(False)
        self._refresh_ota_panel()
        self._show_connect_view("自动恢复失败，可直接在上位机中重新扫描并连接，无需进入 Windows 设置。")
        self.scan_button.configure(state="normal")
        self.connect_button.configure(
            state="normal" if self._get_selected_index() is not None else "disabled"
        )
        completion_message = self._pending_ota_completion_message or "OTA 固件写入已完成"
        self._pending_ota_completion_message = ""
        messagebox.showwarning(
            "OTA 后重连",
            f"{completion_message}\n\n自动恢复连接失败：{message}\n请在上位机中重新扫描并连接。",
        )

    def _render_devices(self, devices: list[dict[str, Any]]) -> None:
        self.device_tree.delete(*self.device_tree.get_children())
        for idx, device in enumerate(devices):
            values = (
                device.get("name", ""),
                device.get("address", ""),
                device.get("rssi", ""),
            )
            self.device_tree.insert("", tk.END, iid=str(idx), values=values)

        if devices:
            self.device_tree.selection_set("0")
            self.device_tree.focus("0")
            self.device_tree.see("0")

    def _render_services(self, event: dict[str, Any]) -> None:
        services = list(event.get("services", []) or [])
        lines = list(event.get("lines", []) or [])
        if not services:
            self._set_gatt_placeholder()
            return

        svc_count = len(services)
        chr_count = sum(len(service.get("characteristics", [])) for service in services)
        svc_set = {self._normalize_uuid(str(service.get("uuid", ""))) for service in services}
        chr_set: set[str] = set()
        battery_props: list[str] = []
        summary_lines: list[str] = []
        for idx, service in enumerate(services, 1):
            chars = list(service.get("characteristics", []) or [])
            summary_lines.append(f"{idx}. {service.get('uuid', '')}  特征:{len(chars)}")
            for char in chars:
                cu = str(char.get("uuid", ""))
                chr_set.add(self._normalize_uuid(cu))
                if self._normalize_uuid(cu) == BATTERY_LEVEL_UUID:
                    battery_props = list(char.get("properties", []) or [])
                summary_lines.append(f"    - {cu} | {','.join(char.get('properties', []))}")

        self._gatt_vars["service_count"].set(f"服务数量：{svc_count}")
        self._gatt_vars["char_count"].set(f"特征数量：{chr_count}")
        self._gatt_vars["control"].set(
            f"Control Service：{'已发现' if ZY100_CONTROL_SERVICE_UUID in svc_set else '未发现'}"
        )
        self._gatt_vars["command"].set(f"Command：{'已发现' if ZY100_COMMAND_UUID in chr_set else '未发现'}")
        self._gatt_vars["ack"].set(f"ACK / Status：{'已发现' if ZY100_ACK_UUID in chr_set else '未发现'}")
        self._gatt_vars["device_info"].set(
            f"Device Info：{'已发现' if ZY100_DEVICE_INFO_UUID in chr_set else '未发现'}"
        )
        battery_found = BATTERY_SERVICE_UUID in svc_set and BATTERY_LEVEL_UUID in chr_set
        self._gatt_vars["battery"].set(
            f"Battery Level：{'已发现' if battery_found else '未发现'} {battery_props if battery_found else ''}"
        )
        self._gatt_vars["export"].set(f"Export Data：{'已发现' if ZY100_EXPORT_DATA_UUID in chr_set else '未发现'}")
        if "calibration" in self._gatt_vars:
            self._gatt_vars["calibration"].set(
                f"Calibration Service：{'已发现' if CALIBRATION_SERVICE_UUID in svc_set else '未发现'}"
            )
        self._set_text_widget(self.gatt_list_text, "\n".join(summary_lines))

        export = [
            "GATT 服务与特征值导出",
            "====================",
            self._gatt_vars["service_count"].get(),
            self._gatt_vars["char_count"].get(),
            self._gatt_vars["control"].get(),
            self._gatt_vars["command"].get(),
            self._gatt_vars["ack"].get(),
            self._gatt_vars["device_info"].get(),
            self._gatt_vars["battery"].get(),
            self._gatt_vars["export"].get(),
            self._gatt_vars["calibration"].get() if "calibration" in self._gatt_vars else "",
            self._gatt_vars["notify"].get(),
            "",
            "完整明细：",
            "---------",
            *lines,
        ]
        self._gatt_export_text = "\n".join(export)

    def _render_control_status(self, event: dict[str, Any]) -> None:
        service_found = bool(event.get("service_found"))
        command_found = bool(event.get("command_found"))
        ack_found = bool(event.get("ack_found"))
        info_found = bool(event.get("device_info_found"))
        export_found = bool(event.get("export_found"))
        self._command_found = command_found
        self._gatt_mismatch_active = not (service_found and command_found and ack_found and export_found)

        self.control_status_var.set(
            "ZY100 Control Service："
            f"{'已发现' if service_found else '未发现'}，"
            f"Command {'已发现' if command_found else '未发现'}，"
            f"ACK {'已发现' if ack_found else '未发现'}"
        )
        self._gatt_vars["control"].set(f"Control Service：{'已发现' if service_found else '未发现'}")
        self._gatt_vars["command"].set(
            f"Command：{'已发现' if command_found else '未发现'} {event.get('command_properties', [])}"
        )
        self._gatt_vars["ack"].set(f"ACK / Status：{'已发现' if ack_found else '未发现'} {event.get('ack_properties', [])}")
        self._gatt_vars["device_info"].set(f"Device Info：{'已发现' if info_found else '未发现'}")
        self._gatt_vars["export"].set(
            f"Export Data：{'已发现' if export_found else '未发现'} {event.get('export_properties', [])}"
        )
        self._refresh_command_button_state()

    @staticmethod
    def _parse_device_info_fields(value: str) -> dict[str, str]:
        fields: dict[str, str] = {}
        for part in value.split(";"):
            if "=" not in part:
                continue
            key, raw_value = part.split("=", 1)
            key = key.strip().lower()
            if key:
                fields[key] = raw_value.strip()
        return fields

    def _reset_device_info_summary(self) -> None:
        self._device_serial = ""
        self._device_serial_source = ""
        self._offline_control_supported = False
        self._offline_stop_any_supported = False
        self._connection_user_synced = False
        self._offline_start_supported = False
        self._device_channel_short_received = False
        self._device_channel_short_valid = False
        if hasattr(self, "device_version_var"):
            self.device_version_var.set(DEVICE_INFO_VERSION_PLACEHOLDER)
        if hasattr(self, "device_channel_var"):
            self.device_channel_var.set(DEVICE_CHANNEL_PLACEHOLDER)

    def _render_device_info(self, event: dict[str, Any]) -> None:
        value = str(event.get("value", "")).strip()
        fields = self._parse_device_info_fields(value)
        code = fields.get("code") or "-"
        fw = fields.get("fw") or "-"
        self._offline_control_supported = fields.get("offctl") == "1"
        self._offline_stop_any_supported = fields.get("offstop_any") == "1"
        try:
            version_code = int(code, 10)
        except ValueError:
            version_code = None
        self._offline_start_supported = bool(
            self._offline_control_supported
            and version_code is not None
            and version_code >= LEGACY_OFFLINE_INTENT_FIX_CODE
        )
        serial = (
            fields.get("sn")
            or fields.get("serial")
            or fields.get("serial_number")
            or ""
        ).strip()
        if serial:
            self._device_serial = serial
            self._device_serial_source = "device_info"
            self._log_diagnostic(
                "[SN_CACHE] source=device_info action=updated "
                f"value={serial}"
            )
        else:
            self._log_diagnostic("[SN_CACHE] source=device_info action=empty action=kept_existing")

        if hasattr(self, "device_version_var"):
            self.device_version_var.set(f"Device Info: code={code} fw={fw}")
        if hasattr(self, "_gatt_vars") and "device_info" in self._gatt_vars:
            if value:
                self._gatt_vars["device_info"].set(f"Device Info：已读取 code={code} fw={fw} raw={value}")
            else:
                self._gatt_vars["device_info"].set(f"Device Info：已读取 code={code} fw={fw}")
        if not self._offline_control_supported and hasattr(self, "command_result_var"):
            self.command_result_var.set("离线 BLE 控制：需升级固件（缺少 offctl=1）")
        elif not self._offline_start_supported and hasattr(self, "command_result_var"):
            self.command_result_var.set(
                f"旧固件离线开始已保护：请使用实体双击，或升级至 {LEGACY_OFFLINE_INTENT_FIX_CODE} 及以上"
            )
        if hasattr(self, "_command_buttons_by_cmd"):
            self._refresh_command_button_state()

    def _render_mfg_info(self, event: dict[str, Any]) -> None:
        value = str(event.get("value", "")).strip()
        fields = self._parse_device_info_fields(value)
        channel = fields.get("channel") or "unknown"
        serial = (
            fields.get("sn")
            or fields.get("serial")
            or fields.get("serial_number")
            or ""
        ).strip()
        previous_serial = getattr(self, "_device_serial", "").strip()
        if serial:
            self._device_serial = serial
            self._device_serial_source = "mfg_info"
            action = "updated"
        else:
            action = "kept_existing" if previous_serial else "empty"
        effective_serial = getattr(self, "_device_serial", "").strip()
        self._log_diagnostic(
            "[SN_CACHE] source=mfg_info "
            f"parsed={serial or '<empty>'} action={action} "
            f"effective={effective_serial or '<empty>'}"
        )
        if hasattr(self, "device_channel_var"):
            if channel in {"CG", "ST", "DEFAULT"}:
                if not (
                    getattr(self, "_device_channel_short_received", False)
                    and getattr(self, "_device_channel_short_valid", False)
                ):
                    self.device_channel_var.set(f"Channel: {channel}")
            elif not getattr(self, "_device_channel_short_received", False):
                self.device_channel_var.set("Channel: unknown")

    def _render_device_channel(self, event: dict[str, Any]) -> None:
        value = str(event.get("value") or "DEFAULT").strip()
        valid = bool(event.get("valid", value in {"CG", "ST", "DEFAULT"}))
        if value not in {"CG", "ST", "DEFAULT"}:
            value = "DEFAULT"
            valid = False
        self._device_channel_short_received = True
        self._device_channel_short_valid = valid
        if hasattr(self, "device_channel_var"):
            self.device_channel_var.set(f"Channel: {value}")

    def _reset_battery_summary(self) -> None:
        if hasattr(self, "battery_level_var"):
            self.battery_level_var.set("设备电量：-")
        if hasattr(self, "_gatt_vars") and "battery" in self._gatt_vars:
            self._gatt_vars["battery"].set("Battery Level：未发现/未订阅")

    def _render_battery_level(self, event: dict[str, Any]) -> None:
        try:
            percent = int(event.get("percent"))
        except (TypeError, ValueError):
            return
        source = str(event.get("source", ""))
        uuid = str(event.get("uuid", BATTERY_LEVEL_UUID))
        self.battery_level_var.set(f"设备电量：{percent}%")
        if hasattr(self, "_gatt_vars") and "battery" in self._gatt_vars:
            status = "通知" if source == "notify" else "读取"
            current = self._gatt_vars["battery"].get()
            prefix = "Battery Level：已订阅" if "已订阅" in current else f"Battery Level：已{status}"
            self._gatt_vars["battery"].set(f"{prefix} {uuid}，当前 {percent}%")

    def _render_battery_subscription(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        uuid = str(event.get("uuid", BATTERY_LEVEL_UUID))
        if status == "subscribed":
            text = f"Battery Level：已订阅 {uuid}"
        elif status == "missing":
            text = "Battery Level：特征未发现/未订阅"
        elif status == "failed":
            text = f"Battery Level：订阅失败 {event.get('error', '')}"
        else:
            text = "Battery Level：未订阅"
        if hasattr(self, "_gatt_vars") and "battery" in self._gatt_vars:
            self._gatt_vars["battery"].set(text)

    def _render_ack_subscription(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        uuid = str(event.get("uuid", ""))
        self._ack_subscribed = status == "subscribed"

        if status == "subscribed":
            text = f"ACK Notify：已订阅 {uuid}"
        elif status == "missing":
            text = "ACK Notify：特征未发现"
        elif status == "failed":
            text = f"ACK Notify：订阅失败 {event.get('error', '')}"
        else:
            text = "ACK Notify：未订阅"

        self.ack_subscription_var.set(text)
        self._gatt_vars["notify"].set(text)
        self._refresh_command_button_state()

    def _render_export_subscription(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        uuid = str(event.get("uuid", ""))
        self._export_subscribed = status == "subscribed"
        if status == "subscribed":
            text = f"Export Data：已订阅 {uuid}"
        elif status == "missing":
            text = "Export Data：特征未发现"
        elif status == "failed":
            text = f"Export Data：订阅失败 {event.get('error', '')}"
        else:
            text = "Export Data：未订阅"
        self._gatt_vars["export"].set(text)
        self.transfer_status_var.set(f"传输状态：{text}")
        self._refresh_command_button_state()

    def _update_calibration_motion_hint(self) -> None:
        if self._is_closing:
            return
        changed_at = self._calibration_last_progress_changed_monotonic
        if (
            self._calibration_active
            and 0 <= self._calibration_last_progress < 100
            and changed_at is not None
            and (time.monotonic() - changed_at) >= 3.0
        ):
            self._set_calibration_banner(
                f"动作进度：{self._calibration_last_progress}%｜当前动作维度重复，请改变旋转轴",
                "collecting",
            )
        self.root.after(1000, self._update_calibration_motion_hint)

    def _set_calibration_banner(self, text: str, kind: str) -> None:
        colors = {
            "idle": ("#E5E7EB", "#1F2937"),
            "collecting": ("#DBEAFE", "#1E3A8A"),
            "working": ("#FEF3C7", "#92400E"),
            "success": ("#DCFCE7", "#166534"),
            "limited": ("#FEF3C7", "#92400E"),
            "no_update": ("#E5E7EB", "#374151"),
            "failure": ("#FEE2E2", "#991B1B"),
            "pending": ("#FFEDD5", "#9A3412"),
        }
        background, foreground = colors.get(kind, colors["idle"])
        if hasattr(self, "calibration_result_var"):
            self.calibration_result_var.set(text)
        if hasattr(self, "calibration_result_banner"):
            self.calibration_result_banner.configure(
                background=background,
                foreground=foreground,
            )

    def _show_calibration_popup_once(
        self,
        transaction_id: int,
        result: str,
        title: str,
        message: str,
    ) -> None:
        key = (transaction_id & 0xFF, result)
        if key in self._calibration_popup_keys:
            return
        self._calibration_popup_keys.add(key)
        if result in {"success", "limited", "no_update"}:
            messagebox.showinfo(title, message)
        else:
            messagebox.showerror(title, message)

    def _render_calibration_capability(self, event: dict[str, Any]) -> None:
        self._calibration_available = bool(event.get("available"))
        text = "已发现" if self._calibration_available else "未发现（旧固件仍可使用采集功能）"
        self.calibration_capability_var.set(f"地磁校准服务：{text}")
        if hasattr(self, "_gatt_vars") and "calibration" in self._gatt_vars:
            self._gatt_vars["calibration"].set(f"Calibration Service：{text}")
        self._refresh_calibration_controls()

    def _render_calibration_subscription(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        if status == "subscribed":
            self._calibration_available = True
            self.calibration_capability_var.set("地磁校准服务：已订阅 TX / Status")
        elif status == "waiting_encryption":
            self._calibration_available = False
            self.calibration_capability_var.set(
                f"地磁校准服务：等待配对加密（第 {int(event.get('attempt', 1))} 次）"
            )
        elif status == "unavailable":
            self._calibration_available = False
            self.calibration_capability_var.set("地磁校准服务：当前固件不支持")
        else:
            self._calibration_available = False
            self.calibration_capability_var.set(f"地磁校准服务：订阅失败 {event.get('error', '')}")
        self._refresh_calibration_controls()

    def _render_calibration_info(self, event: dict[str, Any]) -> None:
        if bool(event.get("valid")):
            self.calibration_info_var.set(
                "记录："
                f"generation={int(event.get('generation', 0))} "
                f"bytes={int(event.get('record_bytes', 0))} "
                f"model={event.get('mag_model_kind', 'FULL')} "
                f"flags=0x{int(event.get('valid_flags', 0)):08X} "
                f"crc=0x{int(event.get('crc32', 0)):08X}"
            )
        else:
            self.calibration_info_var.set("记录：设备当前没有有效校准数据")

    def _render_calibration_gate(self, event: dict[str, Any]) -> None:
        state = str(event.get("state", "unknown"))
        self._calibration_gate_state = state
        self._calibration_gate_ready = bool(event.get("ready"))
        summary = event.get("summary") or {}
        if state == "uncalibrated":
            self.calibration_info_var.set(
                "校准状态：未做校准；当前按默认参数使用，可在线采集或进行地磁校准"
            )
        elif state == "valid":
            self.calibration_info_var.set(
                "校准状态：摘要与完整记录已确认；"
                f"generation={int(summary.get('generation', 0))} "
                f"model={summary.get('mag_model_kind', 'NONE')} "
                f"flags=0x{int(summary.get('valid_flags', 0)):08X} "
                f"crc=0x{int(summary.get('crc32', 0)):08X}"
            )
        elif state == "record_syncing":
            self.calibration_info_var.set(
                "校准状态：摘要已确认，正在下载并校验完整记录；"
                f"generation={int(summary.get('generation', 0))} "
                f"crc=0x{int(summary.get('crc32', 0)):08X}"
            )
        else:
            message = str(event.get("message", "")).strip()
            self.calibration_info_var.set(
                f"校准状态未确认；在线采集和地磁校准已锁定。{message}"
            )
        if state == "valid" and self._calibration_terminal_result in {
            "device_success",
            "device_limited",
        }:
            transaction_id = self._calibration_operation_transaction or 0
            self._calibration_device_success_pending = False
            self._calibration_terminal_result = "success"
            text = "基础地磁校准正常完成，新摘要与完整记录已确认"
            self._set_calibration_banner(text, "success")
            self._show_calibration_popup_once(
                transaction_id,
                "success",
                "基础地磁校准正常完成",
                text,
            )
        self._refresh_command_button_state()
        self._refresh_calibration_controls()

    def _render_calibration_command(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        message = str(event.get("message", ""))
        if status in {"failed", "unavailable", "busy"}:
            self._calibration_active = False
        self.calibration_status_var.set(f"校准命令：{status} {message}".strip())
        self._refresh_calibration_controls()
        self._refresh_command_button_state()
        self._refresh_ota_panel()

    def _render_calibration_status(self, event: dict[str, Any]) -> None:
        code = int(event.get("code", 0))
        name = str(event.get("name", "UNKNOWN"))
        detail = int(event.get("detail", 0))
        transaction_id = int(event.get("transaction_id", 0))
        status_zh = calibration_status_zh(code)
        active_codes = {14, 15, 16, 17, 18, 24}
        failure_codes = {8, 9, 10, 11, 12, 13, 20, 21, 22, 23, 25}
        self._calibration_active = code in active_codes
        if code in {14, 15} and transaction_id:
            if self._calibration_operation_transaction != transaction_id:
                self._calibration_started_monotonic = time.monotonic()
                self._calibration_last_progress = 0
                self._calibration_last_progress_changed_monotonic = time.monotonic()
            self._calibration_operation_transaction = transaction_id
            self._calibration_terminal_result = None
            self._calibration_device_success_pending = False
        if code == 16:
            progress = max(0, min(99, detail))
            if progress != self._calibration_last_progress:
                self._calibration_last_progress_changed_monotonic = time.monotonic()
            self._calibration_last_progress = progress
            self.calibration_progress_var.set(float(progress))
            self._set_calibration_banner(
                f"动作进度：{progress}%｜请同时进行滚转、俯仰和偏航；达到100%即可停止",
                "collecting",
            )
        elif code in {17, 18, 19, 26, 27}:
            self._calibration_last_progress = 100
            self._calibration_last_progress_changed_monotonic = None
            self.calibration_progress_var.set(100.0)
            if code == 17:
                self._set_calibration_banner("基础条件已满足，正在计算硬铁偏置", "working")
            elif code == 18:
                self._set_calibration_banner("拟合成功，正在安全保存到设备 Flash", "working")
        elif code in failure_codes:
            self.calibration_progress_var.set(float(self._calibration_last_progress))
        if not (code in {2, 3, 4} and self._calibration_terminal_result is not None):
            self.calibration_status_var.set(
                f"校准状态：{status_zh}（{name}） detail={detail} transaction={transaction_id}"
            )
        generation = int(event.get("generation", 0))
        crc32 = int(event.get("crc32", 0))
        if code == 19:
            self._calibration_operation_transaction = transaction_id
            self._calibration_device_success_pending = True
            self._calibration_terminal_result = "device_success"
            self._set_calibration_banner(
                "设备端校准和 Flash 保存成功，正在接收并校验校准记录",
                "success",
            )
            self.calibration_info_var.set(
                f"记录：设备保存成功 generation={generation} crc=0x{crc32:08X}，等待接收并校验"
            )
        elif code == 26:
            reason = calibration_completion_detail_zh(detail)
            self._calibration_operation_transaction = transaction_id
            self._calibration_device_success_pending = True
            self._calibration_terminal_result = "device_limited"
            self._set_calibration_banner(
                "基础校准正常完成，已保存新的硬铁参数；正在接收并校验记录",
                "success",
            )
            self.calibration_info_var.set(
                f"记录：基础硬铁参数已保存 generation={generation} crc=0x{crc32:08X}"
            )
        elif code == 27:
            reason = calibration_completion_detail_zh(detail)
            self._calibration_operation_transaction = transaction_id
            self._calibration_device_success_pending = False
            self._calibration_terminal_result = "success"
            self._set_calibration_banner(
                f"基础校准正常完成，已有参数更好，继续保留；{reason}", "success"
            )
            if not self._calibration_device_success_pending:
                self._show_calibration_popup_once(
                    transaction_id,
                    "success",
                    "基础地磁校准正常完成",
                    f"设备已有参数质量更好，本次不覆盖。\n\n{reason}",
                )
        elif code in failure_codes:
            reason = calibration_failure_detail_zh(detail) if detail else status_zh
            self._calibration_terminal_result = "failure"
            self._calibration_device_success_pending = False
            self._set_calibration_banner(f"校准失败：{status_zh}；{reason}", "failure")
            self._show_calibration_popup_once(
                transaction_id,
                "failure",
                "地磁校准失败",
                f"设备地磁校准失败。\n\n状态：{status_zh}\n原因：{reason}\n事务ID：{transaction_id}",
            )
        elif code in {2, 3, 4} and self._calibration_terminal_result is not None:
            # 记录传输状态单独显示，不覆盖已经锁存的校准业务结果。
            pass
        elif code in {14, 15}:
            self._set_calibration_banner(status_zh, "working")
        self._refresh_calibration_controls()
        self._refresh_command_button_state()
        self._refresh_ota_panel()

    def _render_calibration_transfer(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        received = int(event.get("received_bytes", 0))
        total = int(event.get("total_bytes", 0))
        if event.get("transfer_kind") == "diagnostics":
            self.calibration_diagnostics_status_var.set(
                f"诊断摘要：{status} {received}/{total} bytes"
            )
        else:
            self.calibration_transfer_var.set(
                f"传输：{status} {received}/{total} bytes"
            )

    def _render_calibration_record(self, event: dict[str, Any]) -> None:
        terminal_before = self._calibration_terminal_result
        self._last_calibration_record = dict(event)
        bias = event.get("mag_bias_raw_counts")
        matrix = event.get("mag_soft_iron_matrix_row_major")
        lines = [
            "校准记录已完成 CRC/TLV 校验并保存",
            f"generation = {event.get('generation', 0)}",
            f"model = {event.get('mag_model_kind', 'FULL')}",
            f"valid_flags = 0x{int(event.get('valid_flags', 0)):08X}",
            f"record_bytes = {event.get('record_bytes', 0)}",
            f"crc32 = {event.get('crc32', '')}",
            f"ACK = {event.get('ack_status', '')}",
            f"saved_path = {event.get('saved_path', '')}",
            "",
            f"hard_iron_bias_raw_counts = {bias}",
            "soft_iron_matrix_row_major =",
        ]
        if isinstance(matrix, list) and len(matrix) == 9:
            for row in range(3):
                lines.append("  " + "  ".join(f"{float(matrix[row * 3 + col]): .8f}" for col in range(3)))
        else:
            lines.append(f"  {matrix}")
        self._set_text_widget(self.calibration_record_text, "\n".join(lines))
        self.calibration_transfer_var.set(f"传输：记录已保存，ACK {event.get('ack_status', '')}")
        self.calibration_info_var.set(
            f"记录：generation={event.get('generation', 0)} "
            f"crc={event.get('crc32', '')}"
        )
        transaction_id = int(event.get("transaction_id", 0))
        ack_status = str(event.get("ack_status", ""))
        if ack_status == "sent":
            self._calibration_active = False
            self._calibration_device_success_pending = False
            if terminal_before == "device_limited":
                result_kind = "success"
                banner_kind = "success"
                title = "基础地磁校准正常完成"
                summary = "新的硬铁参数已保存，上位机已校验并发送 ACK"
            elif terminal_before == "device_no_update_pending":
                result_kind = "success"
                banner_kind = "success"
                title = "基础地磁校准正常完成"
                summary = "已有参数质量更好，本次不覆盖并继续使用已有记录"
            else:
                result_kind = "success"
                banner_kind = "success"
                title = "地磁校准成功"
                summary = "基础硬铁模型已保存，上位机已校验并发送 ACK"
            self._calibration_terminal_result = result_kind
            self.calibration_progress_var.set(100.0)
            self._set_calibration_banner(
                summary,
                banner_kind,
            )
            self._show_calibration_popup_once(
                transaction_id,
                result_kind,
                title,
                f"{summary}。\n\n"
                f"模型：{event.get('mag_model_kind', 'FULL')}\n"
                f"generation：{event.get('generation', 0)}\n"
                f"CRC32：{event.get('crc32', '')}\n"
                f"保存位置：{event.get('saved_path', '')}",
            )
        else:
            self._set_calibration_banner(
                "设备校准成功且记录已保存到电脑，但 ACK 尚未完成，可重连后重新读取",
                "pending",
            )

    def _render_calibration_error(self, event: dict[str, Any]) -> None:
        message = str(event.get("message", "未知校准错误"))
        self._calibration_active = False
        self._calibration_terminal_result = "failure"
        self._set_calibration_banner(f"校准链路错误：{message}", "failure")
        self.calibration_status_var.set(f"校准错误：{message}")
        self._show_calibration_popup_once(
            self._calibration_operation_transaction or 0,
            "failure",
            "地磁校准错误",
            message,
        )
        self._refresh_calibration_controls()
        self._refresh_command_button_state()
        self._refresh_ota_panel()

    def _set_calibration_diagnostics_banner(self, text: str, kind: str) -> None:
        colors = {
            "idle": ("#E5E7EB", "#1F2937"),
            "success": ("#DCFCE7", "#166534"),
            "limited": ("#FEF3C7", "#92400E"),
            "failure": ("#FEE2E2", "#991B1B"),
        }
        background, foreground = colors.get(kind, colors["idle"])
        if hasattr(self, "calibration_diagnostics_result_var"):
            self.calibration_diagnostics_result_var.set(text)
        if hasattr(self, "calibration_diagnostics_banner"):
            self.calibration_diagnostics_banner.configure(
                background=background, foreground=foreground
            )

    def _render_calibration_diagnostics_command(
        self, event: dict[str, Any]
    ) -> None:
        status = str(event.get("status", ""))
        message = str(event.get("message", ""))
        if status == "unsupported":
            self._set_calibration_diagnostics_banner(
                "当前固件不支持诊断摘要；原校准结果不受影响", "idle"
            )
        elif status in {"failed", "busy", "unavailable"}:
            self._set_calibration_diagnostics_banner(
                f"诊断摘要读取未完成：{message}", "limited"
            )
        elif status == "retrying":
            self._set_calibration_diagnostics_banner(
                "设备忙，500 ms 后自动重试一次", "limited"
            )
        else:
            self._set_calibration_diagnostics_banner(
                "正在通过 BLE 读取地磁诊断摘要", "idle"
            )
        self.calibration_diagnostics_status_var.set(
            f"诊断摘要：{status} {message}".strip()
        )

    def _render_calibration_diagnostics(self, event: dict[str, Any]) -> None:
        self._last_calibration_diagnostics = dict(event)
        coverage = int(event.get("coverage_mask", 0))
        directions = int(event.get("direction_mask", 0))
        sensor_errors = int(event.get("sensor_read_errors", 0))
        model = str(event.get("final_model_name", "UNKNOWN"))
        ready = str(event.get("final_ready_name", "UNKNOWN"))
        direction_names = ("-X", "+X", "-Y", "+Y", "-Z", "+Z")
        missing_octants = [
            str(index) for index in range(8) if not (coverage & (1 << index))
        ]
        missing_directions = [
            name
            for index, name in enumerate(direction_names)
            if not (directions & (1 << index))
        ]
        if sensor_errors:
            conclusion = f"优先检查传感器/通信：读取错误 {sensor_errors} 次"
            banner_kind = "failure"
        elif ready == "INSUFFICIENT_SAMPLES":
            conclusion = "有效样本不足，请延长校准运动时间"
            banner_kind = "failure"
        elif ready == "INSUFFICIENT_COVERAGE":
            conclusion = (
                "基础校准未完成，缺少方向："
                + (", ".join(missing_directions) if missing_directions else "未知")
            )
            banner_kind = "failure"
        elif ready == "INSUFFICIENT_AXIS_SPAN":
            conclusion = "至少一个轴的变化跨度不足，请增加该轴翻转幅度"
            banner_kind = "failure"
        elif model == "HARD_IRON":
            conclusion = "基础校准正常，硬铁偏置模型可用"
            banner_kind = "success"
        elif model == "AXIS_ALIGNED":
            conclusion = "兼容记录：轴对齐软铁模型可用"
            banner_kind = "success"
        elif model == "FULL" and ready == "READY":
            conclusion = "兼容记录：旧版完整地磁模型可用"
            banner_kind = "success"
        else:
            conclusion = (
                f"诊断完成：ready={ready}，最终模型={model}"
            )
            banner_kind = "idle"
        self._set_calibration_diagnostics_banner(conclusion, banner_kind)

        mins = list(event.get("min_values") or [0, 0, 0])
        maxs = list(event.get("max_values") or [0, 0, 0])
        spans = list(event.get("spans") or [0, 0, 0])
        lines = [
            conclusion,
            "",
            f"completion = {event.get('completion_name', 'UNKNOWN')}",
            f"final_model = {model}",
            f"sample_count = {event.get('sample_count', 0)}",
            f"rejected_sample_count = {event.get('rejected_sample_count', 0)}",
            f"sensor_read_errors = {sensor_errors}",
            f"elapsed_ms = {event.get('elapsed_ms', 0)}",
            f"fit_attempts = {event.get('fit_attempts', 0)}",
            f"coverage_mask = 0x{coverage:02X}; "
            + (
                "八象限完整"
                if not missing_octants
                else f"缺失象限 bit: {', '.join(missing_octants)}"
            ),
            f"direction_mask = 0x{directions:02X}; "
            + (
                "六方向完整"
                if not missing_directions
                else f"缺失方向: {', '.join(missing_directions)}"
            ),
            f"final_ready = {ready}",
            f"full_solve = {event.get('full_solve_name', 'UNKNOWN')}",
            f"axis_solve = {event.get('axis_solve_name', 'UNKNOWN')}",
            f"hard_solve = {event.get('hard_solve_name', 'UNKNOWN')}",
            f"rms = {float(event.get('rms', 0.0)):.6f} "
            f"(raw={event.get('rms_x1e6', 0)})",
            f"X min/max/span = {mins[0]} / {maxs[0]} / {spans[0]}",
            f"Y min/max/span = {mins[1]} / {maxs[1]} / {spans[1]}",
            f"Z min/max/span = {mins[2]} / {maxs[2]} / {spans[2]}",
            f"diagnostics_crc32 = {event.get('crc32', '')}",
            f"ACK = {event.get('ack_status', '')}",
            f"saved_path = {event.get('saved_path', '')}",
        ]
        if hasattr(self, "calibration_diagnostics_text"):
            self._set_text_widget(
                self.calibration_diagnostics_text, "\n".join(lines)
            )
        self.calibration_diagnostics_status_var.set(
            f"诊断摘要：已保存，ACK {event.get('ack_status', '')}"
        )

    def _render_calibration_diagnostics_error(
        self, event: dict[str, Any]
    ) -> None:
        message = str(event.get("message", "未知诊断错误"))
        self.calibration_diagnostics_status_var.set(
            f"诊断摘要：读取失败 {message}"
        )
        self._set_calibration_diagnostics_banner(
            f"诊断摘要读取失败，可点击“重新读取诊断”：{message}", "failure"
        )

    def _reset_calibration_status(
        self,
        message: str = "等待连接",
        *,
        preserve_result: bool = False,
    ) -> None:
        self._calibration_available = False
        self._calibration_active = False
        if hasattr(self, "calibration_capability_var"):
            self.calibration_capability_var.set("地磁校准服务：未发现")
        if hasattr(self, "calibration_status_var"):
            self.calibration_status_var.set(f"校准状态：{message}")
        if hasattr(self, "calibration_transfer_var"):
            self.calibration_transfer_var.set("传输：-")
        if hasattr(self, "calibration_progress_var"):
            if not preserve_result:
                self.calibration_progress_var.set(0.0)
        if not preserve_result:
            self._calibration_operation_transaction = None
            self._calibration_started_monotonic = None
            self._calibration_last_progress = 0
            self._calibration_device_success_pending = False
            self._calibration_terminal_result = None
            self._set_calibration_banner(message, "idle")
        self._refresh_calibration_controls()

    def _render_gatt_mismatch(self, event: dict[str, Any]) -> None:
        self._gatt_mismatch_active = True
        self._command_found = bool(event.get("command_found"))
        self._ack_subscribed = bool(event.get("ack_subscribed"))
        self._export_subscribed = bool(event.get("export_subscribed"))
        self._rtc_sync_ok = False
        missing = [str(item) for item in event.get("missing", []) or []]
        missing_text = ", ".join(missing) if missing else "unknown"
        detail = str(event.get("detail") or event.get("message") or WINDOWS_GATT_CACHE_HINT)
        legacy_profile_detected = bool(event.get("legacy_profile_detected"))
        address = str(event.get("address") or "")
        if not address:
            idx = self._get_selected_index()
            if idx is not None and 0 <= idx < len(self.devices):
                address = str(self.devices[idx].get("address") or "")
        if address:
            self._gatt_mismatch_blocked_addresses.add(address)

        self.status_var.set("GATT_MISMATCH")
        self.connect_hint_var.set(f"GATT 未完整就绪：{missing_text}；可保留配对重新连接。")
        self.control_status_var.set(f"ZY100 Control Service：GATT_MISMATCH missing={missing_text}")
        self.rtc_sync_var.set("RTC Sync: blocked by GATT_MISMATCH")
        self.command_result_var.set(detail)
        self.transfer_status_var.set("传输状态：GATT_MISMATCH")
        self._gatt_vars["notify"].set("GATT_MISMATCH：ACK/Export Notify 未就绪")
        self._set_command_buttons_enabled(True)
        self.disconnect_button.configure(state="disabled")
        self.scan_button.configure(state="normal")
        self.connect_button.configure(state="normal" if self._get_selected_index() is not None else "disabled")
        self._log(f"ERROR: {detail}")
        warning_key = address or detail
        if warning_key not in self._shown_gatt_mismatch_keys:
            self._shown_gatt_mismatch_keys.add(warning_key)
            messagebox.showwarning("GATT_MISMATCH", detail)

    def _render_rtc_sync(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))

        if status == "syncing":
            self._rtc_sync_ok = False
            user_id = int(event.get("user_id", 1))
            training_id = int(event.get("training_id", 1))
            self.status_var.set("Connected, Syncing RTC")
            self.rtc_sync_var.set(f"RTC Sync: syncing user_id={user_id} training_id={training_id}")
            self.command_result_var.set("指令状态：正在同步设备 RTC")
        elif status == "retry":
            self._rtc_sync_ok = False
            seq = int(event.get("seq", 0))
            retry = int(event.get("retry", 0))
            self.status_var.set("Connected, Syncing RTC")
            self.rtc_sync_var.set(f"RTC Sync: retry={retry} last_seq={seq}")
            self.command_result_var.set(f"指令状态：RTC 同步超时，重试 {retry}/3")
        elif status == "ok":
            self._rtc_sync_ok = True
            self._gatt_mismatch_active = False
            self._export_blocking_commands = False
            user_id = int(event.get("user_id", 1))
            training_id = int(event.get("training_id", 1))
            unix_time_ms = int(event.get("unix_time_ms", 0))
            local_time = (
                datetime.fromtimestamp(unix_time_ms / 1000).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                if unix_time_ms
                else "-"
            )
            self.status_var.set("Ready")
            self.rtc_sync_var.set(
                f"RTC Sync: OK user_id={user_id} training_id={training_id} "
                f"unix_time_ms={unix_time_ms} local_time={local_time}"
            )
            self.command_result_var.set("指令状态：Ready，RTC 已同步")
        elif status == "failed":
            self._rtc_sync_ok = False
            message = str(event.get("message", "设备时间同步失败，请重新连接设备"))
            self.status_var.set("RTC sync failed")
            self.rtc_sync_var.set(f"RTC Sync: failed - {message}")
            self.command_result_var.set(message)
            messagebox.showerror("RTC Sync", message)

        self._refresh_command_button_state()

    def _render_command_write_result(self, event: dict[str, Any]) -> None:
        cmd_name = str(event.get("cmd_name", ""))
        seq = int(event.get("seq", 0))
        payload_hex = str(event.get("payload_hex", ""))
        if event.get("ok"):
            response = "response=True" if event.get("response") else "response=False"
            self.last_sent_var.set(f"最近发送：{cmd_name} seq={seq} {response}")
            self.command_result_var.set("指令状态：已写入，等待 ACK")
            self._append_command_history(f"TX {cmd_name} seq={seq} {response}\n{payload_hex}")
        else:
            error = str(event.get("error", ""))
            if cmd_name == command_name(CMD_START_CAPTURE):
                self._start_command_pending = False
                self._refresh_command_button_state()
            if cmd_name == command_name(CMD_OFFLINE_CAPTURE_START):
                self._offline_start_command_pending = False
                self._refresh_command_button_state()
            self.command_result_var.set(f"指令状态：发送失败 {error}")
            self._append_command_history(f"TX_FAIL {cmd_name} seq={seq} error={error}\n{payload_hex}")

    def _render_shipping_status(self, event: dict[str, Any]) -> None:
        status = str(event.get("status") or "")
        message = str(event.get("message") or "")
        if status == "accepted":
            self.command_result_var.set("指令状态：已接受进入船运模式，等待设备断开")
            self.status_var.set("船运模式：设备即将断开")
        elif status == "disconnected":
            self.command_result_var.set("指令状态：船运模式已执行，连接已断开")
            self.status_var.set("连接已断开")
        else:
            self.command_result_var.set(message or "进入船运模式失败")
        if message:
            self._append_command_history(f"SHIPPING {status}: {message}")
        self._refresh_command_button_state()

    def _render_ota_dfu_entry_status(self, event: dict[str, Any]) -> None:
        message = str(event.get("message", "正在进入 DFU"))
        self.status_var.set(f"OTA：{message}")
        self.ota_manager.update_enter_dfu_progress(message)
        self._refresh_ota_panel()

    def _render_ota_dfu_entry_write_result(self, event: dict[str, Any]) -> None:
        payload_hex = str(event.get("payload_hex", ""))
        target = str(event.get("target", ""))
        uuid = str(event.get("uuid", ""))
        if event.get("ok"):
            response = "response=True" if event.get("response") else "response=False"
            detail = " ".join(part for part in (response, target, uuid) if part)
            self.last_sent_var.set(f"最近发送：ENTER_OTA_DFU {detail}")
            self.command_result_var.set("指令状态：进入 DFU 指令已写入，等待 ACK 或设备断开")
            self._append_command_history(f"TX ENTER_OTA_DFU {detail}\n{payload_hex}")
            return

        error = str(event.get("error", ""))
        self.command_result_var.set(f"指令状态：进入 DFU 指令发送失败 {error}")
        detail = " ".join(part for part in (target, uuid, f"error={error}") if part)
        self._append_command_history(f"TX_FAIL ENTER_OTA_DFU {detail}\n{payload_hex}")

    def _render_ota_dfu_entry_ack(self, event: dict[str, Any]) -> None:
        raw = str(event.get("raw_hex", ""))
        uuid = str(event.get("uuid", ""))
        self.command_result_var.set("指令状态：进入 DFU ACK OK")
        self.ota_manager.update_enter_dfu_progress("进入 DFU ACK 已收到，等待普通 BLE 断开")
        self._set_text_widget(
            self.ack_detail_text,
            "\n".join(
                [
                    "ENTER_OTA_DFU ACK OK",
                    "",
                    f"uuid = {uuid}",
                    f"raw = {raw}",
                ]
            ),
        )
        self._append_command_history(f"RX ENTER_OTA_DFU_ACK\n{raw}")
        self._refresh_ota_panel()

    def _handle_ota_dfu_entry_result(self, event: dict[str, Any]) -> None:
        message = str(event.get("message", "进入 DFU 指令完成"))
        self.ota_manager.update_enter_dfu_progress(message)
        self.status_var.set(f"OTA：{message}")
        self._refresh_ota_panel()

        if event.get("ok") and event.get("ready_for_dfu_scan"):
            self._start_dfu_upgrade_current_image(after_entry_command=True)
            return

        self._ota_operation_active = False
        self.ota_manager.fail_enter_dfu_command(message)
        self._refresh_ota_panel()
        if self._command_found:
            self._show_connected_view()
        else:
            self._show_connect_view("进入 DFU 指令失败，请检查连接后重试。")
        messagebox.showerror("OTA", message)

    def _render_ack(self, event: dict[str, Any]) -> None:
        ack = dict(event.get("ack", {}) or {})
        cmd_echo = int(ack.get("cmd_echo", 0))
        status = int(ack.get("status", 0))
        exec_mode = int(ack.get("exec_mode", 0))
        detail = int(ack.get("detail", 0))
        device_state = int(ack.get("device_state", 0))
        device_state_text = str(ack.get("device_state_text", ""))
        if (
            device_state != DEVICE_STATE_WAIT_START
            or (cmd_echo == CMD_START_CAPTURE and status != 0)
            or (cmd_echo == CMD_STATE_NOTIFY and device_state == DEVICE_STATE_WAIT_START)
        ):
            self._start_command_pending = False
        if cmd_echo == CMD_OFFLINE_CAPTURE_START:
            if status != 0 or exec_mode != 0x03:
                self._offline_start_command_pending = False
        if device_state == DEVICE_STATE_OFFLINE_CAPTURING:
            self._offline_start_command_pending = False
        self._last_device_state = device_state
        self._refresh_command_button_state()
        self._refresh_calibration_controls()
        if cmd_echo == CMD_STATE_NOTIFY:
            conclusion = f"STATE_NOTIFY device_state=0x{device_state:02X} ({device_state_text})"
        elif cmd_echo == CMD_TIME_SYNC and status == 0x00 and exec_mode == 0x02:
            conclusion = "RTC TIME_SYNC ACK OK"
        elif status == 0x00:
            conclusion = "ACK OK"
        else:
            conclusion = "ACK ERROR"
        lines = [
            conclusion,
            "",
            f"uuid            = {ack.get('uuid', '')}",
            f"cmd_echo        = 0x{cmd_echo:02X} ({ack.get('cmd_name', '')})",
            f"seq_echo        = {ack.get('seq_echo', '')}",
            f"status          = 0x{status:02X} ({ack.get('status_text', '')})",
            f"device_state    = 0x{device_state:02X} ({device_state_text})",
            f"exec_mode       = 0x{exec_mode:02X} ({ack.get('exec_mode_text', '')})",
            f"user_id_echo    = {ack.get('user_id', '')}",
            f"training_id     = {ack.get('training_id', '')}",
            f"detail          = {detail}",
            "",
            f"raw             = {ack.get('raw_hex', '')}",
        ]
        prefix = "设备状态" if cmd_echo == CMD_STATE_NOTIFY else "指令状态"
        self.command_result_var.set(f"{prefix}：{conclusion}")
        self._set_text_widget(self.ack_detail_text, "\n".join(lines))
        history_tag = "RX STATE_NOTIFY" if cmd_echo == CMD_STATE_NOTIFY else "RX ACK"
        self._append_command_history(
            f"{history_tag} {ack.get('cmd_name', '')} seq={ack.get('seq_echo', '')} "
            f"status=0x{status:02X} exec=0x{exec_mode:02X}\n{ack.get('raw_hex', '')}"
        )

    def _render_ack_error(self, event: dict[str, Any]) -> None:
        lines = [
            "ACK 解析失败",
            "",
            f"uuid = {event.get('uuid', '')}",
            f"error = {event.get('error', '')}",
            f"raw = {event.get('raw_hex', '')}",
        ]
        self.command_result_var.set("指令状态：ACK 解析失败")
        self._set_text_widget(self.ack_detail_text, "\n".join(lines))
        self._append_command_history(f"RX_ACK_ERR {event.get('error', '')}\n{event.get('raw_hex', '')}")

    def _render_offline_v2_status(self, event: dict[str, Any]) -> None:
        status = str(event.get("status") or "")
        state = str(event.get("state") or "")
        session_id = int(event.get("session_id") or 0)
        generation = int(event.get("generation") or event.get("list_generation") or 0)
        offset = int(event.get("offset") or 0)
        received_offset = int(event.get("received_offset") or offset)
        total_bytes = int(event.get("total_bytes") or 0)
        reason = str(event.get("reason") or "")
        error_category = str(event.get("error_category") or "")

        active_states = {
            "listing",
            "begin",
            "resuming",
            "receiving",
            "local_finalize",
            "final_confirm",
            "reclaiming",
            "wait_capture",
        }
        if state in active_states or status == "device_busy":
            self._export_blocking_commands = True
        elif status in {"all_complete", "legacy_unsupported"}:
            self._export_blocking_commands = False

        status_text = {
            "capture_preempted": "本地离线采集已抢占，等待采集结束",
            "capture_finished": "采集结束，按最新场次重新上传",
            "resume_reset_required": "设备要求旧场从头重传",
            "list_page": "读取 Session 列表",
            "all_complete": "全部 Session 已同步",
            "device_busy": "设备正在离线采集，等待采集结束",
            "receiving": "接收事件流",
            "resumed": "已从本地断点恢复",
            "resume_accepted": "设备已接受断点",
            "acknowledged": "累计 ACK 已确认",
            "local_finalize": "本地校验与原子落盘",
            "saved_local": "本地 Session 已完成",
            "reclaim_status": "设备回收进度",
            "sync_abort": "设备中止本次同步，保留断点",
            "legacy_unsupported": "旧 FEUF 离线协议不支持",
            "error": "离线 V2 同步错误",
        }.get(status, status or state)
        detail = f"离线 V2：{status_text}"
        if session_id:
            detail += f" session=0x{session_id:08X}"
        if generation:
            detail += f" generation={generation}"
        if error_category:
            detail += f" category={error_category}"
        if reason:
            detail += f" reason={reason}"
        health = str(event.get("health") or "")
        if health:
            health_text = {
                "normal": "正常",
                "controlled_stop": "受控停止",
                "abnormal_finalized": "异常收尾",
                "recovered_prefix": "掉电恢复前缀",
            }.get(health, health)
            detail += f" health={health_text}"
            if health != "normal":
                detail = "⚠ " + detail
        self.transfer_status_var.set(detail)

        if total_bytes:
            percent = min(100.0, (received_offset / total_bytes) * 100.0)
            self.transfer_progress_var.set(
                f"当前传输：{received_offset}/{total_bytes} bytes ({percent:.1f}%)，"
                f"持久化边界={offset}"
            )
        elif status == "list_page":
            self.transfer_progress_var.set(
                "LIST 分页："
                f"cursor={int(event.get('cursor') or 0)} "
                f"next={int(event.get('next_cursor') or 0)} "
                f"total={int(event.get('session_count') or 0)}"
            )
        elif status == "capture_preempted":
            self.transfer_progress_var.set("上传/回收已暂停；采集结束自动继续，被打断旧场从头重传")
        elif status == "reclaim_status":
            self.transfer_progress_var.set(
                "设备回收："
                f"state={'已暂停' if event.get('reclaim_state') == 'paused' else event.get('reclaim_state', '')} "
                f"{int(event.get('erased_bytes') or 0)}/{int(event.get('extent_bytes') or 0)} bytes"
            )
        elif status in {"local_finalize", "saved_local", "all_complete"}:
            self.transfer_progress_var.set(f"当前传输：{status_text}")

        if status in {"saved_local", "reclaim_status", "all_complete"}:
            self._refresh_session_list()

    def _render_transfer_status(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", ""))
        export_id = int(event.get("export_id") or 0)
        bytes_received = int(event.get("bytes_received") or 0)
        estimated = int(event.get("estimated_total_bytes") or 0)
        chunk_count = int(event.get("chunk_count") or 0)
        session_key = str(event.get("session_key", ""))
        reason = str(event.get("reason", ""))
        device_ready = bool(event.get("device_ready", False))

        blocking_statuses = {
            "receiving",
            "verifying",
            "saving_local",
            "saved_local",
            "confirm_sent",
            "confirm_ack_ok",
            "reclaiming",
            "reclaim_unknown",
            "reclaim_failed",
            "clear_requested",
            "device_clear_pending",
            "device_clear_waiting",
            "device_clear_unknown",
            "transfer_failed",
            "clear_failed",
            "device_clear_failed",
        }
        if status in blocking_statuses:
            self._export_blocking_commands = True
        if status == "reclaim_ok":
            self._export_blocking_commands = not device_ready
        if status in {"needs_device_clear", "clear_ok", "device_clear_ok", "device_clear_late_ok"}:
            self._export_blocking_commands = False

        status_text = {
            "receiving": "接收中",
            "verifying": "校验中",
            "saving_local": "本地保存中",
            "saved_local": "本地已保存",
            "confirm_sent": "已发送 EXPORT_CONFIRM",
            "confirm_ack_ok": "导出已确认，等待设备空闲",
            "reclaiming": "设备 tail reclaim 中",
            "reclaim_ok": "设备 tail reclaim 完成",
            "reclaim_unknown": "设备 tail reclaim 未确认",
            "reclaim_failed": "设备 tail reclaim 失败",
            "needs_device_clear": "导出已确认",
            "clear_requested": "已发送 CLEAR_FLASH",
            "clear_ok": "设备清理完成",
            "clear_failed": "设备清理失败",
            "device_clear_pending": "设备清理中",
            "device_clear_waiting": "等待设备清理确认",
            "device_clear_ok": "设备清理完成",
            "device_clear_late_ok": "稍后确认设备清理完成",
            "device_clear_unknown": "设备清理暂未确认",
            "transfer_failed": "传输失败",
            "device_clear_failed": "设备清理失败",
        }.get(status, status)

        detail = f"传输状态：{status_text}"
        if export_id:
            detail += f" export_id={export_id}"
        if session_key:
            detail += f" session={session_key}"
        if reason:
            detail += f" reason={reason}"
        self.transfer_status_var.set(detail)

        if status == "receiving":
            if estimated:
                percent = (bytes_received / estimated) * 100.0
                display_percent = min(percent, 100.0)
                over_estimate = "，已超过预估" if bytes_received > estimated else ""
                self.transfer_progress_var.set(
                    f"当前传输：{bytes_received}/{estimated} bytes 预估({display_percent:.1f}%{over_estimate}) "
                    f"chunks={chunk_count}"
                )
            else:
                self.transfer_progress_var.set(f"当前传输：{bytes_received} bytes chunks={chunk_count}")
        elif status == "verifying":
            total_bytes = int(event.get("total_bytes") or 0)
            crc = int(event.get("stream_crc32") or 0)
            self.transfer_progress_var.set(
                f"当前传输：EXPORT_END 实际 {total_bytes} bytes crc=0x{crc:08X} chunks={chunk_count}"
            )
        elif status in {"saving_local", "saved_local", "confirm_sent"}:
            self.transfer_progress_var.set(f"当前传输：export_id={export_id} {status_text}")
        elif status == "reclaiming":
            self.transfer_progress_var.set("当前传输：EXPORT_CONFIRM 已处理，等待设备 final async ACK 完成 tail reclaim。")
        elif status == "reclaim_ok":
            if device_ready:
                self.transfer_progress_var.set("当前传输：tail reclaim 已完成，设备已回到 WAIT_START。")
            else:
                self.transfer_progress_var.set("当前传输：tail reclaim 已完成，等待下一组 EXPORT_START 或 PING 返回 WAIT_START。")
        elif status == "reclaim_unknown":
            self.transfer_progress_var.set("当前传输：未收到 reclaim final ACK；本地数据保持完成，重连后用下一组 EXPORT_START 或 PING 确认设备状态。")
        elif status == "reclaim_failed":
            self.transfer_progress_var.set("当前传输：设备 tail reclaim 确认失败，请检查设备状态或重新连接。")
        elif status == "confirm_ack_ok":
            self.transfer_progress_var.set("当前传输：导出确认已被设备接受，等待 WAIT_START 或下一组自动导出。")
        elif status == "needs_device_clear":
            self.transfer_progress_var.set("当前传输：数据已完整保存并确认；新协议由设备 tail reclaim 释放空间，不需要 CLEAR_FLASH。")
        elif status == "clear_requested":
            self.transfer_progress_var.set("当前传输：已请求 CLEAR_FLASH，等待设备返回异步完成 ACK。")
        elif status == "clear_ok":
            self.transfer_progress_var.set("当前传输：显式 CLEAR_FLASH 已完成，设备 Flash 已清理。")
        elif status == "clear_failed":
            self.transfer_progress_var.set("当前传输：CLEAR_FLASH 返回失败，请检查设备状态或断开重连后重试。")
        elif status == "device_clear_pending":
            self.transfer_progress_var.set("当前传输：数据已完整保存，设备正在清理中...")
        elif status == "device_clear_waiting":
            self.transfer_progress_var.set("当前传输：数据已完整保存，设备清理时间较长，可继续等待或稍后重连确认。")
        elif status == "device_clear_unknown":
            self.transfer_progress_var.set("当前传输：数据已完整保存，但暂未确认设备清理完成；重新连接后会自动确认。")
        elif status in {"device_clear_ok", "device_clear_late_ok"}:
            self.transfer_progress_var.set("当前传输：数据已完整保存，设备清理已完成。")
        elif status == "transfer_failed":
            self.transfer_progress_var.set(f"当前传输：{status_text}，请断开重连后让设备重传")
        elif status == "device_clear_failed":
            self.transfer_progress_var.set("当前传输：数据已保存，但设备清理返回失败，请检查设备状态或手动清理。")

        self._refresh_command_button_state()

    def _render_online_stream_status(self, event: dict[str, Any]) -> None:
        status = str(event.get("status", "idle"))
        session_id = int(event.get("session_id") or 0)
        ready_sent = bool(event.get("ready_sent"))
        ready_ack_ok = bool(event.get("ready_ack_ok"))
        active = bool(event.get("active"))
        bps = int(event.get("bps") or 0)
        store_dir = str(event.get("store_dir") or "")
        last_error = str(event.get("last_error") or "")
        fallback = str(event.get("fallback_reason") or "")
        operation_id = int(event.get("operation_id") or 0)
        attempt = int(event.get("attempt") or 0)
        start_operation_state = str(
            event.get("start_operation_state") or "IDLE"
        )
        if "start_operation_pending" in event:
            self._start_command_pending = bool(
                event.get("start_operation_pending")
            )
            self._refresh_command_button_state()

        raw_count = int(event.get("raw_count") or 0)
        event_count = int(event.get("event_count") or 0)
        summary_count = int(event.get("summary_count") or 0)
        raw_bytes = int(event.get("raw_bytes") or 0)
        event_bytes = int(event.get("event_bytes") or 0)
        summary_bytes = int(event.get("summary_bytes") or 0)
        raw_ack = int(event.get("raw_ack") or 0)
        event_ack = int(event.get("event_ack") or 0)
        summary_ack = int(event.get("summary_ack") or 0)
        end_ack = int(event.get("end_ack") or 0)
        last_record_type = str(event.get("last_record_type") or "-")
        last_record_id = int(event.get("last_record_id") or 0)
        pending_summary = int(event.get("summary_pending") or 0)
        packet_count_value = event.get("packet_count") if "packet_count" in event else None
        mag_sample_count = int(event.get("mag_sample_count") or 0)
        mag_read_errors = int(event.get("mag_read_error_count") or 0)
        mag_missed = int(event.get("mag_missed_deadline_count") or 0)
        time_analysis = event.get("time_analysis")
        if not isinstance(time_analysis, dict):
            time_analysis = None

        detail_parts = [
            f"Online: {status}",
            "mode=serialized",
            f"ready_sent={1 if ready_sent else 0}",
            f"ready_ack={1 if ready_ack_ok else 0}",
            f"active={1 if active else 0}",
        ]
        if operation_id:
            detail_parts.append(f"op={operation_id}")
            detail_parts.append(f"attempt={attempt}")
            detail_parts.append(f"start_op={start_operation_state}")
        if fallback:
            detail_parts.append(f"fallback={fallback}")
        if last_error:
            detail_parts.append(f"error={last_error}")
        self._online_ui_active = active
        self.online_status_var.set(" | ".join(detail_parts))
        total_count = raw_count + event_count + summary_count
        packet_count = int(packet_count_value or 0) if packet_count_value is not None else total_count
        total_bytes = raw_bytes + event_bytes + summary_bytes
        total_ack = raw_ack + event_ack + summary_ack
        self.online_counts_var.set(
            f"IMU 800Hz包：{packet_count} | 地磁100Hz样本：{mag_sample_count} | Record：{total_count}"
        )
        self.online_bytes_var.set(f"采样数据：{total_bytes} Bytes")
        self.online_ack_var.set(f"接收确认：{total_ack} | 结束确认：{end_ack}")
        if time_analysis is not None and bool(time_analysis.get("valid")):
            interval_us = float(time_analysis.get("average_interval_us") or 0.0)
            rate_hz = float(time_analysis.get("average_rate_hz") or 0.0)
            ppm = float(time_analysis.get("deviation_ppm_from_800") or 0.0)
            duration_s = float(time_analysis.get("duration_s") or 0.0)
            self.online_rate_var.set(
                "Fixed40 Unix估算："
                f"{interval_us:.3f} us/包 | {rate_hz:.6f} Hz | {ppm:+.2f} ppm"
            )
        elif time_analysis is not None:
            reason = str(time_analysis.get("invalid_reason") or "unknown")
            duration_s = 0.0
            self.online_rate_var.set(f"Fixed40 Unix估算：不可计算（{reason}）")
        else:
            duration_s = 0.0
            self.online_rate_var.set(f"实时速率：{bps} B/s")
        session_text = f"Session: 0x{session_id:08X}" if session_id else "Session: -"
        session_text += f" | last={last_record_type}#{last_record_id}"
        if mag_sample_count or mag_read_errors or mag_missed:
            session_text += (
                f" | MAG err={mag_read_errors} missed={mag_missed}"
            )
        if time_analysis is not None and bool(time_analysis.get("valid")):
            unix_start = str(time_analysis.get("first_unix_time_local") or "")
            unix_end = str(time_analysis.get("last_unix_time_local") or "")
            if unix_start and unix_end:
                session_text += f" | Unix={unix_start} ~ {unix_end}"
        if time_analysis is not None and bool(time_analysis.get("valid")):
            session_text += f" | Fixed40时长={duration_s:.6f}s"
        if store_dir:
            session_text += f" | store={store_dir}"
        self.online_detail_var.set(session_text)

        self._online_chart_values = {
            "IMU 800Hz": packet_count,
            "MAG 100Hz": mag_sample_count,
        }
        now_ms = int(datetime.now().timestamp() * 1000)
        if now_ms - self._online_last_redraw_ms >= 100:
            self._online_last_redraw_ms = now_ms
            self._draw_online_chart()

    def _reset_online_stream_status(self) -> None:
        if not hasattr(self, "online_status_var"):
            return
        self.online_status_var.set("Online: idle")
        self.online_counts_var.set("IMU 800Hz包：0 | 地磁100Hz样本：0 | Record：0")
        self.online_bytes_var.set("采样数据：0 Bytes")
        self.online_ack_var.set("接收确认：0 | 结束确认：0")
        self.online_rate_var.set("实时速率：0 B/s")
        self.online_detail_var.set("Session: -")
        self._online_ui_active = False
        self._online_chart_values = {"IMU 800Hz": 0, "MAG 100Hz": 0}
        self._draw_online_chart()

    def _draw_online_chart(self) -> None:
        canvas = getattr(self, "online_chart_canvas", None)
        if canvas is None:
            return
        try:
            width = max(320, int(canvas.winfo_width()))
            height = max(220, int(canvas.winfo_height()))
        except tk.TclError:
            return
        canvas.delete("all")
        margin_x = 54
        margin_y = 32
        baseline = height - margin_y
        chart_height = max(80, height - margin_y * 2)
        values = dict(getattr(self, "_online_chart_values", {}) or {})
        max_value = max(1, max(values.values(), default=0))
        labels = ("800Hz DATA",)
        colors = {"800Hz DATA": "#3B82F6"}
        slot = max(70, (width - margin_x * 2) // len(labels))
        bar_width = min(110, int(slot * 0.55))
        canvas.create_line(margin_x, baseline, width - margin_x + 16, baseline, fill="#A0A0A0")
        canvas.create_text(margin_x, margin_y - 12, text="800Hz 在线采集数据", anchor="w", fill="#333333", font=("Consolas", 11, "bold"))
        for idx, label in enumerate(labels):
            value = int(values.get(label, 0))
            x_center = margin_x + slot * idx + slot // 2
            bar_h = int((value / max_value) * chart_height) if value else 0
            x0 = x_center - bar_width // 2
            y0 = baseline - bar_h
            x1 = x_center + bar_width // 2
            canvas.create_rectangle(x0, y0, x1, baseline, fill=colors[label], outline="")
            canvas.create_text(x_center, max(margin_y, y0 - 14), text=str(value), fill="#222222", font=("Consolas", 11, "bold"))
            canvas.create_text(x_center, baseline + 18, text=label, fill="#333333", font=("Consolas", 10))

    def _maybe_refresh_online_session_list(self, status: str) -> None:
        if not hasattr(self, "online_session_tree"):
            return
        snapshot = getattr(self, "_online_list_snapshot", None)
        if isinstance(snapshot, dict) and snapshot.get("session_key"):
            sessions = list(getattr(self, "_online_sessions", []))
            key = snapshot["session_key"]
            index = next((i for i, row in enumerate(sessions) if row.get("session_key") == key), None)
            if index is None:
                self._render_online_session_list([dict(snapshot)] + sessions)
            else:
                session = {**sessions[index], **snapshot}
                self._online_sessions[index] = session
                self.online_session_tree.item(str(index), values=self._online_session_row(session))
        if status in {"end", "error", "aborted_partial", "fallback_offline"}:
            self._refresh_online_session_list()

    def _refresh_online_session_list(self) -> None:
        if not hasattr(self, "online_session_store") or getattr(self, "_is_closing", False):
            return
        if getattr(self, "_online_ui_active", False):
            return  # Live counters arrive with persisted snapshots, without a disk scan.
        if self._online_list_future is not None:
            self._online_list_refresh_pending = True
            return
        self._online_list_requested_revision = self._online_list_revision
        self._online_list_future = self._online_list_executor.submit(self.online_session_store.list_sessions)

    def _poll_online_session_list(self) -> None:
        future = self._online_list_future
        if future is None or not future.done():
            return
        self._online_list_future = None
        try:
            sessions = future.result()
            if self._online_list_requested_revision != self._online_list_revision:
                self._online_list_refresh_pending = True
            elif not getattr(self, "_online_ui_active", False):
                self._render_online_session_list(sessions)
        except Exception as exc:
            self.online_sessions_var.set(f"Online sessions: read failed: {exc}")
        if self._online_list_refresh_pending:
            self._online_list_refresh_pending = False
            self._refresh_online_session_list()

    def _online_session_row(self, session: dict[str, Any]) -> tuple:
        return (
            self._format_online_session_id(session.get("session_id")),
            session.get("user_id", ""),
            session.get("training_id", ""),
            session.get("unix_start_local") or session.get("host_started_at", ""),
            session.get("status", ""),
            self._format_online_counts(session),
            self._format_online_bytes(session),
        )

    def _render_online_session_list(self, sessions: list[dict[str, Any]]) -> None:
        self._online_sessions = sessions
        if hasattr(self, "online_sessions_var"):
            self.online_sessions_var.set(
                f"Online sessions: {len(self._online_sessions)} | root={self.online_session_store.root}"
            )
        self._online_last_list_refresh_ms = int(datetime.now().timestamp() * 1000)
        if not hasattr(self, "online_session_tree"):
            return

        selected_keys = set(getattr(self, "_selected_online_session_keys", ()))
        selected_key = getattr(self, "_selected_online_session_key", None)
        if selected_key:
            selected_keys.add(selected_key)
        self.online_session_tree.delete(*self.online_session_tree.get_children())
        selected_iids: list[str] = []
        for idx, session in enumerate(self._online_sessions):
            key = str(session.get("session_key", ""))
            values = self._online_session_row(session)
            iid = str(idx)
            self.online_session_tree.insert("", tk.END, iid=iid, values=values, tags=(key,))
            if key in selected_keys:
                selected_iids.append(iid)

        if selected_iids:
            self.online_session_tree.selection_set(*selected_iids)
            self.online_session_tree.focus(selected_iids[0])
            self.online_session_tree.see(selected_iids[0])
            self._selected_online_session_keys = [
                str(self._online_sessions[int(iid)].get("session_key", "")) for iid in selected_iids
            ]
            self._selected_online_session_key = self._selected_online_session_keys[0]
        elif self._online_sessions:
            self.online_session_tree.selection_set("0")
            self.online_session_tree.focus("0")
            self._selected_online_session_key = str(self._online_sessions[0].get("session_key", ""))
            self._selected_online_session_keys = [self._selected_online_session_key]
        else:
            self._selected_online_session_key = None
            self._selected_online_session_keys = []

    def _on_online_session_selected(self, _: Any) -> None:
        selection = self.online_session_tree.selection()
        if not selection:
            self._selected_online_session_key = None
            self._selected_online_session_keys = []
            return
        sessions = self._get_selected_online_sessions()
        self._selected_online_session_keys = [str(session.get("session_key", "")) for session in sessions]
        self._selected_online_session_key = self._selected_online_session_keys[0] if sessions else None

    def _download_online_session(self) -> None:
        if not self._online_file_job_available():
            return
        sessions = self._get_selected_online_sessions()
        if any(row.get("status") in {"active", "ending"} for row in sessions):
            messagebox.showwarning("采集文件处理中", "请等待该场采集停止并完成文件收尾后再操作。")
            return
        if not sessions:
            messagebox.showinfo("提示", "请先选择在线采集记录。")
            return
        downloadable = [session for session in sessions if session.get("can_download")]
        if not downloadable:
            messagebox.showwarning("提示", "该在线采集记录缺少 metadata 或已标记异常，不能下载。")
            return

        if len(sessions) == 1:
            key = str(downloadable[0].get("session_key", ""))
            path = filedialog.asksaveasfilename(
                title="下载在线采集记录",
                defaultextension=".zip",
                filetypes=[("ZIP 文件", "*.zip"), ("所有文件", "*.*")],
                initialfile=f"{key}.zip",
            )
            if not path:
                return
            self._start_online_file_job("导出", [(key, Path(path))])
            return

        destination = filedialog.askdirectory(title=f"选择 {len(downloadable)} 条在线采集记录的下载文件夹")
        if not destination:
            return
        jobs = [(str(row.get("session_key", "")),
                 Path(destination) / f"{row.get('session_key', '')}.zip") for row in downloadable]
        self._start_online_file_job("导出", jobs)

    def _delete_online_session(self) -> None:
        if not self._online_file_job_available():
            return
        sessions = self._get_selected_online_sessions()
        if any(row.get("status") in {"active", "ending"} for row in sessions):
            messagebox.showwarning("采集文件处理中", "请等待该场采集停止并完成文件收尾后再操作。")
            return
        if not sessions:
            messagebox.showinfo("提示", "请先选择在线采集记录。")
            return
        keys = [str(session.get("session_key", "")) for session in sessions]
        target = keys[0] if len(keys) == 1 else f"选中的 {len(keys)} 条记录"
        if not messagebox.askyesno(
            "确认删除",
            f"仅删除本机保存的在线采集数据，不会清除设备端数据。\n\n删除{target}？",
        ):
            return
        self._start_online_file_job("删除", [(key, None) for key in keys])

    def _online_file_job_available(self) -> bool:
        if getattr(self, "_is_closing", False):
            return False
        if (getattr(self, "_online_ui_active", False)
                or getattr(self, "_start_command_pending", False)):
            messagebox.showwarning("采集进行中", "请在采集结束后导出或删除文件，避免磁盘操作影响接收。")
            return False
        if getattr(self, "_online_file_future", None) is not None:
            messagebox.showinfo("文件处理中", "后台正在处理文件，请稍候；界面和连接仍可操作。")
            return False
        return True

    def _start_online_file_job(self, operation: str, jobs: list) -> None:
        # File/confirmation dialogs can pump Tk events. Recheck after the
        # dialog in case a capture or application shutdown started meanwhile.
        if not self._online_file_job_available():
            return
        # Capture the selection on the UI thread; the worker never accesses Tk.
        jobs = tuple(jobs)
        store = self.online_session_store

        def run():
            failures = []
            for key, destination in jobs:
                try:
                    if operation == "删除":
                        store.delete_session(key)
                    else:
                        store.write_session_zip(key, destination)
                except Exception as exc:
                    failures.append(f"{key}: {exc}")
            return len(jobs) - len(failures), failures

        self._online_file_operation = operation
        self._online_file_future = self._online_list_executor.submit(run)
        self.online_sessions_var.set(f"正在后台{operation} {len(jobs)} 条记录…")
        self._refresh_command_button_state()

    def _poll_online_file_job(self) -> None:
        future = getattr(self, "_online_file_future", None)
        if future is None or not future.done():
            return
        self._online_file_future = None
        operation = self._online_file_operation
        try:
            saved, failures = future.result()
        except Exception as exc:
            saved, failures = 0, [str(exc)]
        if operation == "删除":
            self._selected_online_session_key = None
            self._selected_online_session_keys = []
        self._online_list_revision += 1
        self._refresh_online_session_list()
        self._refresh_command_button_state()
        message = f"{operation}完成：成功 {saved} 条，失败 {len(failures)} 条。"
        self.online_sessions_var.set(message)
        if not getattr(self, "_is_closing", False):
            if failures:
                messagebox.showerror(f"{operation}结果", message + "\n\n" + "\n".join(failures))
            else:
                messagebox.showinfo(f"{operation}结果", message)

    def _get_selected_online_sessions(self) -> list[dict[str, Any]]:
        selection = self.online_session_tree.selection() if hasattr(self, "online_session_tree") else ()
        indexes: list[int] = []
        for iid in selection:
            try:
                idx = int(iid)
            except (TypeError, ValueError):
                continue
            if 0 <= idx < len(self._online_sessions):
                indexes.append(idx)
        return [self._online_sessions[idx] for idx in sorted(set(indexes))]

    def _get_selected_online_session(self) -> dict[str, Any] | None:
        sessions = self._get_selected_online_sessions()
        return sessions[0] if sessions else None

    @staticmethod
    def _format_online_session_id(value: Any) -> str:
        try:
            return f"0x{int(value or 0):08X}"
        except (TypeError, ValueError):
            return "-"

    @staticmethod
    def _format_online_counts(session: dict[str, Any]) -> str:
        if "packet_count" in session:
            return str(int(session.get("packet_count") or 0))
        if "sample_count" in session:
            return str(int(session.get("sample_count") or 0))
        counts = session.get("counts") if isinstance(session.get("counts"), dict) else {}
        return str(sum(int(counts.get(kind) or 0) for kind in ("raw", "event", "summary")))

    @staticmethod
    def _format_online_bytes(session: dict[str, Any]) -> str:
        if "sample_bytes" in session:
            return str(int(session.get("sample_bytes") or 0))
        byte_counts = session.get("bytes") if isinstance(session.get("bytes"), dict) else {}
        return str(sum(int(byte_counts.get(kind) or 0) for kind in ("raw", "event", "summary")))

    def _refresh_session_list(self) -> None:
        try:
            self._sessions = self.session_store.list_sessions()
        except Exception as exc:
            self.transfer_detail_var.set(f"本地 session：读取失败 {exc}")
            return

        if not hasattr(self, "session_tree"):
            return
        selected_keys = set(getattr(self, "_selected_session_keys", ()))
        selected_key = getattr(self, "_selected_session_key", None)
        if selected_key:
            selected_keys.add(selected_key)
        self.session_tree.delete(*self.session_tree.get_children())
        selected_iids: list[str] = []
        for idx, session in enumerate(self._sessions):
            key = str(session.get("session_key", ""))
            values = (
                self._format_online_session_id(session.get("session_id")),
                session.get("generation", ""),
                (
                    session.get("owner_user_id")
                    if session.get("owner_user_id_known", False)
                    else "未知（旧数据）"
                ),
                self._format_ms(session.get("start_unix_ms")),
                self._format_duration(session.get("duration_ms")),
                session.get("event_count", ""),
                session.get("logical_bytes", session.get("total_bytes", "")),
                f"0x{int(session.get('stream_crc32') or 0):08X}" if session.get("stream_crc32") is not None else "",
                session.get("transfer_status", ""),
                session.get("reclaim_status", ""),
            )
            iid = str(idx)
            self.session_tree.insert("", tk.END, iid=iid, values=values, tags=(key,))
            if key in selected_keys:
                selected_iids.append(iid)

        self.transfer_detail_var.set(f"本地 session：{len(self._sessions)} 条，目录：{self.session_store.root}")
        if selected_iids:
            self.session_tree.selection_set(*selected_iids)
            self.session_tree.focus(selected_iids[0])
            self.session_tree.see(selected_iids[0])
            self._selected_session_keys = [
                str(self._sessions[int(iid)].get("session_key", "")) for iid in selected_iids
            ]
            self._selected_session_key = self._selected_session_keys[0]
        elif self._sessions:
            self.session_tree.selection_set("0")
            self.session_tree.focus("0")
            self._selected_session_key = str(self._sessions[0].get("session_key", ""))
            self._selected_session_keys = [self._selected_session_key]
        else:
            self._selected_session_key = None
            self._selected_session_keys = []

    def _on_session_selected(self, _: Any) -> None:
        selection = self.session_tree.selection()
        if not selection:
            self._selected_session_key = None
            self._selected_session_keys = []
            return
        sessions = self._get_selected_sessions()
        self._selected_session_keys = [str(session.get("session_key", "")) for session in sessions]
        self._selected_session_key = self._selected_session_keys[0] if sessions else None

    def _on_download_session(self) -> None:
        session = self._get_selected_session()
        if session is None:
            messagebox.showinfo("提示", "请先选择一个 session。")
            return
        if not session.get("can_download"):
            messagebox.showwarning("提示", "该 session 文件不完整或已标记异常，不能下载。")
            return
        key = str(session.get("session_key", ""))
        path = filedialog.asksaveasfilename(
            title="下载 session",
            defaultextension=".zip",
            filetypes=[("ZIP 文件", "*.zip"), ("所有文件", "*.*")],
            initialfile=f"{key}.zip",
        )
        if not path:
            return
        try:
            self.session_store.write_zip(key, path)
            messagebox.showinfo("提示", f"已保存到：{path}")
        except Exception as exc:
            messagebox.showerror("错误", f"下载失败：{exc}")

    def _on_delete_session(self) -> None:
        sessions = self._get_selected_sessions()
        if not sessions:
            messagebox.showinfo("提示", "请先选择要删除的 session。")
            return
        keys = [str(session.get("session_key", "")) for session in sessions]
        target = keys[0] if len(keys) == 1 else f"选中的 {len(keys)} 条记录"
        if not messagebox.askyesno(
            "确认删除",
            f"仅删除本机保存的 session，不会清除设备 Flash。\n\n删除 {target}？",
        ):
            return
        failures: list[str] = []
        for key in keys:
            try:
                self.session_store.delete_session(key)
            except Exception as exc:
                failures.append(f"{key}: {exc}")
        self._selected_session_key = None
        self._selected_session_keys = []
        self._refresh_session_list()
        if failures:
            messagebox.showerror(
                "批量删除完成（有失败）",
                f"成功 {len(keys) - len(failures)} 条，失败 {len(failures)} 条。\n\n" + "\n".join(failures),
            )

    def _get_selected_sessions(self) -> list[dict[str, Any]]:
        selection = self.session_tree.selection() if hasattr(self, "session_tree") else ()
        indexes: list[int] = []
        for iid in selection:
            try:
                idx = int(iid)
            except (TypeError, ValueError):
                continue
            if 0 <= idx < len(self._sessions):
                indexes.append(idx)
        return [self._sessions[idx] for idx in sorted(set(indexes))]

    def _get_selected_session(self) -> dict[str, Any] | None:
        sessions = self._get_selected_sessions()
        return sessions[0] if sessions else None

    @staticmethod
    def _format_device_clear_status(session: dict[str, Any]) -> str:
        reclaim_status = str(session.get("reclaim_status") or "")
        if reclaim_status:
            return {
                "reclaiming": "reclaiming",
                "reclaim_ok": "reclaim_ok",
                "reclaim_unknown": "reclaim_unknown",
                "reclaim_failed": "reclaim_failed",
            }.get(reclaim_status, reclaim_status)
        clear_status = str(session.get("device_clear_status") or "")
        if not clear_status:
            if session.get("device_clear_verified"):
                clear_status = "clear_ok"
            elif session.get("confirm_ack_status") == "clear_timeout":
                clear_status = "clear_unknown"
            else:
                clear_status = str(session.get("confirm_ack_status") or "")
        return {
            "needs_device_clear": "可选全清",
            "clear_requested": "清理中",
            "clear_pending": "清理中",
            "clear_waiting": "等待确认",
            "clear_ok": "已清理",
            "clear_late_ok": "稍后确认已清理",
            "clear_unknown": "未确认",
            "clear_failed": "清理失败",
            "pending": "确认中",
            "OK": "已确认",
        }.get(clear_status, clear_status)

    @staticmethod
    def _format_ms(value: Any) -> str:
        try:
            ms = int(value or 0)
        except (TypeError, ValueError):
            return "-"
        if ms <= 0:
            return "-"
        return datetime.fromtimestamp(ms / 1000).strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def _format_duration(value: Any) -> str:
        try:
            ms = int(value or 0)
        except (TypeError, ValueError):
            return "-"
        if ms <= 0:
            return "-"
        seconds = ms // 1000
        return f"{seconds // 60:02d}:{seconds % 60:02d}"

    def _append_command_history(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.command_history_text.configure(state="normal")
        self.command_history_text.insert(tk.END, f"[{ts}] {text}\n\n")
        self._trim_text(self.command_history_text, 500)
        self.command_history_text.see(tk.END)
        self.command_history_text.configure(state="disabled")

    def _set_gatt_placeholder(self) -> None:
        self._gatt_vars["service_count"].set("服务数量：0")
        self._gatt_vars["char_count"].set("特征数量：0")
        self._gatt_vars["control"].set("Control Service：未发现")
        self._gatt_vars["command"].set("Command：未发现")
        self._gatt_vars["ack"].set("ACK / Status：未发现")
        self._gatt_vars["device_info"].set("Device Info：未发现")
        self._gatt_vars["battery"].set("Battery Level：未发现/未订阅")
        self._gatt_vars["export"].set("Export Data：未发现")
        if "calibration" in self._gatt_vars:
            self._gatt_vars["calibration"].set("Calibration Service：未发现")
        self._gatt_vars["notify"].set("ACK Notify：未订阅")
        self._reset_battery_summary()
        self._reset_device_info_summary()
        self._set_text_widget(
            self.gatt_list_text,
            "等待连接设备后显示服务列表。\n\n当前要求：9ECA Control Service + Command + ACK Notify + Export Data Notify；可选：Battery Service / Battery Level Notify。",
        )
        self._gatt_export_text = "当前暂无 GATT 数据。"

    def _refresh_ota_panel(self) -> None:
        self.ota_state_var.set(f"OTA状态：{self.ota_manager.state.value}")
        self.ota_connection_var.set("DFU连接：已连接" if self.ota_manager.connected else "DFU连接：未连接")

        image = self.ota_manager.image_info
        if image is None:
            self.ota_file_var.set("固件文件：未选择")
            self.ota_size_var.set("文件大小：-")
            self.ota_sha_var.set("SHA256：-")
        else:
            self.ota_file_var.set(f"固件文件：{image.file_name}")
            self.ota_size_var.set(f"文件大小：{image.size_bytes} bytes")
            self.ota_sha_var.set(f"SHA256：{image.sha256}")

        preflight = self.ota_manager.preflight_check()
        if preflight.ok:
            self.ota_preflight_var.set("前置检查：固件已选择，可扫描 DFU")
        else:
            self.ota_preflight_var.set(f"前置检查：{'；'.join(preflight.issues)}")

        self.ota_protocol_var.set("协议状态：普通 BLE 进入 DFU + RTL8762D legacy DFU")
        self.ota_dfu_progress_var.set(f"DFU进度：{self.ota_manager.dfu_progress_summary}")

        if (
            not self.ota_manager.dfu_running
            and self.ota_manager.state in {OtaState.IDLE, OtaState.FILE_SELECTED, OtaState.CONNECTED, OtaState.SERVICE_CHECKED}
        ):
            lines = self.ota_manager.service_result.to_lines()
            self._set_text_widget(self.ota_service_text, "\n".join(lines))

        self._set_ota_controls_enabled(not self.ota_manager.dfu_running)

    def _set_ota_controls_enabled(self, enabled: bool) -> None:
        base_state = (
            "normal"
            if enabled
            and not getattr(self, "_calibration_active", False)
            and not getattr(self, "_offline_priority_active", False)
            and not getattr(self, "_feature_config_sync_active", False)
            and not getattr(self, "_ota_operation_active", False)
            else "disabled"
        )
        if hasattr(self, "ota_select_button"):
            self.ota_select_button.configure(state=base_state)
        if hasattr(self, "ota_clear_button"):
            self.ota_clear_button.configure(state=base_state)
        if hasattr(self, "ota_start_button"):
            start_allowed = bool(
                base_state == "normal"
                and getattr(self, "_business_ready", False)
                and getattr(self, "_last_device_state", None)
                == DEVICE_STATE_WAIT_START
                and not getattr(self, "_start_command_pending", False)
                and not getattr(self, "_offline_start_command_pending", False)
            )
            self.ota_start_button.configure(
                state="normal" if start_allowed else "disabled"
            )

    def _on_ota_log(self, message: str) -> None:
        if hasattr(self, "log_text"):
            self._log(f"[OTA] {message}")
        if hasattr(self, "ota_log_text"):
            self._append_ota_log(message)

    def _append_ota_log(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.ota_log_text.configure(state="normal")
        self.ota_log_text.insert(tk.END, f"[{ts}] {message}\n")
        self._trim_text(self.ota_log_text, 400)
        self.ota_log_text.see(tk.END)
        self.ota_log_text.configure(state="disabled")

    def _download_gatt_summary(self) -> None:
        self._save_text(self._gatt_export_text, "导出 GATT 服务与特征值", "gatt_services.txt")

    def _download_text_widget(self, widget: tk.Text, title: str, filename: str) -> None:
        prev = widget.cget("state")
        widget.configure(state="normal")
        content = widget.get("1.0", tk.END).strip()
        widget.configure(state=prev)
        self._save_text(content, title, filename)

    def _save_text(self, content: str, title: str, filename: str) -> None:
        if not content:
            messagebox.showinfo("提示", "当前内容为空，无需导出。")
            return
        path = filedialog.asksaveasfilename(
            title=title,
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")],
            initialfile=filename,
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(content)
            messagebox.showinfo("提示", f"已保存到：{path}")
        except OSError as exc:
            messagebox.showerror("错误", f"保存失败：{exc}")

    @staticmethod
    def _normalize_uuid(uuid: str) -> str:
        return uuid.strip().lower()

    @staticmethod
    def _trim_text(widget: tk.Text, keep_lines: int) -> None:
        line_count = int(float(widget.index("end-1c").split(".")[0]))
        if line_count > keep_lines:
            widget.delete("1.0", f"{line_count - keep_lines}.0")

    @staticmethod
    def _set_text_widget(widget: tk.Text, content: str) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, content)
        widget.configure(state="disabled")

    @staticmethod
    def _clear_text_widget(widget: tk.Text) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", tk.END)
        widget.configure(state="disabled")

    def _log(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, f"[{ts}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")

    def _log_diagnostic(self, message: str) -> None:
        if hasattr(self, "log_text"):
            self._log(message)

    def _on_close(self) -> None:
        if self._is_closing:
            return
        self._is_closing = True
        self.root.title("ZY100 BLE — 正在关闭连接并完成文件收尾…")
        self._close_error = ""

        def close_services() -> None:
            try:
                if not getattr(self, "_dfu_shutdown_done", False):
                    self.dfu_ota_client.shutdown()
                    self._dfu_shutdown_done = True
            except Exception as exc:
                self._close_error = str(exc)
            finally:
                if not self.ble_manager.shutdown():
                    self._close_error = "后台文件收尾尚未完成，连接已停止。请稍后再次关闭；本次不会标记为正常结束。"
                if not self._close_error:
                    # Runs on the close thread, not Tk. Let an admitted export
                    # finish instead of exiting with a truncated ZIP file.
                    self._online_list_executor.shutdown(wait=True, cancel_futures=False)

        self._close_thread = threading.Thread(target=close_services, name="ble-close", daemon=True)
        self._close_thread.start()
