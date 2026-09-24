from __future__ import annotations

import asyncio
import re
import sys
import threading
import time
import traceback
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Any, Callable, Iterable, NamedTuple

from .connection_recovery import ConnectionFailure, ConnectionRecovery

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice

from .calibration_protocol import (
    CAL_ACTIVE_STATUSES,
    CAL_FAILURE_STATUSES,
    CALIBRATION_INFO_UUID,
    CALIBRATION_RX_UUID,
    CALIBRATION_SERVICE_UUID,
    CALIBRATION_STATUS_UUID,
    CALIBRATION_TX_UUID,
    CalibrationReceiver,
    build_calibration_ack,
    build_calibration_diagnostics_request,
    build_calibration_info_confirmation,
    build_calibration_cached_record_confirmation,
    build_calibration_request,
    build_start_mag_calibration,
    parse_calibration_info,
    parse_calibration_status,
)
from .calibration_store import CalibrationStore
from .ci_state import HostCiState, HostCiStateMachine
from .connection_coordinator import (
    BootstrapV1Strategy,
    BootstrapV2Strategy,
    BootstrapV3Strategy,
    BootstrapV4Strategy,
    ConnectionCoordinator,
    ConnectionStage,
    UnsupportedProtocolError,
)
from .feature_config import FeatureConfig, FeatureConfigStore
from .gatt_operation_queue import GattPriority
from .gatt_trust_store import GattTrustStore
from .export_pipeline import ExportNotificationPipeline, QueuedExportNotification
from .export_speed import (
    BleSpeedSnapshot,
    WindowsCentralProfile,
    WindowsCentralProfileManager,
    WinRtExportSpeedAdapter,
)
from .host_selftest import (
    get_bleak_services,
    run_host_ble_self_check_offline,
    run_host_ble_self_check_online,
    self_check_enabled,
)
from .link_trace import HostBleLinkTrace
from .online_stress import CAP_STRESS, RECORD_STRESS
from .online_stress_controller import OnlineStressController
from .online_store import OnlineSessionStore
from .online_storage_worker import OnlineStorageWorker
from .offline_v2_protocol import (
    OFFLINE_MANIFEST_SUPPORTED_VERSIONS,
    is_offline_v2_frame_type,
    parse_offline_outer_frame,
)
from .offline_v2_store import OfflineV2SessionStore
from .offline_v2_sync import OfflineV2SyncController, OfflineV2SyncState
from .online_diagnostics import (
    ONLINE_DIAG_HEARTBEAT_SECONDS,
    OnlineHostDiagnostics,
)
from .session_store import SessionStore, now_iso
from .zy100_protocol import (
    CMD_CLEAR_FLASH,
    CMD_CONNECTION_USER_SYNC,
    CMD_FEATURE_CONFIG_SYNC,
    CMD_TIMEOUT_ACTION_ACK,
    CMD_TIMEOUT_ACTION_NOTIFY,
    TIMEOUT_ACTION_OFFLINE_IDLE,
    TIMEOUT_ACTION_STANDBY,
    STATE_DETAIL_LOW_BATTERY_REASON,
    parse_timeout_action,
    build_timeout_action_ack,
    CMD_CONNECTION_STATE_NOTIFY,
    CMD_GET_CONNECTION_STATE,
    CMD_GET_LINK_STATE,
    CMD_ENTER_SHIPPING,
    CMD_HOST_CI_MODE_ENABLE,
    CMD_HOST_PROFILE_RESULT,
    CMD_LINK_STATE_NOTIFY,
    CMD_EXPORT_CONFIRM,
    CMD_ONLINE_RECORD_ACK,
    CMD_ONLINE_STREAM_READY,
    CMD_OFFLINE_CHUNK_ACK,
    CMD_OFFLINE_FINAL_CONFIRM,
    CMD_OFFLINE_FOREIGN_PURGE,
    CMD_OFFLINE_CAPTURE_START,
    CMD_OFFLINE_CAPTURE_STOP,
    CMD_OFFLINE_RECLAIM_STATUS,
    CMD_OFFLINE_SESSION_BEGIN,
    CMD_OFFLINE_SESSION_LIST,
    CMD_OFFLINE_SESSION_RESUME,
    CMD_OTA_COMMIT,
    CMD_OTA_LINK_INTENT,
    CMD_OTA_PREPARE,
    CMD_PAUSE_CAPTURE,
    CMD_PING,
    CMD_START_CAPTURE,
    CMD_STATE_NOTIFY,
    CMD_TIME_SYNC,
    build_enter_shipping_command,
    status_text,
    CONNECTION_READY_FEATURE_CONFIG,
    HOST_PROFILE_BALANCED,
    HOST_PROFILE_MASK_ALL,
    HOST_CI_ENABLE_DETAIL_CI_SETTLING,
    HOST_CI_ENABLE_DETAIL_DEVICE_NOT_IDLE,
    HOST_PROFILE_POWER,
    HOST_PROFILE_THROUGHPUT,
    HOST_RESULT_ACTUAL_STABLE,
    HOST_RESULT_REQUEST_ACCEPTED,
    HOST_RESULT_REQUEST_FAILED,
    HOST_CI_HANDOFF_ACQUIRE_WINDOWS_CENTRAL,
    HOST_CI_HANDOFF_RELEASE_FOR_EXACT_STANDBY,
    LINK_STATE_INTENT,
    LINK_STATE_WAIT_HOST,
    LINK_STATE_APPLIED,
    LINK_STATE_FAILED,
    LINK_STATE_TIMEOUT,
    LINK_STATE_HANDOFF_RELEASE_REQUEST,
    LINK_STATE_PERIPHERAL_EXACT_PENDING,
    LINK_STATE_HOST_ACQUIRE_REQUEST,
    LINK_STATE_HANDOFF_ABORTED,
    DEVICE_STATE_BLE_EXPORT_WAIT_CONFIRM,
    DEVICE_STATE_CAPTURING,
    DEVICE_STATE_CLEARING_FLASH,
    DEVICE_STATE_OFFLINE_CAPTURING,
    DEVICE_STATE_OFFLINE_FINALIZING,
    DEVICE_STATE_OFFLINE_INTENT_READY,
    DEVICE_STATE_OFFLINE_SESSION_READY,
    DEVICE_STATE_OFFLINE_RECLAIMING,
    DEVICE_STATE_ONLINE_END_WAIT_ACK,
    DEVICE_STATE_ONLINE_STREAMING,
    DEVICE_STATE_STOPPING,
    DEVICE_STATE_WAIT_START,
    ZY100_BLE_OFFLINE_DETAIL_START_CANCELLED,
    DEVICE_NAME_PREFIX,
    FEUF_FRAME_DATA,
    EXPORT_START,
    ONLINE_CAPABILITY_CURRENT,
    ONLINE_FRAME_ABORT,
    ONLINE_FRAME_END,
    ONLINE_FRAME_RECORD,
    ONLINE_FRAME_START,
    ONLINE_RECORD_END,
    ONLINE_RECORD_EVENT,
    ONLINE_RECORD_RAW,
    ONLINE_RECORD_SUMMARY,
    ONLINE_SUMMARY_ACK_BATCH_DEFAULT,
    OTA_DFU_ENTRY_ACK,
    OTA_DFU_ENTRY_ACK_TIMEOUT_SECONDS,
    OTA_DFU_ENTRY_DISCONNECT_DELAY_SECONDS,
    OTA_DFU_ENTRY_REJECT,
    OTA_DFU_ENTRY_REQUEST,
    PREFERRED_DEVICE_NAME,
    START_DETAIL_ONLINE_CI9_NOT_READY,
    START_DETAIL_ONLINE_CI_LLCP_STUCK,
    ZY100_ACK_UUID,
    ZY100_COMMAND_UUID,
    ZY100_CONTROL_SERVICE_UUID,
    ZY100_DEVICE_INFO_UUID,
    ZY100_DEVICE_CHANNEL_UUID,
    ZY100_EXPORT_DATA_UUID,
    ZY100_FACTORY_SERVICE_UUID,
    ZY100_MAX_PAIRED_CENTRALS,
    ZY100_MAX_SIMULTANEOUS_LINKS,
    ZY100_MFG_INFO_UUID,
    ZY100Ack,
    ExportReceiver,
    ExportReceiverEvent,
    OnlineCompletedRecord,
    OnlineReceiverEvent,
    OnlineStreamReceiver,
    build_export_confirm_command,
    build_connection_user_sync_command,
    build_feature_config_sync_command,
    build_get_link_state_command,
    build_get_connection_state_command,
    build_host_ci_mode_enable_command,
    build_host_ci_handoff_command,
    build_host_profile_result_command,
    build_online_record_ack_command,
    build_online_stream_ready_command,
    build_ota_commit_command,
    build_ota_link_intent_command,
    build_ota_prepare_command,
    build_time_sync_command,
    build_zy100_command,
    command_name,
    hex_bytes,
    is_online_export_frame_type,
    is_time_sync_ack_ok,
    online_record_type_name,
    parse_feuf_stream,
    parse_export_frame,
    parse_zy100_ack,
    parse_connection_state_snapshot,
    CONNECTION_DEFERRED_OFFLINE_CAPTURE,
    CONNECTION_READY_OFFLINE_CAPTURE_OBSERVER,
)


LogCallback = Callable[[str], None]


# Production 10448 introduced BLE offline START/STOP. The pending-start
# cleanup/cancel fix first shipped in 10488; earlier builds can remain in
# OFFLINE_INTENT_READY forever after an accepted 0x2B request.
LEGACY_OFFLINE_INTENT_BUG_MIN_CODE = 10448
LEGACY_OFFLINE_INTENT_FIX_CODE = 10488
LEGACY_OFFLINE_INTENT_GRACE_SECONDS = 3.0
LEGACY_OFFLINE_INTENT_RECONNECT_DELAY_SECONDS = 1.0
EventCallback = Callable[[dict[str, Any]], None]


@dataclass(slots=True)
class _ClearFlashTransaction:
    seq: int
    connection_token: int
    client: Any
    ack_received: bool = False
    terminal_started: bool = False


@dataclass(frozen=True, slots=True)
class OtaDfuEntryTarget:
    write_uuid: str
    notify_uuid: str
    label: str
    write_handle: int | None = None
    notify_handle: int | None = None


_BLUETOOTH_ADDRESS_RE = re.compile(r"(?:Dev_)?([0-9A-Fa-f]{12})(?=#)")

WINDOWS_GATT_CACHE_HINT = (
    "未发现 ZY100 Control Service，可能是 Windows BLE GATT 缓存，请关闭蓝牙适配器重开、"
    "删除设备缓存或重启上位机后重新扫描。"
)

LEGACY_SIMPLE_SERVICE_UUID = "0000a00a-0000-1000-8000-00805f9b34fb"
REALTEK_OTA_SERVICE_UUID = "0000d0ff-3c17-d293-8e48-14fe2e4da212"
LEGACY_SIMPLE_CHARACTERISTIC_UUIDS = {
    "v1_read": "0000b001-0000-1000-8000-00805f9b34fb",
    "v2_write": "0000b002-0000-1000-8000-00805f9b34fb",
    "v3_notify": "0000b003-0000-1000-8000-00805f9b34fb",
    "v4_indicate": "0000b004-0000-1000-8000-00805f9b34fb",
    "v5_write_notify": "0000b005-0000-1000-8000-00805f9b34fb",
}
LEGACY_SIMPLE_PROFILE_HINT = (
    f"检测到旧版 Simple BLE Service {LEGACY_SIMPLE_SERVICE_UUID}（B001/B002/B003/B004/B005），"
    "但当前工具需要 ZY100 9ECA Control Service。请升级设备固件到 9ECA 协议版本，"
    "或使用匹配 A00A/B00x 协议的旧版工具；若刚升级固件后仍显示 A00A，请再处理 Windows BLE GATT 缓存。"
)

STATUS_OK = 0x00
STATUS_NOT_READY = 0x05
STATUS_BUSY = 0x04
STATUS_UNSUPPORTED_CMD = 0x03
EXEC_MODE_REAL_ACTION = 0x02
EXEC_MODE_DRY_RUN_NO_ACTION = 0x01
EXEC_MODE_ACCEPTED_ASYNC = 0x03
EXEC_MODE_ASYNC_DONE = 0x04
ONLINE_READY_ACK_TIMEOUT_SECONDS = 1.0
ONLINE_READY_RETRY_DELAY_SECONDS = 5.0
BOOTSTRAP_READY_OFFLINE_GATE = 1 << 6
BOOTSTRAP_READY_OFFLINE_SESSION_ATTENTION = 1 << 12
START_FINAL_ACK_TIMEOUT_SECONDS = 75.0
OFFLINE_START_FINAL_ACK_TIMEOUT_SECONDS = 10.0
HOST_CI_ENABLE_SETTLE_SAMPLE_INTERVAL_SECONDS = 0.25
HOST_CI_ENABLE_SETTLE_SAMPLE_COUNT = 2
HOST_CI_ENABLE_SETTLE_TIMEOUT_SECONDS = 5.0
BOOTSTRAP_QUERY_TIMEOUT_SECONDS = 3.0
RECOVERY_ONLY_IDLE_TIMEOUT_SECONDS = 120.0
CI_OBSERVER_POLL_SECONDS = 1.0
START_CI48_RESTORE_TIMEOUT_SECONDS = 25.0
START_CI48_STABLE_SAMPLE_COUNT = 2
START_OP_IDLE = "IDLE"
START_OP_PREPARING = "PREPARING"
START_OP_WAIT_FINAL = "WAIT_FINAL"
START_OP_RECOVERING = "RECOVERING"
START_OP_TERMINAL = "TERMINAL"
START_OP_BUSY_STATES = {
    START_OP_PREPARING,
    START_OP_WAIT_FINAL,
    START_OP_RECOVERING,
    START_OP_TERMINAL,
}
DEVICE_STATE_ERROR = 0x07
DEVICE_STATE_EXPORT_READY = 0x04
DEVICE_STATE_BLE_EXPORTING = 0x09
EXPORT_SPEED_TRIGGER_STATES = {
    DEVICE_STATE_EXPORT_READY,
    DEVICE_STATE_BLE_EXPORTING,
    DEVICE_STATE_BLE_EXPORT_WAIT_CONFIRM,
}

ONLINE_NORMAL_DEVICE_STATE_STATUS = {
    DEVICE_STATE_ONLINE_STREAMING: "streaming",
    DEVICE_STATE_STOPPING: "stopping",
    DEVICE_STATE_ONLINE_END_WAIT_ACK: "end_wait_ack",
    DEVICE_STATE_CLEARING_FLASH: "clearing_flash",
    DEVICE_STATE_WAIT_START: "wait_start",
}

ONLINE_SUMMARY_FULL_ACK_DELAY_S = 0.08
ONLINE_SUMMARY_PARTIAL_RETRY_S = 1.0

CLEAR_STATUS_NEEDS_DEVICE_CLEAR = "needs_device_clear"
CLEAR_STATUS_REQUESTED = "clear_requested"
CLEAR_STATUS_OK = "clear_ok"
CLEAR_STATUS_FAILED = "clear_failed"

RECLAIM_STATUS_RECLAIMING = "reclaiming"
RECLAIM_STATUS_OK = "reclaim_ok"
RECLAIM_STATUS_UNKNOWN = "reclaim_unknown"
RECLAIM_STATUS_FAILED = "reclaim_failed"

GATT_REDISCOVERY_RETRY_DELAYS_S = (0.8, 1.5)
GATT_HARD_RECONNECT_DELAY_S = 2.0
GATT_HARD_RECONNECT_SCAN_S = 4.0
CI_RECOVERY_SCAN_ATTEMPTS = 3
CI_RECOVERY_SCAN_RETRY_DELAY_SECONDS = 1.0
GATT_DISCOVERY_ALL_SERVICES = "all_services"
GATT_DISCOVERY_ZY100_CONTROL = "zy100_control_uuid"
OTA_RECONNECT_SCAN_ATTEMPTS = 6
OTA_RECONNECT_SCAN_SECONDS = 4.0
OTA_RECONNECT_RETRY_DELAY_SECONDS = 2.0
# On Windows, the first cached-service connection can consume the device's only
# BLE link even after the Bleak client disconnects. Waiting to observe the same
# signature twice then deadlocks on scanning because the device no longer
# advertises. This fingerprint is strict and is only acted on in OTA recovery.
OTA_RECONNECT_CACHE_SIGNATURE_THRESHOLD = 1
OTA_RECONNECT_UNPAIR_SETTLE_SECONDS = 2.0
OTA_PREPARE_ACK_TIMEOUT_SECONDS = 2.0
OTA_LINK_INTENT_ACK_TIMEOUT_SECONDS = 2.0
OTA_LINK_INTENT_WINDOW_MS = 12000
OTA_HIGH_SPEED_OPERATION_TAG = 0x004F5441
OTA_ACTIVE_LINK_TIMEOUT_SECONDS = 10.0
OTA_ACTIVE_LINK_POLL_SECONDS = 0.05
OTA_NOTIFY_SETTLE_EVENT_COUNT = 2
OTA_NOTIFY_SETTLE_MIN_SECONDS = 0.12
OTA_NOTIFY_SETTLE_MAX_SECONDS = 0.50
OTA_LEGACY_RETRY_WAIT_SECONDS = 0.60
CAL_SECURITY_RETRY_DELAYS_S = (0.25, 0.5, 1.0, 2.0)
CAL_RECORD_SYNC_ATTEMPTS = 2
CAL_RECORD_SYNC_TIMEOUT_SECONDS = 3.0
BATTERY_SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"
PAIRING_FAILURE_MARKERS = (
    "could not pair",
    "failed to pair",
    "pairing failed",
    "pair failed",
    "devicepairingresultstatus",
)


class _FeatureConfigTransaction(NamedTuple):
    connection_token: int
    transaction_id: int
    client: BleakClient | None
    user_id: int | None
    config: FeatureConfig
    ack_queue: asyncio.Queue[ZY100Ack]


class UserSyncResult(Enum):
    FAILED = "failed"
    WAITING_CAPTURE = "waiting_capture"
    COMPLETE = "complete"


class BleManager:
    """BLE communication layer for the current ZY100 9ECA control/export protocol."""

    def __init__(self, event_callback: EventCallback, log_callback: LogCallback) -> None:
        self._event_callback = event_callback
        self._log_callback = log_callback

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_event_loop,
            name="BleEventLoopThread",
            daemon=True,
        )
        self._thread.start()

        self._client: BleakClient | None = None
        self._devices: dict[str, BLEDevice] = {}
        self._connected_address: str | None = None
        self._last_services_payload: list[dict[str, Any]] = []
        self._last_service_objects: list[Any] = []

        self._zy100_control_found = False
        self._command_found = False
        self._ack_found = False
        self._export_found = False
        self._command_uuid: str | None = None
        self._ack_uuid: str | None = None
        self._export_uuid: str | None = None
        self._ack_subscribed = False
        self._export_subscribed = False
        self._battery_level_char: BleakGATTCharacteristic | None = None
        self._battery_level_uuid: str | None = None
        self._battery_subscribed = False
        self._battery_percent: int | None = None
        self._calibration_service_found = False
        self._calibration_info_uuid: str | None = None
        self._calibration_tx_uuid: str | None = None
        self._calibration_rx_uuid: str | None = None
        self._calibration_status_uuid: str | None = None
        self._calibration_tx_subscribed = False
        self._calibration_status_subscribed = False
        self._calibration_command_pending = False
        self._calibration_active = False
        self._calibration_transaction_id = 0
        self._calibration_receiver = CalibrationReceiver()
        self._calibration_store = CalibrationStore()
        self._latest_calibration_record: Any = None
        self._calibration_gate_state = "unknown"
        self._calibration_summary: dict[str, Any] = {}
        self._calibration_info_confirm_transaction_id: int | None = None
        self._calibration_info_confirm_future: asyncio.Future[Any] | None = None
        self._calibration_cache_confirm_transaction_id: int | None = None
        self._calibration_cache_confirm_future: asyncio.Future[Any] | None = None
        self._calibration_record_sync_future: asyncio.Future[bool] | None = None
        self._calibration_record_ack_transaction_id: int | None = None
        self._calibration_record_request_active = False
        self._calibration_record_expected_summary: dict[str, Any] = {}
        self._calibration_diag_request_transaction_id: int | None = None
        self._calibration_diag_request_pending = False
        self._calibration_diag_auto_pending = False
        self._calibration_diag_retry_used = False
        self._calibration_diag_generation = 0
        self._calibration_diag_record_crc32 = 0
        self._notify_ready = False
        self._rtc_sync_ok = False
        self._active_user_id: int | None = None
        self._active_training_id: int | None = None
        self._last_rtc_sync_ms: int | None = None
        self._time_sync_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._time_sync_seq: int | None = None
        self._time_sync_user_id: int | None = None
        self._time_sync_training_id: int | None = None
        self._connection_user_sync_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._connection_user_sync_seq: int | None = None
        self._connection_user_context_supported = False
        self._connection_user_synced = False
        self._user_sync_result = UserSyncResult.FAILED
        self._capture_identity_attempted = False
        self._connection_requested_user_id = None
        self._connection_user_id: int | None = None
        self._connection_user_session_count = 0
        self._foreign_session_count = 0
        self._feature_config_store = FeatureConfigStore()
        self._feature_config_snapshot = FeatureConfig()
        self._feature_config_user_id: int | None = None
        self._feature_config_supported = False
        self._feature_config_synced = False
        self._feature_config_pending = False
        self._feature_config_generation = 0
        self._feature_config_crc32 = 0
        self._feature_config_seq: int | None = None
        self._feature_config_ack_queue: asyncio.Queue[ZY100Ack] = asyncio.Queue()
        self._feature_config_transaction_id = 0
        self._feature_config_confirmed: dict[str, Any] | None = None
        self._feature_config_sync_lock = asyncio.Lock()
        self._feature_config_retry_task: asyncio.Task[None] | None = None
        self._foreign_prompt_shown = False
        self._foreign_purge_device_active = False
        self._foreign_prompt_future: asyncio.Future[bool] | None = None
        self._foreign_purge_ack_queue: asyncio.Queue[ZY100Ack] = asyncio.Queue()
        self._foreign_purge_expected_user_id: int | None = None
        self._next_seq = 1
        self._control_command_lock = asyncio.Lock()
        self._connection_coordinator = ConnectionCoordinator(self._emit, platform="windows")
        self._link_trace = HostBleLinkTrace.from_environment()
        self._gatt_trust_store = GattTrustStore()
        self._gatt_fast_path_active = False
        self._connection_token = 0
        self._timeout_action: dict[str, int] | None = None
        self._timeout_action_client: BleakClient | None = None
        self._device_info_text = ""
        self._device_channel_supported = False
        self._device_channel_received = False
        self._device_channel_value: str | None = None
        self._bootstrap_query_seq: int | None = None
        self._bootstrap_query_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._bootstrap_snapshot_future: asyncio.Future[Any] | None = None
        self._bootstrap_generation = 0
        self._bootstrap_revision: int | None = None
        self._bootstrap_last_status: int | None = None
        self._bootstrap_ready_bits = 0
        self._bootstrap_device_state: int | None = None
        self._bootstrap_suggested_action: int | None = None
        self._bootstrap_stage: int | None = None
        self._bootstrap_reason: int | None = None
        self._bootstrap_snapshot_event = asyncio.Event()
        self._offline_observer_active = False
        self._offline_observer_task: asyncio.Task[None] | None = None
        self._offline_runtime_last_state: int | None = None
        self._connection_business_ready = False
        self._recovery_only = False
        self._recovery_idle_task: asyncio.Task[None] | None = None
        self._session_store = SessionStore()
        self._export_receiver = ExportReceiver()
        self._online_receiver = OnlineStreamReceiver()
        self.stress = OnlineStressController(self)
        self._online_store = OnlineSessionStore()
        self._online_storage = OnlineStorageWorker()
        self._online_io_epoch = 0
        self._online_abort_tasks: dict[OnlineSessionStore, asyncio.Task] = {}
        self._online_saved_ns = 0
        self._offline_v2_store = OfflineV2SessionStore()
        self._offline_v2_commands_ready = False
        self._central_balanced_applied_event = asyncio.Event()
        self._offline_priority_blocked = False
        self._offline_priority_phase = "idle"
        self._offline_v2_host_ci_deferred = False
        self._offline_v2_host_ci_defer_detail = 0
        self._offline_v2_host_ci_resume_task: asyncio.Task[None] | None = None
        self._legacy_offline_intent_reconnect_used = False
        self._offline_v2_sync = OfflineV2SyncController(
            store=self._offline_v2_store,
            command_sender=self._send_offline_v2_command,
            event_callback=self._on_offline_v2_event,
            log_callback=self._log,
            wait_for_balanced=self._wait_for_offline_balanced_restore,
        )
        self._export_pipeline = ExportNotificationPipeline()
        self._export_parse_task: asyncio.Task[None] | None = None
        self._export_speed = WinRtExportSpeedAdapter(self._log)
        self._central_profiles = WindowsCentralProfileManager(self._log)
        self._central_mode_enabled = False
        self._central_hybrid_enabled = False
        self._central_session_state = "TERMINAL"
        self._central_session_id = 0
        self._central_generation = 0
        self._central_transition = 0
        self._central_enable_seq: int | None = None
        self._central_enable_future: asyncio.Future[ZY100Ack] | None = None
        self._central_applied_future: asyncio.Future[bool] | None = None
        self._central_link_intent_event = asyncio.Event()
        self._central_early_link_ack: ZY100Ack | None = None
        self._pending_actual_stable: tuple[tuple[int, int, int, int], WindowsCentralProfile, int] | None = None
        self._central_enable_reconnect_used = False
        self._central_enable_reconnect_requested = False
        self._central_profile_task: asyncio.Task[None] | None = None
        self._central_profile_key: tuple[int, int, int, int] | None = None
        self._central_heartbeat_task: asyncio.Task[None] | None = None
        self._central_handoff_task: asyncio.Task[None] | None = None
        self._central_recovery_used = False
        self._ci_state = HostCiStateMachine(self._log)
        self._ci_observer_task: asyncio.Task[None] | None = None
        self._last_transfer_progress_emit_ms = 0
        self._current_device_name = ""
        self._post_ready_info_task: asyncio.Task[None] | None = None
        self._connection_started_monotonic = 0.0
        self._gatt_ready_monotonic = 0.0
        self._failed_export_timeout_task: asyncio.Task[None] | None = None
        self._export_failure_epoch = 0
        self._online_metadata_retry_task: asyncio.Task[None] | None = None
        self._online_fault_task: asyncio.Task[None] | None = None
        self._online_fault_reason = ""
        self._online_fault_can_drain = False
        self._online_fault_terminal = asyncio.Event()
        self._pending_confirm_session_key: str | None = None
        self._pending_confirm_export_id: int | None = None
        self._pending_confirm_seq: int | None = None
        self._pending_confirm_ack_ok = False
        self._pending_confirm_final_ack_ok = False
        self._clear_flash_session_keys: set[str] = set()
        self._clear_flash_offline_v2_session_keys: set[str] = set()
        self._clear_flash_seq: int | None = None
        self._clear_flash_transaction: _ClearFlashTransaction | None = None
        self._clear_flash_active = False
        self._clear_flash_device_accepted = False
        self._clear_flash_accept_task: asyncio.Task[None] | None = None
        self._clear_flash_accept_ok = False
        self._clear_flash_result_future: asyncio.Future[bool] | None = None
        self._shipping_operation_active = False
        self._shipping_seq: int | None = None
        self._shipping_accepted = False
        self._export_commands_blocked = False
        self._pending_command_active = False
        self._last_device_state: int | None = None
        self._online_ready_sent = False
        self._online_ready_ack_ok = False
        self._online_start_seq: int | None = None
        self._online_start_final_timeout_task: asyncio.Task[None] | None = None
        self._offline_start_seq: int | None = None
        self._offline_start_final_timeout_task: asyncio.Task[None] | None = None
        self._online_start_ci_recovery_used = False
        self._online_start_ci_recovery_task: asyncio.Task[None] | None = None
        self._online_start_operation_counter = 0
        self._online_start_operation_id = 0
        self._online_start_attempt = 0
        self._online_start_operation_state = START_OP_IDLE
        self._online_start_recovery_preserve = False
        self._online_start_ci48_restore_task: asyncio.Task[None] | None = None
        self._online_ready_seq: int | None = None
        self._online_ready_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._online_ready_retry_task: asyncio.Task[None] | None = None
        self._online_active = False
        self._online_end_finalizing = None
        self._online_status = "idle"
        self._online_session_id: int | None = None
        self._online_speed_skip_until = 0.0
        self._online_summary_ack_batch = ONLINE_SUMMARY_ACK_BATCH_DEFAULT
        self._online_summary_pending: list[OnlineCompletedRecord] = []
        self._online_summary_ack_task: asyncio.Task[None] | None = None
        self._online_summary_partial_task: asyncio.Task[None] | None = None
        self._online_started_perf = 0.0
        self._online_stats: dict[str, Any] = self._new_online_stats()
        self._online_diag = OnlineHostDiagnostics()
        self._online_diag_task: asyncio.Task[None] | None = None
        self._host_self_check_enabled = self_check_enabled()
        self._host_self_check_offline_future: Future | None = None
        self._host_self_check_online_task: asyncio.Task[None] | None = None
        self._host_ble_capability: dict[str, Any] = {}
        self._expected_disconnect_client_ids: set[int] = set()
        self._connection_recovery = ConnectionRecovery(self)
        self._gatt_retry_delays_s = GATT_REDISCOVERY_RETRY_DELAYS_S
        self._gatt_hard_reconnect_delay_s = GATT_HARD_RECONNECT_DELAY_S
        self._gatt_hard_reconnect_scan_s = GATT_HARD_RECONNECT_SCAN_S
        self._ci_recovery_scan_attempts = CI_RECOVERY_SCAN_ATTEMPTS
        self._ci_recovery_scan_retry_delay_s = CI_RECOVERY_SCAN_RETRY_DELAY_SECONDS
        self._ota_dfu_entry_ack_future: asyncio.Future[dict[str, Any]] | None = None
        self._ota_link_intent_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._ota_link_intent_seq: int | None = None
        self._ota_prepare_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._ota_prepare_seq: int | None = None
        self._ota_commit_ack_future: asyncio.Future[ZY100Ack] | None = None
        self._ota_commit_seq: int | None = None
        self._ota_commit_connection: tuple[object, int] | None = None
        self._ota_commit_accepted_connection: tuple[object, int] | None = None
        self._ota_high_speed_counter = 0
        self._ota_high_speed_key: tuple[int, int, int, int] | None = None
        self._ota_entry_operation_active = False
        self._ota_link_intent_ack_timeout_s = OTA_LINK_INTENT_ACK_TIMEOUT_SECONDS
        self._ota_prepare_ack_timeout_s = OTA_PREPARE_ACK_TIMEOUT_SECONDS
        self._ota_commit_ack_timeout_s = OTA_DFU_ENTRY_ACK_TIMEOUT_SECONDS
        self._ota_active_link_timeout_s = OTA_ACTIVE_LINK_TIMEOUT_SECONDS
        self._ota_notify_settle_min_s = OTA_NOTIFY_SETTLE_MIN_SECONDS
        self._ota_notify_settle_max_s = OTA_NOTIFY_SETTLE_MAX_SECONDS
        self._ota_legacy_retry_wait_s = OTA_LEGACY_RETRY_WAIT_SECONDS
        self._ota_dfu_entry_ack_timeout_s = OTA_DFU_ENTRY_ACK_TIMEOUT_SECONDS
        self._ota_dfu_entry_disconnect_delay_s = OTA_DFU_ENTRY_DISCONNECT_DELAY_SECONDS
        self._ota_dfu_entry_notify_uuid: str | None = None
        self._ota_dfu_entry_notify_subscribed = False
        self._ota_reconnect_scan_attempts = OTA_RECONNECT_SCAN_ATTEMPTS
        self._ota_reconnect_scan_s = OTA_RECONNECT_SCAN_SECONDS
        self._ota_reconnect_retry_delay_s = OTA_RECONNECT_RETRY_DELAY_SECONDS
        self._ota_reconnect_cache_signature_threshold = OTA_RECONNECT_CACHE_SIGNATURE_THRESHOLD
        self._ota_reconnect_unpair_settle_s = OTA_RECONNECT_UNPAIR_SETTLE_SECONDS
        self._ota_reconnect_active = False

        if self._host_self_check_enabled:
            self._host_self_check_offline_future = self._submit(self._run_host_self_check_offline())

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _submit(self, coro: Any) -> Future:
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def _log(self, message: str) -> None:
        started_ns = time.perf_counter_ns()
        self._log_callback(message)
        online_diag = getattr(self, "_online_diag", None)
        if online_diag is not None and not message.startswith("[ONLINE_HOST_"):
            online_diag.note_log_callback(
                (time.perf_counter_ns() - started_ns) / 1_000_000.0
            )

    def _log_connection_timing(self, stage: str) -> None:
        if self._connection_started_monotonic <= 0.0:
            return
        now = time.monotonic()
        total_ms = (now - self._connection_started_monotonic) * 1000.0
        protocol_ms = (
            (now - self._gatt_ready_monotonic) * 1000.0
            if self._gatt_ready_monotonic > 0.0
            else 0.0
        )
        self._log(
            f"[BLE_TIMING] stage={stage} total_ms={total_ms:.1f} "
            f"protocol_ms={protocol_ms:.1f}"
        )

    def _emit(self, event: dict[str, Any]) -> None:
        started_ns = time.perf_counter_ns()
        self._event_callback(event)
        online_diag = getattr(self, "_online_diag", None)
        if online_diag is not None and event.get("type") == "online_stream_status":
            online_diag.note_event_callback(
                (time.perf_counter_ns() - started_ns) / 1_000_000.0
            )

    def note_online_ui_dispatch(self, delay_ms: float) -> None:
        if self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(
            self._online_diag.note_ui_dispatch,
            max(0.0, float(delay_ms)),
        )

    def note_online_ui_work(self, elapsed_ms: float) -> None:
        if not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._online_diag.note_ui_work, elapsed_ms)

    @staticmethod
    def _normalize_uuid(uuid: str) -> str:
        return uuid.strip().lower()

    @staticmethod
    def _format_bluetooth_address(value: int) -> str:
        if not (0 <= value <= 0xFFFFFFFFFFFF):
            raise ValueError(f"Bluetooth address out of range: {value!r}")
        return ":".join(f"{byte:02X}" for byte in value.to_bytes(6, byteorder="big"))

    @staticmethod
    def _parse_bluetooth_address(address: str) -> int | None:
        compact = address.strip().replace(":", "").replace("-", "")
        if len(compact) != 12:
            return None
        try:
            return int(compact, 16)
        except ValueError:
            return None

    @classmethod
    def _extract_bluetooth_address_from_device_id(cls, device_id: str) -> str | None:
        match = _BLUETOOTH_ADDRESS_RE.search(device_id)
        if match is None:
            return None
        try:
            return cls._format_bluetooth_address(int(match.group(1), 16))
        except ValueError:
            return None

    @staticmethod
    def _is_zp_device_name(name: str) -> bool:
        return name.startswith(DEVICE_NAME_PREFIX) and len(name) == 9

    def _make_winrt_cached_device(self, address: str, name: str = "") -> BLEDevice | None:
        bluetooth_address = self._parse_bluetooth_address(address)
        if bluetooth_address is None:
            return None

        try:
            from bleak.backends.winrt.scanner import RawAdvData
        except Exception as exc:  # pragma: no cover - non-Windows or missing WinRT
            self._log(f"[BLE_CACHE] WinRT cached device unavailable: {exc}")
            return None

        class _WinRtAddressArgs:
            def __init__(self, value: int) -> None:
                self.bluetooth_address = value

        normalized_address = self._format_bluetooth_address(bluetooth_address)
        return BLEDevice(
            normalized_address,
            name.strip() or normalized_address,
            RawAdvData(_WinRtAddressArgs(bluetooth_address), None),
        )

    async def _load_windows_cached_zp_devices(self) -> list[dict[str, Any]]:
        if sys.platform != "win32":
            return []

        try:
            from winrt.windows.devices.enumeration import DeviceInformation
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_CACHE] Windows device cache unavailable: {exc}")
            return []

        try:
            infos = await DeviceInformation.find_all_async()
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_CACHE] Windows device cache scan failed: {exc}")
            return []

        devices_by_address: dict[str, dict[str, Any]] = {}
        for info in infos:
            name = str(getattr(info, "name", "") or "").strip()
            if not self._is_zp_device_name(name):
                continue

            device_id = str(getattr(info, "id", "") or "")
            if "BTHLE" not in device_id.upper():
                continue

            address = self._extract_bluetooth_address_from_device_id(device_id)
            if address is None or address in devices_by_address:
                continue

            device = self._make_winrt_cached_device(address, name)
            if device is None:
                continue

            devices_by_address[address] = {
                "device": device,
                "name": name,
                "address": address,
                "device_id": device_id,
            }

        return sorted(
            devices_by_address.values(),
            key=lambda item: (
                item["name"] != PREFERRED_DEVICE_NAME,
                item["name"],
                item["address"],
            ),
        )

    def scan(self, timeout: float = 3.0) -> None:
        self._submit(self._scan(timeout))

    async def _scan(self, timeout: float, *, disconnect_first: bool = True) -> None:
        try:
            if disconnect_first:
                await self._connection_recovery.cancel()
            if disconnect_first and self._client is not None:
                if self._client.is_connected:
                    self._log("Existing BLE connection will be closed before scan")
                await self._safe_disconnect(emit=False)

            self._log(f"Start BLE scan ({timeout:.1f}s)")
            discovered = await BleakScanner.discover(timeout=timeout, return_adv=True)

            results: list[dict[str, Any]] = []
            self._devices.clear()

            for address, (device, adv) in discovered.items():
                name = (device.name or adv.local_name or "").strip()
                if not self._is_zp_device_name(name):
                    continue

                rssi = getattr(adv, "rssi", None)
                service_uuids = list(getattr(adv, "service_uuids", []) or [])
                self._devices[address] = device
                results.append(
                    {
                        "name": name,
                        "address": address,
                        "rssi": rssi,
                        "service_uuids": service_uuids,
                    }
                )
                self._log(f"[BLE_SCAN] name={name} address={address} rssi={rssi}")

            cached_count = 0
            for cached in await self._load_windows_cached_zp_devices():
                address = str(cached["address"])
                if address in self._devices:
                    continue

                self._devices[address] = cached["device"]
                results.append(
                    {
                        "name": cached["name"],
                        "address": address,
                        "rssi": "Windows缓存",
                        "service_uuids": [],
                        "source": "windows_cached",
                    }
                )
                cached_count += 1
                self._log(f"[BLE_CACHE] name={cached['name']} address={address} source=windows_cached")

            results.sort(
                key=lambda item: (
                    item["name"] != PREFERRED_DEVICE_NAME,
                    item["name"],
                    item["address"],
                )
            )
            self._log(
                f"Scan done, discovered {len(discovered)} device(s), matched {len(results)} ZP device(s), "
                f"cached {cached_count}"
            )
            self._emit({"type": "scan_result", "devices": results})
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"Scan failed: {exc}")
            self._emit({"type": "error", "message": f"Scan failed: {exc}"})

    def connect(self, address: str, user_id: int = 1, training_id: int = 1) -> None:
        if self._connection_recovery.task is not None:
            self._log("[BLE_RECOVERY] ignored duplicate connect")
            return
        self._central_enable_reconnect_used = False
        self._legacy_offline_intent_reconnect_used = False
        config, warning = self._feature_config_store.load(user_id)
        self._feature_config_confirmed = None
        self._feature_config_transaction_id += 1
        self._feature_config_snapshot = config
        self._feature_config_user_id = user_id
        self._feature_config_crc32 = config.crc32(user_id)
        self._feature_config_synced = False
        self._feature_config_pending = False
        self._emit_feature_config_status(
            "profile_locked",
            message=warning or "连接配置快照已锁定",
        )
        self._submit(self._connection_recovery.run(address, "", user_id, training_id, ota=False))

    def load_feature_config_profile(
        self, user_id: int
    ) -> tuple[FeatureConfig, str | None]:
        return self._feature_config_store.load(user_id)

    def feature_config_manual_gate(self, user_id: int) -> tuple[bool, str]:
        client = self._client
        if user_id <= 0 or self._connection_user_id != user_id:
            return False, "功能配置仅允许修改当前蓝牙连接的 user_id"
        if client is None or not client.is_connected:
            return False, "功能配置仅允许在普通蓝牙已连接时修改"
        if not self._feature_config_supported:
            return False, "当前固件不支持功能配置同步，页面保持只读"
        if not self._connection_business_ready:
            return False, "设备尚未进入 Business Ready，功能配置保持只读"
        if self._last_device_state != DEVICE_STATE_WAIT_START:
            return False, "设备不是 WAIT_START 空闲态，功能配置保持只读"
        if self._calibration_command_pending or self._calibration_active:
            return False, "地磁校准进行中，不能修改功能配置"
        if self._ota_entry_operation_active or self._ota_reconnect_active:
            return False, "OTA 进行中，不能修改功能配置"
        if self._offline_priority_blocked:
            return False, "离线采集或数据同步进行中，不能修改功能配置"
        if self._pending_command_active or self._feature_config_pending:
            return False, "当前存在待处理设备操作，不能修改功能配置"
        if self._online_active or self._online_start_operation_state != START_OP_IDLE:
            return False, "在线采集操作进行中，不能修改功能配置"
        return True, ""

    def feature_config_reapply_gate(self, user_id: int) -> tuple[bool, str]:
        allowed, reason = self.feature_config_manual_gate(user_id)
        if not allowed:
            return False, reason
        info = self._connection_coordinator.device_info
        if info is None or info.code is None or info.code < 10524:
            return False, "当前固件不支持硬件重新应用"
        confirmed = self._feature_config_confirmed
        config = self._feature_config_snapshot
        if (not self._feature_config_synced or not confirmed
                or confirmed.get("user_id") != user_id
                or confirmed.get("config") != config.to_dict()
                or confirmed.get("crc32") != config.crc32(user_id)):
            return False, "本次连接尚无有效的设备确认配置"
        if not config.auto_capture:
            return False, "设备已确认的 WOM 配置未开启"
        return True, ""

    def reapply_confirmed_feature_config(self, user_id: int) -> None:
        """One explicit application of the confirmed snapshot, ignoring the form/profile."""
        allowed, reason = self.feature_config_reapply_gate(user_id)
        if not allowed:
            raise ValueError(reason)
        config = self._feature_config_snapshot
        self._feature_config_pending = True
        transaction = self._new_feature_config_transaction(config)
        self._emit_feature_config_status(
            "saved_pending", transaction=transaction, candidate=config,
            message="已提交同值重新应用，WOM 配置保持开启；等待设备完成回执")
        self._submit(self._sync_feature_config(
            "ui_reapply_confirmed", config_override=config, transaction=transaction))

    def save_feature_config_profile(
        self,
        user_id: int,
        config: FeatureConfig,
        *,
        sync: bool = False,
    ) -> str:
        allowed, reason = self.feature_config_manual_gate(user_id)
        if not allowed:
            raise ValueError(reason)
        config.validate()
        path = self._feature_config_store.path_for_user(user_id)
        if sync:
            self._feature_config_pending = True
            self._feature_config_crc32 = config.crc32(user_id)
            transaction = self._new_feature_config_transaction(config)
            self._emit_feature_config_status(
                "saved_pending", transaction=transaction,
                message="已提交设备，确认成功后再保存本地档案", candidate=config)
            self._submit(self._sync_feature_config(
                "ui_save_and_sync", config_override=config, persist_on_success=True,
                transaction=transaction))
        else:
            path = self._feature_config_store.save(user_id, config)
            self._emit_feature_config_status(
                "saved_local", message="已保存本地用户配置，下次连接时同步", candidate=config)
        return str(path)

    def respond_foreign_offline_cleanup(self, confirm: bool) -> None:
        """Resolve the once-per-connection foreign-session administrator prompt."""
        self._loop.call_soon_threadsafe(
            self._resolve_foreign_offline_cleanup,
            bool(confirm),
        )

    def _resolve_foreign_offline_cleanup(self, confirm: bool) -> None:
        future = self._foreign_prompt_future
        if future is not None and not future.done():
            future.set_result(confirm)

    def reconnect_after_ota(
        self,
        address: str,
        name: str,
        user_id: int = 1,
        training_id: int = 1,
    ) -> None:
        """Recover the normal 9ECA connection after the DFU device reboots."""
        self._central_enable_reconnect_used = False
        self._submit(
            self._reconnect_after_ota(
                address,
                name=name,
                user_id=user_id,
                training_id=training_id,
            )
        )

    def get_connection_identity(self) -> dict[str, str]:
        return {
            "address": self._connected_address or "",
            "name": self._current_device_name,
        }

    def _emit_ota_reconnect_stage(self, stage: str, message: str, **extra: Any) -> None:
        self._log(f"[BLE_OTA_RECOVER] stage={stage} message={message}")
        self._emit(
            {
                "type": "ota_reconnect_stage",
                "stage": stage,
                "message": message,
                **extra,
            }
        )

    async def _reconnect_after_ota(
        self,
        address: str,
        *,
        name: str,
        user_id: int,
        training_id: int,
    ) -> None:
        self._ota_reconnect_active = True
        try:
            await self._connection_recovery.run(address, name, user_id, training_id, ota=True)
        finally:
            self._ota_reconnect_active = False

    def respond_pairing_repair(self, operation_id: int, approved: bool) -> None:
        self._loop.call_soon_threadsafe(self._connection_recovery.respond, operation_id, approved)

    def pairing_repair_is_current(self, operation_id: int) -> bool:
        return self._connection_recovery.current(operation_id)

    async def _is_windows_device_paired(self, address: str) -> bool:
        return (await self._resolve_windows_pairing_target(address)).state == "paired"

    async def _resolve_windows_pairing_target(self, address: str):
        from .windows_pairing import resolve_pairing_target
        return await resolve_pairing_target(address)

    def _is_standard_only_gatt_cache_signature(self) -> bool:
        service_uuids, char_uuids = self._collect_discovered_uuid_sets()
        if self._has_legacy_simple_profile() or ZY100_CONTROL_SERVICE_UUID in service_uuids:
            return False
        standard_services = {
            "00001800-0000-1000-8000-00805f9b34fb",
            "00001801-0000-1000-8000-00805f9b34fb",
            BATTERY_SERVICE_UUID,
        }
        zy100_characteristics = {
            ZY100_COMMAND_UUID,
            ZY100_ACK_UUID,
            ZY100_EXPORT_DATA_UUID,
        }
        return bool(service_uuids) and service_uuids.issubset(standard_services) and not (
            char_uuids & zy100_characteristics
        )

    async def _unpair_windows_device(self, address: str) -> bool:
        recovery = self._connection_recovery
        if (recovery.authorized_address != address.upper() or not recovery.authorized_endpoint_id
                or recovery.task is None or recovery.cancelled):
            self._log("[BLE_RECOVERY] unpair denied: no current target-specific approval")
            return False
        if sys.platform != "win32":
            self._log("[BLE_OTA_RECOVER] unpair unavailable on non-Windows host")
            return False
        try:
            from winrt.windows.devices.enumeration import DeviceUnpairingResultStatus
            operation_id = recovery.operation_id
            endpoint_id = recovery.authorized_endpoint_id
            target = await self._resolve_windows_pairing_target(address)
            if (not recovery.current(operation_id) or recovery.cancelled
                    or recovery.authorized_address != address.upper()
                    or recovery.authorized_endpoint_id != endpoint_id
                    or target.endpoint_id != endpoint_id or target.state != "paired"):
                self._log("[BLE_RECOVERY] unpair denied: endpoint or operation changed")
                return False
            # Consume the authorization before the asynchronous destructive call.
            recovery.authorized_endpoint_id = ""
            recovery.authorized_address = ""
            result = await target.info.pairing.unpair_async()
            self._log(f"[BLE_RECOVERY] unpair endpoint={endpoint_id} status={result.status}")
            return result.status in (DeviceUnpairingResultStatus.UNPAIRED,
                                     DeviceUnpairingResultStatus.ALREADY_UNPAIRED)
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_OTA_RECOVER] unpair failed reason={exc}")
            return False

    async def _connect(self, address: str, *, user_id: int, training_id: int) -> None:
        if not address:
            self._emit({"type": "error", "message": "No device selected"})
            return
        if user_id <= 0:
            self._emit({"type": "error", "message": "user_id must be 1..0xFFFFFFFF"})
            return

        self._connection_started_monotonic = time.monotonic()
        self._gatt_ready_monotonic = 0.0
        self._connection_token = self._connection_coordinator.begin(address)
        self._link_trace.begin_connection(self._connection_token, address)

        device = self._devices.get(address)
        if device is None:
            device = self._make_winrt_cached_device(address)
            if device is None:
                self._emit({"type": "error", "message": "Device list expired, please scan again"})
                return
            self._devices[address] = device
            self._log(f"[BLE_CACHE] connect by known address address={address}")

        try:
            name = device.name or ""
            # Every connect request performs a fresh, fully ordered bootstrap.
            # Reusing a bound client would bypass Device Info strategy selection
            # and could admit callbacks belonging to the previous token.
            if self._client is not None and self._client.is_connected:
                self._log("Existing BLE connection will be closed before fresh discovery")
                await self._safe_disconnect(emit=False)
                self._connection_token = self._connection_coordinator.begin(address)
                self._link_trace.begin_connection(self._connection_token, address)
            else:
                self._client = None
                self._connected_address = None
                self._last_services_payload = []
                self._last_service_objects = []

            self._clear_control_state()

            self._current_device_name = name
            client, control_status = await self._connect_with_gatt_retries(device, address, name)
            await self._initialize_connection(client, control_status, device, address, name,
                                              user_id=user_id, training_id=training_id)
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._link_trace.record_exception(
                "connect_failed",
                exc,
                gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
            )
            if self._gatt_fast_path_active:
                self._gatt_trust_store.invalidate(address)
                self._log("[BLE_GATT][FAST] invalidated_after_critical_failure=1")
            self._log(f"Connect failed: {exc}")
            self._connection_coordinator.transition(
                ConnectionStage.FAILED, reason=str(exc)
            )
            self._emit({"type": "error", "message": f"Connect failed: {exc}"})
            await self._safe_disconnect(emit=False)

    async def _initialize_connection(self, client: BleakClient, control_status: dict[str, Any],
                                     device: BLEDevice, address: str, name: str, *,
                                     user_id: int, training_id: int) -> None:
        self._gatt_ready_monotonic = time.monotonic()
        self._log_connection_timing("gatt_ready")
        if not self._has_required_gatt():
            self._emit_gatt_mismatch(control_status)
            await self._disconnect_after_gatt_mismatch(client)
            return
        strategy = await self._read_device_info(client)
        if self._gatt_fast_path_active and not self._trusted_device_info_matches(address):
            self._log("[BLE_GATT][FAST] device_info_changed=1 fallback=uncached")
            self._gatt_trust_store.invalidate(address)
            await self._disconnect_for_gatt_retry(client)
            client, control_status = await self._connect_with_gatt_retries(
                device, address, name
            )
            if not self._has_required_gatt():
                self._emit_gatt_mismatch(control_status)
                await self._disconnect_after_gatt_mismatch(client)
                return
            strategy = await self._read_device_info(client)

        self._emit(
            {
                "type": "connection_stage",
                "stage": "subscribing_notify",
                "message": "Connected, Subscribing Notify",
            }
        )
        await self._subscribe_ack()
        if not self._ack_subscribed:
            if self._connection_recovery.task is not None:
                raise ConnectionFailure("subscription", "ACK 通知订阅失败")
            self._emit_gatt_mismatch(control_status, include_subscriptions=True)
            await self._disconnect_after_gatt_mismatch(client)
            return

        if isinstance(strategy, BootstrapV1Strategy):
            if not await self._query_bootstrap_connection_state():
                if self._connection_recovery.task is not None:
                    raise ConnectionFailure("bootstrap", "Bootstrap 查询未完成")
                self._connection_coordinator.transition(
                    ConnectionStage.FAILED, reason="bootstrap_query_failed"
                )
                await self._safe_disconnect(emit=False)
                return
            if not await self._wait_for_security_ready():
                if self._connection_recovery.task is not None:
                    raise ConnectionFailure("security_timeout", "安全验证超时；未证明配对损坏")
                self._connection_coordinator.transition(
                    ConnectionStage.FAILED, reason="security_timeout"
                )
                await self._safe_disconnect(emit=False)
                return
        else:
            await self._subscribe_export()
            if not self._is_gatt_ready():
                self._emit_gatt_mismatch(control_status, include_subscriptions=True)
                await self._disconnect_after_gatt_mismatch(client)
                return

        self._connection_requested_user_id = user_id
        central_ready = await self._enable_windows_central_mode()
        if self._central_enable_reconnect_requested:
            await self._retry_central_enable_connection(
                address, user_id=user_id, training_id=training_id
            )
            return
        if not central_ready and not self._offline_v2_host_ci_deferred:
            if self._connection_recovery.task is not None:
                raise ConnectionFailure("host_ci", "Host-CI 尚未就绪")
            await self._fail_required_central_mode("connect")
            return
        if not central_ready:
            self._log(
                "[OFF_OBS][DEFER] step=connection_user_sync "
                "action=wait_for_host_ci"
            )
            await self._enter_offline_capture_observer(
                address=address,
                name=name,
                user_id=user_id,
                training_id=training_id,
                central_ready=False,
            )
            return
        if not await self._prepare_connection_user_context(user_id):
            self._enter_recovery_only("connection_user_sync_failed")
            return
        if not self._offline_v2_observer_required():
            self._connection_coordinator.transition(ConnectionStage.SYNCING_CALIBRATION)
            if not await self._initialize_calibration():
                self._enter_recovery_only("calibration_failed")
                return
            if isinstance(strategy, BootstrapV1Strategy):
                await self._subscribe_export()
                if not self._export_subscribed:
                    self._connection_coordinator.transition(
                        ConnectionStage.FAILED, reason="export_cccd_failed"
                    )
                    await self._safe_disconnect(emit=False)
                    return
            if not self._connection_user_context_supported:
                time_sync_result = await self._sync_rtc_after_notify(
                    user_id=user_id, training_id=training_id
                )
                if time_sync_result is False:
                    self._enter_recovery_only("time_sync_failed")
                    return
        else:
            await self._enter_offline_capture_observer(
                address=address,
                name=name,
                user_id=user_id,
                training_id=training_id,
                central_ready=central_ready,
            )
            return
        self._connection_coordinator.transition(ConnectionStage.CONTROL_READY)
        self._offline_v2_commands_ready = central_ready
        self._connection_coordinator.transition(ConnectionStage.SYNCING_OFFLINE)
        if central_ready and not self._offline_v2_observer_required():
            if self._connection_user_context_supported:
                business_ready = await self._run_user_isolated_offline_then_online(
                    "connect_ready",
                    user_id=user_id,
                    training_id=training_id,
                )
            else:
                business_ready = await self._run_offline_priority_then_online("connect_ready")
        else:
            business_ready = False
        if not business_ready:
            self._enter_recovery_only("offline_or_online_ready_failed")
            return
        if isinstance(strategy, BootstrapV1Strategy):
            if not await self._wait_for_business_ready_snapshot(0.5):
                if not await self._query_bootstrap_connection_state():
                    self._enter_recovery_only("device_business_gate_not_confirmed")
                    return
            if self._bootstrap_last_status != 2:
                self._enter_recovery_only("device_business_gate_not_confirmed")
                return
        self._connection_business_ready = True
        self._set_offline_priority_blocked(
            False,
            reason="connect_business_ready",
            phase="ready",
        )
        self._connection_coordinator.transition(ConnectionStage.ONLINE_READY)
        self._connection_coordinator.transition(
            ConnectionStage.BUSINESS_READY, address=address, name=name
        )
        self._emit({"type": "connected", "address": address, "name": name})
        self._log_connection_timing("business_ready")
        self._record_gatt_trust(address)
        await self._flush_pending_actual_stable()
        self._schedule_post_ready_info()
        self._schedule_host_self_check_online()

    def _cancel_recovery_idle_timer(self) -> None:
        task = self._recovery_idle_task
        if task is not None and not task.done():
            try:
                current = asyncio.current_task()
            except RuntimeError:
                current = None
            if task is not current:
                task.cancel()
        self._recovery_idle_task = None

    def _schedule_recovery_idle_disconnect(self) -> None:
        self._cancel_recovery_idle_timer()
        token = self._connection_token

        async def disconnect_if_still_idle() -> None:
            try:
                await asyncio.sleep(RECOVERY_ONLY_IDLE_TIMEOUT_SECONDS)
                if self._recovery_only and self._connection_coordinator.is_current(token):
                    self._log("[BLE_RECOVERY] idle_timeout=120s action=disconnect")
                    await self._safe_disconnect()
            except asyncio.CancelledError:
                return
            finally:
                if self._recovery_idle_task is asyncio.current_task():
                    self._recovery_idle_task = None

        self._recovery_idle_task = asyncio.get_running_loop().create_task(
            disconnect_if_still_idle()
        )

    def _enter_recovery_only(self, reason: str) -> None:
        self._recovery_only = True
        self._connection_business_ready = False
        if self._offline_priority_blocked:
            self._set_offline_priority_blocked(
                True,
                reason=reason,
                phase="recovery",
            )
            self._emit(
                {
                    "type": "offline_capture_observer",
                    "active": True,
                    "device_state": self._last_device_state,
                    "phase": "recovery",
                    "message": "离线数据恢复失败，其他操作保持锁定，请重新连接恢复",
                }
            )
        self._connection_coordinator.transition(
            ConnectionStage.RECOVERY_ONLY,
            reason=reason,
        )
        self._schedule_recovery_idle_disconnect()

    @staticmethod
    def _offline_observer_phase(device_state: int | None) -> str:
        if device_state == DEVICE_STATE_OFFLINE_FINALIZING:
            return "finalizing"
        if device_state == DEVICE_STATE_OFFLINE_SESSION_READY:
            return "syncing"
        return "capturing"

    @staticmethod
    def _offline_observer_message(device_state: int | None) -> str:
        if device_state == DEVICE_STATE_OFFLINE_INTENT_READY:
            return "离线采集准备中"
        if device_state == DEVICE_STATE_OFFLINE_FINALIZING:
            return "离线采集正在规范收尾"
        if device_state == DEVICE_STATE_OFFLINE_SESSION_READY:
            return "离线采集已结束，正在自动上传"
        return "正在离线采集，用户同步将在结束后进行"

    def _invalidate_offline_recovery_epoch(self, reason: str) -> None:
        self._cancel_online_ready_retry()
        future = self._online_ready_ack_future
        if future is not None and not future.done():
            future.cancel()
        self._online_ready_ack_future = None
        self._online_ready_seq = None
        self._online_ready_sent = False
        self._online_ready_ack_ok = False
        self._rtc_sync_ok = False
        self._last_rtc_sync_ms = None
        self._log(f"[OFF_OBS][EPOCH] ready_invalidated=1 reason={reason}")

    def _activate_offline_capture_observer(
        self,
        *,
        address: str,
        name: str,
        user_id: int,
        training_id: int,
        central_ready: bool,
        emit_connected: bool,
        origin: str,
    ) -> None:
        first_activation = not self._offline_observer_active
        phase = self._offline_observer_phase(self._last_device_state)
        message = self._offline_observer_message(self._last_device_state)

        if first_activation:
            self._invalidate_offline_recovery_epoch(
                f"offline_capture_observer_{origin}"
            )
        self._cancel_recovery_idle_timer()
        self._recovery_only = False
        self._offline_observer_active = True
        self._connection_business_ready = False
        self._offline_v2_commands_ready = central_ready
        self._set_offline_priority_blocked(
            True,
            reason=f"offline_capture_observer_{origin}",
            phase=phase,
        )
        if first_activation:
            self._connection_coordinator.transition(
                ConnectionStage.DEFERRED_OFFLINE_CAPTURE,
                address=address,
                name=name,
                device_state=self._last_device_state,
                origin=origin,
            )
            if emit_connected:
                self._emit({"type": "connected", "address": address, "name": name})
        self._emit(
            {
                "type": "offline_capture_observer",
                "active": True,
                "device_state": self._last_device_state,
                "phase": phase,
                "origin": origin,
                "message": message,
            }
        )
        self._log(
            "[OFF_OBS][MODE] active=1 "
            f"origin={origin} phase={phase} "
            f"state=0x{(self._last_device_state or 0):02X} "
            f"generation={self._bootstrap_generation}"
        )
        self._bootstrap_snapshot_event.set()

        task = self._offline_observer_task
        if task is None or task.done():
            self._offline_observer_task = asyncio.create_task(
                self._offline_capture_observer_loop(
                    address=address,
                    name=name,
                    user_id=user_id,
                    training_id=training_id,
                    central_ready=central_ready,
                )
            )

    def _offline_priority_supported(self) -> bool:
        info = self._connection_coordinator.device_info
        return bool(info is not None and info.fields.get("offprio") == "1")

    def _on_offline_v2_event(self, event: dict[str, Any]) -> None:
        status = event.get("status")
        if status == "capture_preempted":
            self._set_offline_priority_blocked(True, reason="local_capture", phase="capturing")
        elif status == "capture_finished":
            self._set_offline_priority_blocked(True, reason="local_capture_finished", phase="syncing")
        self._emit(event)

    def _observe_priority_capture(self, device_state: int, source: str) -> bool:
        sync = self._offline_v2_sync
        if self._client is None or not self._client.is_connected or not sync.connected:
            return False
        sync.local_priority_supported = self._offline_priority_supported()
        if not sync.local_priority_supported or not sync.priority_in_progress:
            return False
        if device_state in {DEVICE_STATE_OFFLINE_INTENT_READY, DEVICE_STATE_OFFLINE_CAPTURING,
                            DEVICE_STATE_OFFLINE_FINALIZING}:
            sync.pause_for_local_capture()
            self._set_offline_priority_blocked(True, reason="local_capture",
                                              phase=self._offline_observer_phase(device_state))
            return True
        if sync.state == OfflineV2SyncState.WAIT_CAPTURE:
            # Only authoritative state/bootstrap notifications can release the
            # wait; an old command ACK can contain a pre-capture terminal state.
            if source in {"bootstrap", "state_notify"} and device_state in {
                    DEVICE_STATE_OFFLINE_SESSION_READY, DEVICE_STATE_WAIT_START}:
                sync.resume_after_local_capture()
            return True
        return False

    def _observe_runtime_offline_state(
        self,
        device_state: int,
        *,
        source: str,
    ) -> None:
        if self._observe_priority_capture(device_state, source):
            self._offline_runtime_last_state = device_state
            return
        previous_state = self._offline_runtime_last_state
        self._offline_runtime_last_state = device_state
        if device_state not in {
            DEVICE_STATE_OFFLINE_INTENT_READY,
            DEVICE_STATE_OFFLINE_CAPTURING,
            DEVICE_STATE_OFFLINE_FINALIZING,
            DEVICE_STATE_OFFLINE_SESSION_READY,
        }:
            return

        client = self._client
        if client is None or not client.is_connected:
            return
        if not self._connection_business_ready and not self._offline_observer_active:
            # Initial connection bootstrap owns this case and enters the same
            # observer after Host-CI/security setup has reached its safe point.
            return
        if previous_state == device_state and not self._offline_observer_active:
            return
        if previous_state == device_state and self._offline_observer_active:
            self._bootstrap_snapshot_event.set()
            return

        self._last_device_state = device_state
        self._activate_offline_capture_observer(
            address=self._connected_address or "<unknown>",
            name=self._current_device_name or "ZY100",
            user_id=self._connection_requested_user_id or self._active_user_id or 1,
            training_id=self._active_training_id or 1,
            central_ready=self._central_business_ready(),
            emit_connected=False,
            origin=f"runtime_{source}",
        )

    async def _enter_offline_capture_observer(
        self,
        *,
        address: str,
        name: str,
        user_id: int,
        training_id: int,
        central_ready: bool,
    ) -> None:
        self._activate_offline_capture_observer(
            address=address,
            name=name,
            user_id=user_id,
            training_id=training_id,
            central_ready=central_ready,
            emit_connected=True,
            origin="connect",
        )

    async def _offline_capture_observer_loop(
        self,
        *,
        address: str,
        name: str,
        user_id: int,
        training_id: int,
        central_ready: bool,
    ) -> None:
        token = self._connection_token
        legacy_intent_deadline: float | None = None
        user_context_ready = self._connection_user_context_ready_for(user_id)
        try:
            while self._offline_observer_active and self._connection_coordinator.is_current(token):
                client = self._client
                if client is None or not client.is_connected:
                    return
                state = self._last_device_state
                if not central_ready and self._legacy_offline_intent_recovery_required():
                    if legacy_intent_deadline is None:
                        legacy_intent_deadline = (
                            asyncio.get_running_loop().time()
                            + LEGACY_OFFLINE_INTENT_GRACE_SECONDS
                        )
                        self._log(
                            "[OFF_OBS][LEGACY_RECOVERY] phase=grace "
                            f"code={self._device_version_code()} state=0x{state:02X} "
                            f"timeout_s={LEGACY_OFFLINE_INTENT_GRACE_SECONDS:.1f}"
                        )
                    remaining = legacy_intent_deadline - asyncio.get_running_loop().time()
                    if remaining > 0:
                        self._bootstrap_snapshot_event.clear()
                        try:
                            await asyncio.wait_for(
                                self._bootstrap_snapshot_event.wait(),
                                timeout=remaining,
                            )
                        except asyncio.TimeoutError:
                            pass
                        continue
                    if not self._legacy_offline_intent_reconnect_used:
                        self._legacy_offline_intent_reconnect_used = True
                        self._log(
                            "[OFF_OBS][LEGACY_RECOVERY] phase=controlled_reconnect "
                            f"code={self._device_version_code()} state=0x{state:02X} attempt=1"
                        )
                        self._emit(
                            {
                                "type": "connection_stage",
                                "stage": "legacy_offline_intent_reconnect",
                                "message": "检测到旧固件离线启动卡住，正在执行一次安全重连恢复",
                            }
                        )
                        await self._safe_disconnect(emit=False)
                        await asyncio.sleep(LEGACY_OFFLINE_INTENT_RECONNECT_DELAY_SECONDS)
                        await self._connect(
                            address,
                            user_id=user_id,
                            training_id=training_id,
                        )
                        return
                    self._offline_observer_active = False
                    self._log(
                        "[OFF_OBS][LEGACY_RECOVERY][ERR] result=power_cycle_required "
                        f"code={self._device_version_code()} state=0x{state:02X}"
                    )
                    self._emit(
                        {
                            "type": "error",
                            "message": (
                                "旧固件的离线启动事务仍卡在 0x0E；该版本没有 BLE 取消命令。"
                                "请长按关机后重新开机，或断电/接入 VIN 完成一次冷启动。"
                            ),
                        }
                    )
                    self._enter_recovery_only("legacy_offline_intent_power_cycle_required")
                    return
                legacy_intent_deadline = None
                # A deferred Host-CI belongs to the post-capture bootstrap.
                # Existing leases keep their heartbeat; notification updates wake us.
                if central_ready and not user_context_ready:
                    user_context_ready = await self._ensure_connection_user_context_ready(user_id)
                    if not user_context_ready and self._user_sync_result != UserSyncResult.WAITING_CAPTURE:
                        self._offline_observer_active = False
                        self._enter_recovery_only("observer_connection_user_sync_failed")
                        return
                state = self._last_device_state
                optional_sync_allowed = state != DEVICE_STATE_OFFLINE_FINALIZING
                calibration_sync_allowed = optional_sync_allowed and (
                    state != DEVICE_STATE_OFFLINE_SESSION_READY
                )
                if central_ready and optional_sync_allowed and not self._battery_subscribed:
                    self._log("[OFF_OBS][STEP] step=battery_cache action=read_subscribe")
                    await self._initialize_battery()
                if (
                    central_ready
                    and calibration_sync_allowed
                    and self._calibration_gate_state == "unknown"
                ):
                    self._log("[OFF_OBS][STEP] step=calibration_summary action=cache_only")
                    await self._initialize_calibration(allow_record_sync=False)

                if state in {DEVICE_STATE_OFFLINE_SESSION_READY, DEVICE_STATE_WAIT_START}:
                    await self._finish_offline_capture_observer(
                        address=address,
                        name=name,
                        user_id=user_id,
                        training_id=training_id,
                        central_ready=central_ready,
                    )
                    if self._offline_observer_active and self._offline_v2_capture_in_progress():
                        continue
                    return
                if state is not None and not self._offline_v2_capture_in_progress():
                    self._offline_observer_active = False
                    self._log(
                        "[OFF_OBS][EXIT] result=unexpected_terminal_state "
                        f"state=0x{state:02X}"
                    )
                    self._enter_recovery_only(
                        "observer_terminal_state_without_session_ready"
                    )
                    return

                self._bootstrap_snapshot_event.clear()
                try:
                    await asyncio.wait_for(self._bootstrap_snapshot_event.wait(), timeout=15.0)
                except asyncio.TimeoutError:
                    # The 5 s Host-CI heartbeat maintains the lease; state is notification driven.
                    continue
        except asyncio.CancelledError:
            return
        except Exception as exc:
            self._offline_observer_active = False
            self._log(f"[OFF_OBS][EXIT] result=observer_error reason={exc}")
            self._emit({"type": "error", "message": f"Offline observer failed: {exc}"})
            self._enter_recovery_only("observer_runtime_error")

    async def _finish_offline_capture_observer(
        self,
        *,
        address: str,
        name: str,
        user_id: int,
        training_id: int,
        central_ready: bool,
    ) -> None:
        token = self._connection_token
        self._log(
            "[OFF_OBS][EXIT] result=capture_finished "
            f"state=0x{(self._last_device_state or 0):02X} reason={self._bootstrap_reason or 0}"
        )
        if not central_ready:
            central_ready = await self._enable_windows_central_mode()
        if not self._connection_coordinator.is_current(token):
            return
        if not central_ready:
            self._offline_observer_active = False
            self._enter_recovery_only("observer_ci_not_ready_after_capture")
            return
        if not await self._ensure_connection_user_context_ready(user_id):
            if self._user_sync_result == UserSyncResult.WAITING_CAPTURE:
                return
            if not self._connection_coordinator.is_current(token):
                return
            self._offline_observer_active = False
            self._enter_recovery_only("observer_connection_user_sync_failed")
            return
        await self._subscribe_export()
        if not self._export_subscribed:
            self._offline_observer_active = False
            self._enter_recovery_only("observer_export_cccd_failed")
            return

        self._offline_v2_commands_ready = True
        self._set_offline_priority_blocked(
            True,
            reason="observer_capture_finished",
            phase="syncing",
        )
        self._emit(
            {
                "type": "offline_capture_observer",
                "active": True,
                "device_state": self._last_device_state,
                "phase": "syncing",
                "message": "离线采集已结束，正在自动上传",
            }
        )
        self._connection_coordinator.transition(ConnectionStage.SYNCING_OFFLINE)
        self._offline_v2_sync.local_priority_supported = self._offline_priority_supported()
        await self._offline_v2_sync.start_sync(reason="observer_capture_finished")
        completed = await self._offline_v2_sync.wait_for_priority_completion()
        if not completed:
            self._offline_observer_active = False
            self._enter_recovery_only("observer_offline_sync_failed")
            return
        if self._connection_user_context_supported:
            if not await self._prompt_foreign_sessions_if_needed():
                self._offline_observer_active = False
                self._enter_recovery_only("observer_foreign_query_failed")
                return

        self._connection_coordinator.transition(ConnectionStage.SYNCING_TIME)
        if not await self._sync_rtc_after_notify(user_id=user_id, training_id=training_id):
            self._offline_observer_active = False
            self._enter_recovery_only("observer_time_sync_failed")
            return
        self._connection_coordinator.transition(ConnectionStage.SYNCING_CALIBRATION)
        if not await self._initialize_calibration(allow_record_sync=True):
            self._offline_observer_active = False
            self._enter_recovery_only("observer_calibration_failed")
            return

        if not await self._send_online_stream_ready_if_possible(
            "observer_after_offline",
            wait_ack=True,
            allow_offline_gate=True,
        ):
            self._offline_observer_active = False
            self._enter_recovery_only("observer_online_ready_failed")
            return
        self._connection_coordinator.transition(
            ConnectionStage.SYNCING_FEATURE_CONFIG
        )
        if not await self._sync_feature_config("observer_after_offline"):
            self._offline_observer_active = False
            self._enter_recovery_only("observer_feature_config_failed")
            return
        if isinstance(self._connection_coordinator.strategy, BootstrapV1Strategy):
            if not await self._query_bootstrap_connection_state():
                self._offline_observer_active = False
                self._enter_recovery_only("observer_business_query_failed")
                return
            if self._bootstrap_last_status != 2:
                self._offline_observer_active = False
                self._enter_recovery_only("observer_business_gate_not_confirmed")
                return
        self._offline_observer_active = False
        self._connection_business_ready = True
        self._set_offline_priority_blocked(
            False,
            reason="observer_business_ready",
            phase="ready",
        )
        self._connection_coordinator.transition(ConnectionStage.ONLINE_READY)
        self._connection_coordinator.transition(
            ConnectionStage.BUSINESS_READY, address=address, name=name
        )
        self._emit(
            {
                "type": "offline_capture_observer",
                "active": False,
                "device_state": self._last_device_state,
                "phase": "ready",
                "message": "离线数据已上传，设备已恢复可操作状态",
            }
        )
        self._record_gatt_trust(address)
        await self._flush_pending_actual_stable()
        self._schedule_post_ready_info()
        self._schedule_host_self_check_online()

    async def _connect_with_gatt_retries(
        self,
        device: BLEDevice,
        address: str,
        name: str,
    ) -> tuple[BleakClient, dict[str, Any]]:
        delays = tuple(self._gatt_retry_delays_s)
        total_attempts = len(delays) + 1
        last_error: Exception | None = None
        trusted = self._gatt_trust_store.get(address)
        self._gatt_fast_path_active = False

        if trusted is not None and sys.platform == "win32":
            self._log(f"[BLE_GATT][FAST] trusted=1 address={address}")
            try:
                client, control_status = await self._connect_and_discover_once(
                    device,
                    address,
                    name,
                    attempt=1,
                    total_attempts=1,
                    discovery_mode=GATT_DISCOVERY_ZY100_CONTROL,
                    use_cached_services=True,
                )
                if self._has_required_gatt():
                    self._gatt_fast_path_active = True
                    self._log("[BLE_GATT][FAST] cache_hit=1 required_gatt=1")
                    return client, control_status
                self._log("[BLE_GATT][FAST] invalid required_gatt=0 fallback=uncached")
                self._gatt_trust_store.invalidate(address)
                await self._disconnect_for_gatt_retry(client)
            except Exception as exc:
                self._gatt_trust_store.invalidate(address)
                if self._is_pairing_failure(exc):
                    self._log(
                        "[OFF_OBS][PAIR] trusted=1 status=failed "
                        f"action=stop_retry reason={exc}"
                    )
                    raise
                self._log(f"[BLE_GATT][FAST] failed reason={exc} fallback=uncached")

        for attempt in range(1, total_attempts + 1):
            if attempt > 1:
                delay = delays[attempt - 2]
                self._emit(
                    {
                        "type": "connection_stage",
                        "stage": "retrying_gatt",
                        "message": "Retrying GATT discovery",
                    }
                )
                self._log(f"[BLE_GATT][RETRY] wait_s={delay:.1f} attempt={attempt}/{total_attempts}")
                if delay > 0:
                    await asyncio.sleep(delay)
                refreshed_device = self._make_winrt_cached_device(address, name)
                if refreshed_device is not None:
                    device = refreshed_device
                    self._devices[address] = refreshed_device

            try:
                discovery_mode = (
                    GATT_DISCOVERY_ZY100_CONTROL
                    if attempt == total_attempts
                    else GATT_DISCOVERY_ALL_SERVICES
                )
                client, control_status = await self._connect_and_discover_once(
                    device,
                    address,
                    name,
                    attempt=attempt,
                    total_attempts=total_attempts,
                    discovery_mode=discovery_mode,
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                last_error = exc
                if attempt >= total_attempts:
                    raise
                self._log(f"[BLE_GATT][RETRY] connect/discovery failed attempt={attempt}/{total_attempts} reason={exc}")
                continue

            if self._has_required_gatt():
                return client, control_status

            if not self._should_retry_gatt_discovery():
                return client, control_status

            if attempt >= total_attempts:
                recovered = await self._hard_reconnect_after_gatt_failure(
                    client,
                    device,
                    address,
                    name,
                    next_attempt=total_attempts + 1,
                )
                if recovered is not None:
                    return recovered
                return client, control_status

            missing = ", ".join(self._required_gatt_missing()) or "unknown"
            service_count = len(self._last_service_objects)
            self._log(
                "[BLE_GATT][RETRY] "
                f"incomplete discovery attempt={attempt}/{total_attempts} "
                f"service_count={service_count} missing={missing}"
            )
            await self._disconnect_for_gatt_retry(client)

        if last_error is not None:
            raise last_error
        raise RuntimeError("GATT discovery retry exhausted")

    @staticmethod
    def _is_pairing_failure(error: BaseException) -> bool:
        message = str(error).casefold()
        return any(marker in message for marker in PAIRING_FAILURE_MARKERS)

    async def _hard_reconnect_after_gatt_failure(
        self,
        client: BleakClient,
        device: BLEDevice,
        address: str,
        name: str,
        *,
        next_attempt: int,
    ) -> tuple[BleakClient, dict[str, Any]] | None:
        self._emit(
            {
                "type": "connection_stage",
                "stage": "hard_reconnect",
                "message": "Resetting BLE session before reconnect",
            }
        )
        self._log(
            "[BLE_GATT][RECOVER] "
            f"hard_reconnect begin address={address} delay_s={self._gatt_hard_reconnect_delay_s:.1f}"
        )
        await self._disconnect_for_gatt_retry(client)
        if self._gatt_hard_reconnect_delay_s > 0:
            await asyncio.sleep(self._gatt_hard_reconnect_delay_s)

        scan_device = await self._scan_for_device_by_address(
            address,
            timeout=self._gatt_hard_reconnect_scan_s,
        )
        if scan_device is not None:
            device = scan_device
            self._devices[address] = scan_device
            name = scan_device.name or name
            self._log(f"[BLE_GATT][RECOVER] scan_found=1 address={address} name={name}")
        else:
            refreshed_device = self._make_winrt_cached_device(address, name)
            if refreshed_device is not None:
                device = refreshed_device
                self._devices[address] = refreshed_device
            self._log(f"[BLE_GATT][RECOVER] scan_found=0 fallback=windows_cached address={address}")

        try:
            recovered = await self._connect_and_discover_once(
                device,
                address,
                name,
                attempt=next_attempt,
                total_attempts=next_attempt,
                discovery_mode=GATT_DISCOVERY_ZY100_CONTROL,
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_GATT][RECOVER] reconnect failed reason={exc}")
            return None

        if self._has_required_gatt():
            self._log("[BLE_GATT][RECOVER] hard_reconnect ok")
            return recovered

        missing = ", ".join(self._required_gatt_missing()) or "unknown"
        self._log(f"[BLE_GATT][RECOVER] hard_reconnect incomplete missing={missing}")
        return recovered

    async def _scan_for_device_by_address(self, address: str, *, timeout: float) -> BLEDevice | None:
        if timeout <= 0:
            return None

        target = address.upper()
        self._log(f"[BLE_GATT][RECOVER] scan_exact start timeout_s={timeout:.1f} address={address}")
        try:
            discovered = await BleakScanner.discover(timeout=timeout, return_adv=True)
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_GATT][RECOVER] scan_exact failed reason={exc}")
            return None

        for discovered_address, value in discovered.items():
            if str(discovered_address).upper() != target:
                continue
            device, adv = value
            name = (device.name or getattr(adv, "local_name", "") or "").strip()
            if name and not self._is_zp_device_name(name):
                self._log(f"[BLE_GATT][RECOVER] scan_exact ignored non_zp name={name} address={address}")
                return None
            return device

        return None

    async def _connect_and_discover_once(
        self,
        device: BLEDevice,
        address: str,
        name: str,
        *,
        attempt: int,
        total_attempts: int,
        discovery_mode: str,
        use_cached_services: bool = False,
    ) -> tuple[BleakClient, dict[str, Any]]:
        if getattr(self, "_cleanup_blocked", False):
            from .connection_recovery import ConnectionFailure
            raise ConnectionFailure("cleanup", "上次 BLE 资源释放失败；请退出并重新启动上位机")
        self._clear_control_state()
        self._connection_token = self._connection_coordinator.begin(address)
        self._link_trace.begin_connection(self._connection_token, address)
        self._last_services_payload = []
        self._last_service_objects = []
        requested_services = (
            {
                ZY100_CONTROL_SERVICE_UUID,
                ZY100_FACTORY_SERVICE_UUID,
                CALIBRATION_SERVICE_UUID,
                BATTERY_SERVICE_UUID,
                LEGACY_SIMPLE_SERVICE_UUID,
                REALTEK_OTA_SERVICE_UUID,
            }
            if discovery_mode == GATT_DISCOVERY_ZY100_CONTROL
            else None
        )
        self._log(
            f"Connecting: name={name} address={address} attempt={attempt}/{total_attempts} "
            f"discovery={discovery_mode}"
        )
        backend_options = {}
        if sys.platform == "win32":
            from .winrt_discovery import DiagnosticWinRTClient
            backend_options = {
                "backend": DiagnosticWinRTClient,
                "diagnostic_log": lambda message, op=self._connection_recovery.operation_id, token=self._connection_token: self._log(
                    f"{message} op={op} token={token}"),
                "required_gatt": {ZY100_CONTROL_SERVICE_UUID: {
                    ZY100_COMMAND_UUID, ZY100_ACK_UUID, ZY100_EXPORT_DATA_UUID, ZY100_DEVICE_INFO_UUID}},
                "bootstrap_services": {CALIBRATION_SERVICE_UUID, BATTERY_SERVICE_UUID,
                                       LEGACY_SIMPLE_SERVICE_UUID, REALTEK_OTA_SERVICE_UUID,
                                       ZY100_FACTORY_SERVICE_UUID},
            }
        client = BleakClient(
            device,
            services=requested_services,
            disconnected_callback=self._connection_coordinator.guard_callback(
                self._connection_token, self._handle_disconnected
            ),
            pair=True,
            timeout=30.0,
            winrt={"use_cached_services": use_cached_services},
            **backend_options,
        )
        try:
            await client.connect()
            self._link_trace.link_connected(
                attempt=attempt,
                total_attempts=total_attempts,
                discovery_mode=discovery_mode,
                use_cached_services=use_cached_services,
            )
            self._connection_coordinator.transition(
                ConnectionStage.LINK_CONNECTED, address=address, name=name
            )
            self._connection_coordinator.transition(ConnectionStage.DISCOVERING_GATT)
            self._emit(
                {
                    "type": "connection_stage",
                    "stage": "discovering",
                    "message": "Connected, Discovering Services",
                }
            )
            services = await get_bleak_services(client)
            self._last_service_objects = list(services)

            self._client = client
            self._connected_address = address
            self._offline_v2_sync.set_connection(
                connected=True,
                device_name=name,
                device_address=address,
            )
            self._ci_state.transition(
                HostCiState.ACTIVE_IDLE,
                reason="transport_connected",
                snapshot=self._export_speed.read_link_snapshot(client),
            )
            self._cancel_ci_observer()

            self._log(
                "[BLE_CONNECT] "
                f"transport_connected=1 name={name} address={address} "
                f"max_links={ZY100_MAX_SIMULTANEOUS_LINKS} max_paired={ZY100_MAX_PAIRED_CENTRALS} "
                f"attempt={attempt}/{total_attempts} discovery={discovery_mode}"
            )
            await self._enumerate_services(client, services=services)
            control_status = self._inspect_control_service(client)
            self._inspect_calibration_service(client)
            return client, control_status
        except BaseException as exc:
            self._link_trace.record_exception(
                "link_or_discovery_failed",
                exc,
                attempt=attempt,
                discovery_mode=discovery_mode,
                gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
            )
            await self._disconnect_for_gatt_retry(client)
            raise

    def _should_retry_gatt_discovery(self) -> bool:
        if self._has_required_gatt():
            return False
        return True

    async def _disconnect_for_gatt_retry(self, client: BleakClient) -> None:
        from .client_lifecycle import release_client, ClientCleanupError
        try:
            self._expected_disconnect_client_ids.add(id(client))
            await release_client(client, self._log)
        except ClientCleanupError as exc:
            from .connection_recovery import ConnectionFailure
            raise ConnectionFailure("cleanup", str(exc)) from exc
        finally:
            self._force_close_client_backend(client, reason="gatt_retry")
            if self._client is client:
                self._client = None
                self._connected_address = None
            self._last_services_payload = []
            self._last_service_objects = []
            self._clear_control_state()

    async def _disconnect_after_gatt_mismatch(self, client: BleakClient) -> None:
        try:
            if self._ack_subscribed and self._ack_uuid:
                try:
                    await self._connection_coordinator.stop_notify(
                        self._connection_token, client, self._ack_uuid,
                        name="stop_ack_gatt_mismatch",
                    )
                except Exception as exc:  # pragma: no cover - host stack dependent
                    self._log(f"Stop ACK notify after GATT_MISMATCH failed: {exc}")
            if self._export_subscribed and self._export_uuid:
                try:
                    await self._connection_coordinator.stop_notify(
                        self._connection_token, client, self._export_uuid,
                        name="stop_export_gatt_mismatch",
                    )
                except Exception as exc:  # pragma: no cover - host stack dependent
                    self._log(f"Stop Export Data notify after GATT_MISMATCH failed: {exc}")
            if self._battery_subscribed and self._battery_level_char is not None:
                try:
                    await self._connection_coordinator.stop_notify(
                        self._connection_token, client, self._battery_level_char,
                        name="stop_battery_gatt_mismatch",
                    )
                except Exception as exc:  # pragma: no cover - host stack dependent
                    self._log(f"Stop Battery Level notify after GATT_MISMATCH failed: {exc}")
            await self._stop_ota_dfu_entry_notify(client)
            if client.is_connected:
                self._expected_disconnect_client_ids.add(id(client))
                await client.disconnect()
            self._force_close_client_backend(client, reason="gatt_mismatch")
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"Disconnect after GATT_MISMATCH failed: {exc}")
        finally:
            if self._client is client:
                self._client = None
                self._connected_address = None
            self._clear_control_state()
            self._log("[BLE_GATT][MISMATCH] disconnected=1")

    def _force_close_client_backend(self, client: BleakClient, *, reason: str) -> bool:
        from .client_lifecycle import close_backend
        clean = close_backend(client, lambda message: self._log(f"{message} reason={reason}"))
        if not clean:
            self._cleanup_blocked = True
        return clean

    async def _enumerate_services(self, client: BleakClient, services: Iterable[Any] | None = None) -> None:
        try:
            lines: list[str] = []
            services_payload: list[dict[str, Any]] = []
            services = list(services if services is not None else self._iter_services(client))
            service_count = len(services)
            self._log(f"[BLE_GATT] service_count={service_count}")
            for idx, service in enumerate(services):
                service_uuid = service.uuid.lower()
                self._log(f"[BLE_GATT] service[{idx}]={service_uuid}")
                lines.append(f"[Service] {service.uuid} | {service.description}")
                char_payload: list[dict[str, Any]] = []
                for char in service.characteristics:
                    props = ",".join(char.properties) if char.properties else "<none>"
                    handle = getattr(char, "handle", None)
                    handle_text = f"0x{handle:04X}" if isinstance(handle, int) else ""
                    self._log(
                        "[BLE_GATT] "
                        f"char service={service_uuid} char={char.uuid.lower()} properties={props} "
                        f"handle={handle_text or '<unknown>'}"
                    )
                    lines.append(
                        f"    - [Char] {char.uuid} | {char.description} | {props}"
                        f"{' | handle=' + handle_text if handle_text else ''}"
                    )
                    char_payload.append(
                        {
                            "uuid": char.uuid,
                            "description": char.description,
                            "properties": list(char.properties),
                            "handle": handle,
                        }
                    )
                services_payload.append(
                    {
                        "uuid": service.uuid,
                        "description": service.description,
                        "characteristics": char_payload,
                    }
                )

            if service_count == 0:
                self._last_services_payload = []
                self._last_service_objects = []
                self._log("No GATT services discovered")
                self._emit({"type": "services", "lines": [], "services": []})
                return

            self._last_service_objects = list(services)
            self._last_services_payload = deepcopy(services_payload)
            lines.append("")
            lines.append("[ZY100 9ECA Control/Export UUID Mapping]")
            lines.append(f"Control Service: {ZY100_CONTROL_SERVICE_UUID}")
            lines.append(f"Command: {ZY100_COMMAND_UUID}")
            lines.append(f"ACK / Status: {ZY100_ACK_UUID}")
            lines.append(f"Device Info: {ZY100_DEVICE_INFO_UUID}")
            lines.append(f"Export Data: {ZY100_EXPORT_DATA_UUID}")
            lines.append("")
            lines.append("[ZY100 Factory Read-only UUID Mapping]")
            lines.append(f"Factory Service: {ZY100_FACTORY_SERVICE_UUID}")
            lines.append(f"MfgInfo: {ZY100_MFG_INFO_UUID}")
            lines.append(f"DeviceChannel: {ZY100_DEVICE_CHANNEL_UUID}")
            lines.append("")
            lines.append("[ZY100 9ECA Calibration UUID Mapping]")
            lines.append(f"Calibration Service: {CALIBRATION_SERVICE_UUID}")
            lines.append(f"Calibration Info: {CALIBRATION_INFO_UUID}")
            lines.append(f"Calibration TX: {CALIBRATION_TX_UUID}")
            lines.append(f"Calibration RX: {CALIBRATION_RX_UUID}")
            lines.append(f"Calibration Status: {CALIBRATION_STATUS_UUID}")
            lines.append("")
            lines.append("[Standard Battery Service UUID Mapping]")
            lines.append(f"Battery Service: {BATTERY_SERVICE_UUID}")
            lines.append(f"Battery Level: {BATTERY_LEVEL_UUID}")

            self._log("GATT discovery completed")
            self._emit({"type": "services", "lines": lines, "services": services_payload})
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"Service discovery failed: {exc}")
            self._emit({"type": "error", "message": f"Service discovery failed: {exc}"})

    def _inspect_control_service(self, client: BleakClient) -> dict[str, Any]:
        service_found = any(
            self._normalize_uuid(service.uuid) == ZY100_CONTROL_SERVICE_UUID for service in self._iter_services(client)
        )
        command_char = self._find_characteristic(client, ZY100_COMMAND_UUID)
        ack_char = self._find_characteristic(client, ZY100_ACK_UUID)
        info_char = self._find_characteristic(client, ZY100_DEVICE_INFO_UUID)
        export_char = self._find_characteristic(client, ZY100_EXPORT_DATA_UUID)

        self._zy100_control_found = service_found
        self._command_found = command_char is not None
        self._ack_found = ack_char is not None
        self._export_found = export_char is not None
        self._command_uuid = command_char.uuid.lower() if command_char is not None else None
        self._ack_uuid = ack_char.uuid.lower() if ack_char is not None else None
        self._export_uuid = export_char.uuid.lower() if export_char is not None else None
        self._inspect_battery_service(client)

        self._log(
            "[BLE_SERVICE] "
            f"zy100_control_found={1 if service_found else 0} "
            f"command_found={1 if command_char else 0} ack_found={1 if ack_char else 0} "
            f"export_found={1 if export_char else 0}"
        )
        payload = {
            "type": "zy100_control_status",
            "service_found": service_found,
            "command_found": command_char is not None,
            "ack_found": ack_char is not None,
            "device_info_found": info_char is not None,
            "export_found": export_char is not None,
            "command_uuid": self._command_uuid or "",
            "ack_uuid": self._ack_uuid or "",
            "device_info_uuid": info_char.uuid.lower() if info_char is not None else "",
            "export_uuid": self._export_uuid or "",
            "command_properties": list(command_char.properties) if command_char is not None else [],
            "ack_properties": list(ack_char.properties) if ack_char is not None else [],
            "export_properties": list(export_char.properties) if export_char is not None else [],
        }
        self._emit(payload)
        return payload

    def _inspect_calibration_service(self, client: BleakClient) -> dict[str, Any]:
        service_found = any(
            self._normalize_uuid(service.uuid) == CALIBRATION_SERVICE_UUID
            for service in self._iter_services(client)
        )
        info_char = self._find_characteristic_in_service(client, CALIBRATION_SERVICE_UUID, CALIBRATION_INFO_UUID)
        tx_char = self._find_characteristic_in_service(client, CALIBRATION_SERVICE_UUID, CALIBRATION_TX_UUID)
        rx_char = self._find_characteristic_in_service(client, CALIBRATION_SERVICE_UUID, CALIBRATION_RX_UUID)
        status_char = self._find_characteristic_in_service(
            client, CALIBRATION_SERVICE_UUID, CALIBRATION_STATUS_UUID
        )
        self._calibration_service_found = service_found
        self._calibration_info_uuid = info_char.uuid.lower() if info_char is not None else None
        self._calibration_tx_uuid = tx_char.uuid.lower() if tx_char is not None else None
        self._calibration_rx_uuid = rx_char.uuid.lower() if rx_char is not None else None
        self._calibration_status_uuid = status_char.uuid.lower() if status_char is not None else None
        payload = {
            "type": "calibration_capability",
            "available": bool(service_found and info_char and tx_char and rx_char and status_char),
            "service_found": service_found,
            "info_found": info_char is not None,
            "tx_found": tx_char is not None,
            "rx_found": rx_char is not None,
            "status_found": status_char is not None,
        }
        self._log(
            "[CAL_GATT] "
            f"service={int(service_found)} info={int(info_char is not None)} "
            f"tx={int(tx_char is not None)} rx={int(rx_char is not None)} "
            f"status={int(status_char is not None)}"
        )
        self._emit(payload)
        return payload

    def _inspect_battery_service(self, client: BleakClient) -> BleakGATTCharacteristic | None:
        battery_char = self._find_characteristic_in_service(
            client,
            BATTERY_SERVICE_UUID,
            BATTERY_LEVEL_UUID,
        )
        self._battery_level_char = battery_char
        self._battery_level_uuid = battery_char.uuid.lower() if battery_char is not None else None
        self._battery_subscribed = False
        self._battery_percent = None

        if battery_char is None:
            self._log("[BLE_BATTERY] battery_found=0")
            return None

        props = ",".join(battery_char.properties) if battery_char.properties else "<none>"
        handle = getattr(battery_char, "handle", None)
        handle_text = f"0x{handle:04X}" if isinstance(handle, int) else "<unknown>"
        self._log(
            "[BLE_BATTERY] "
            f"battery_found=1 uuid={self._battery_level_uuid} properties={props} handle={handle_text}"
        )
        return battery_char

    async def _read_device_info(self, client: BleakClient) -> Any:
        self._connection_coordinator.transition(ConnectionStage.READING_DEVICE_INFO)
        info_char = self._find_characteristic(client, ZY100_DEVICE_INFO_UUID)
        if info_char is None or not hasattr(client, "read_gatt_char"):
            raise UnsupportedProtocolError("Device Info characteristic is required")
        try:
            raw = await self._connection_coordinator.read_gatt_char(
                self._connection_token, client, info_char.uuid,
                name="read_device_info",
            )
            text = bytes(raw).decode("ascii", errors="replace").strip("\x00\r\n ")
            strategy = self._connection_coordinator.lock_device_info(text)
            expected_sn = self._connection_recovery.expected_sn
            actual_sn = self._connection_coordinator.device_info.fields.get("sn", "")
            if expected_sn and actual_sn != expected_sn:
                raise ConnectionFailure("identity", "目标设备 SN 不匹配，已停止恢复")
            self._device_info_text = text
            self._log(f"[BLE_DEVICE_INFO] {text}")
            self._emit(
                {
                    "type": "device_info",
                    "value": text,
                    "raw_hex": hex_bytes(bytes(raw)),
                    "strategy": strategy.name,
                }
            )
            return strategy
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"Read Device Info failed: {exc}")
            raise

    async def _read_mfg_info(self, client: BleakClient) -> None:
        info_char = self._find_characteristic(client, ZY100_MFG_INFO_UUID)
        if info_char is None or not hasattr(client, "read_gatt_char"):
            self._log("[BLE_MFG_INFO] characteristic_missing=1")
            return
        try:
            raw = await self._connection_coordinator.read_gatt_char(
                self._connection_token, client, info_char.uuid,
                priority=GattPriority.BACKGROUND,
                name="read_mfg_info",
            )
            text = bytes(raw).decode("ascii", errors="replace").strip("\x00\r\n ")
            field_names = {
                part.split("=", 1)[0].strip().lower()
                for part in text.split(";")
                if "=" in part
            }
            self._log(
                "[BLE_MFG_INFO] "
                f"len={len(raw)} text_len={len(text)} "
                f"has_sn={int(bool({'sn', 'serial', 'serial_number'} & field_names))} "
                f"has_channel={int('channel' in field_names)} value={text}"
            )
            self._emit({"type": "mfg_info", "value": text, "raw_hex": hex_bytes(bytes(raw))})
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_MFG_INFO] read_failed error={exc}")

    async def _read_device_channel(self, client: BleakClient) -> None:
        """Read the production short channel characteristic once.

        This intentionally uses the normal queued GATT read.  MfgInfo remains
        the legacy fallback and is still read independently for serial and
        diagnostic fields.
        """
        info_char = self._find_characteristic_in_service(
            client,
            ZY100_FACTORY_SERVICE_UUID,
            ZY100_DEVICE_CHANNEL_UUID,
        )
        self._device_channel_supported = info_char is not None
        self._device_channel_received = False
        self._device_channel_value = None
        if info_char is None or not hasattr(client, "read_gatt_char"):
            self._log("[BLE_DEVICE_CHANNEL] characteristic_missing=1")
            return
        try:
            raw = await self._connection_coordinator.read_gatt_char(
                self._connection_token,
                client,
                info_char.uuid,
                priority=GattPriority.BACKGROUND,
                name="read_device_channel",
            )
            raw_bytes = bytes(raw)
            text = raw_bytes.decode("ascii", errors="replace")
            valid = text in {"CG", "ST", "DEFAULT"}
            value = text if valid else "DEFAULT"
            self._device_channel_received = True
            self._device_channel_value = value
            self._log(
                f"[BLE_DEVICE_CHANNEL] value={value} valid={1 if valid else 0} "
                f"raw_hex={hex_bytes(raw_bytes)}"
            )
            self._emit(
                {
                    "type": "device_channel",
                    "value": value,
                    "source": "device_channel",
                    "supported": True,
                    "valid": valid,
                    "raw_hex": hex_bytes(raw_bytes),
                }
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_DEVICE_CHANNEL] read_failed error={exc}")

    async def _wait_for_business_ready_snapshot(self, timeout: float) -> bool:
        if self._bootstrap_last_status == 2:
            return True
        self._bootstrap_snapshot_event.clear()
        try:
            await asyncio.wait_for(self._bootstrap_snapshot_event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return self._bootstrap_last_status == 2
        return self._bootstrap_last_status == 2

    def _schedule_post_ready_info(self) -> None:
        task = self._post_ready_info_task
        if task is not None and not task.done():
            task.cancel()
        token = self._connection_token
        client = self._client

        async def read_background() -> None:
            if client is None or not client.is_connected:
                return
            try:
                await self._read_device_channel(client)
                await self._read_mfg_info(client)
                if self._connection_coordinator.is_current(token):
                    await self._initialize_battery()
            except asyncio.CancelledError:
                return
            except Exception as exc:
                self._log(f"[BLE_BACKGROUND] post_ready_failed error={exc}")

        self._post_ready_info_task = asyncio.get_running_loop().create_task(
            read_background()
        )

    def _gatt_service_fingerprint(self) -> str:
        entries: list[str] = []
        for service in self._last_service_objects:
            service_uuid = self._normalize_uuid(str(getattr(service, "uuid", "")))
            entries.append(f"s:{service_uuid}")
            for characteristic in getattr(service, "characteristics", ()) or ():
                char_uuid = self._normalize_uuid(
                    str(getattr(characteristic, "uuid", ""))
                )
                entries.append(f"c:{service_uuid}:{char_uuid}")
        import hashlib

        return hashlib.sha256("\n".join(sorted(entries)).encode("ascii")).hexdigest()

    def _trusted_device_info_matches(self, address: str) -> bool:
        trusted = self._gatt_trust_store.get(address)
        info = self._connection_coordinator.device_info
        if trusted is None or info is None:
            return False
        return all(
            trusted.get(key) == value
            for key, value in {
                "sn": info.fields.get("sn", ""),
                "proto": info.proto,
                "boot": info.boot,
                "code": info.code,
                "fingerprint": self._gatt_service_fingerprint(),
            }.items()
        )

    def _record_gatt_trust(self, address: str) -> None:
        info = self._connection_coordinator.device_info
        if info is None:
            return
        self._gatt_trust_store.save(
            address,
            {
                "sn": info.fields.get("sn", ""),
                "proto": info.proto,
                "boot": info.boot,
                "code": info.code,
                "fingerprint": self._gatt_service_fingerprint(),
            },
        )
        self._log(
            f"[BLE_GATT][FAST] trusted_saved=1 address={address} "
            f"sn={info.fields.get('sn', '')} code={info.code}"
        )

    async def _query_bootstrap_connection_state(self) -> bool:
        client = self._client
        if client is None or not client.is_connected or self._command_uuid is None:
            return False
        self._connection_coordinator.transition(ConnectionStage.BOOTSTRAP_QUERY)
        for attempt in range(1, 3):
            seq = self._reserve_control_seq()
            ack_future: asyncio.Future[ZY100Ack] = self._loop.create_future()
            snapshot_future: asyncio.Future[Any] = self._loop.create_future()
            self._bootstrap_query_seq = seq
            self._bootstrap_query_ack_future = ack_future
            self._bootstrap_snapshot_future = snapshot_future
            try:
                await self._write_central_control_frame(
                    build_get_connection_state_command(seq),
                    label="GET_CONNECTION_STATE",
                )
                ack, snapshot = await asyncio.wait_for(
                    asyncio.gather(ack_future, snapshot_future),
                    timeout=BOOTSTRAP_QUERY_TIMEOUT_SECONDS,
                )
                if isinstance(self._connection_coordinator.strategy, BootstrapV4Strategy):
                    expected_boot = 4
                elif isinstance(self._connection_coordinator.strategy, BootstrapV3Strategy):
                    expected_boot = 3
                elif isinstance(self._connection_coordinator.strategy, BootstrapV2Strategy):
                    expected_boot = 2
                else:
                    expected_boot = 1
                if ack.status != STATUS_OK or snapshot.bootstrap_version != expected_boot:
                    raise RuntimeError(
                        f"bootstrap rejected status=0x{ack.status:02X} version={snapshot.bootstrap_version}"
                    )
                self._bootstrap_generation = snapshot.generation
                self._bootstrap_revision = snapshot.revision
                self._bootstrap_last_status = snapshot.overall_status
                self._bootstrap_ready_bits = snapshot.ready_bits
                self._bootstrap_device_state = snapshot.device_state
                self._bootstrap_suggested_action = snapshot.suggested_action
                self._bootstrap_stage = snapshot.stage
                self._bootstrap_reason = snapshot.reason
                self._last_device_state = snapshot.device_state
                self._bootstrap_snapshot_event.set()
                self._emit(
                    {
                        "type": "connection_bootstrap",
                        "status": snapshot.overall_status,
                        "device_state": snapshot.device_state,
                        "suggested_action": snapshot.suggested_action,
                        "bootstrap_version": snapshot.bootstrap_version,
                        "generation": snapshot.generation,
                        "ready_bits": snapshot.ready_bits,
                        "stage": snapshot.stage,
                        "reason": snapshot.reason,
                        "retry_ms": snapshot.retry_ms,
                    }
                )
                return True
            except (asyncio.TimeoutError, RuntimeError) as exc:
                ack_received = (
                    ack_future.done()
                    and not ack_future.cancelled()
                    and ack_future.exception() is None
                )
                snapshot_received = (
                    snapshot_future.done()
                    and not snapshot_future.cancelled()
                    and snapshot_future.exception() is None
                )
                reason = str(exc) or exc.__class__.__name__
                self._log(
                    "[BLE_BOOTSTRAP] query_failed "
                    f"attempt={attempt}/2 ack_received={int(ack_received)} "
                    f"snapshot_received={int(snapshot_received)} reason={reason}"
                )
            finally:
                if self._bootstrap_query_ack_future is ack_future:
                    self._bootstrap_query_ack_future = None
                    self._bootstrap_query_seq = None
                if self._bootstrap_snapshot_future is snapshot_future:
                    self._bootstrap_snapshot_future = None
        return False

    async def _wait_for_security_ready(self) -> bool:
        client = self._client
        if client is None or not client.is_connected or not self._calibration_status_uuid:
            return False
        self._connection_coordinator.transition(ConnectionStage.WAITING_SECURITY)
        deadline = asyncio.get_running_loop().time() + 60.0
        while asyncio.get_running_loop().time() < deadline:
            try:
                await asyncio.wait_for(
                    self._connection_coordinator.read_gatt_char(
                        self._connection_token, client,
                        self._calibration_status_uuid,
                        name="security_probe",
                    ),
                    timeout=max(1.0, deadline - asyncio.get_running_loop().time()),
                )
                return True
            except asyncio.TimeoutError:
                break
            except Exception as exc:
                if not self._is_insufficient_encryption_error(exc):
                    self._log(f"[BLE_SECURITY] protected_read_failed reason={exc}")
                await asyncio.sleep(0.5)
        return False

    @staticmethod
    def _is_insufficient_encryption_error(exc: BaseException) -> bool:
        text = str(exc).lower()
        return (
            "insufficient encryption" in text
            or "0x0f" in text
            or "gatt protocol error: 15" in text
            or text.startswith("(15,")
        )

    async def _reset_calibration_subscriptions(self, client: BleakClient) -> None:
        if self._calibration_tx_subscribed and self._calibration_tx_uuid:
            try:
                await self._connection_coordinator.stop_notify(
                    self._connection_token, client, self._calibration_tx_uuid,
                    name="stop_calibration_tx_notify",
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                self._log(f"[CAL_SECURITY] stop_tx_notify failed: {exc}")
        if self._calibration_status_subscribed and self._calibration_status_uuid:
            try:
                await self._connection_coordinator.stop_notify(
                    self._connection_token, client, self._calibration_status_uuid,
                    name="stop_calibration_status_notify",
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                self._log(f"[CAL_SECURITY] stop_status_notify failed: {exc}")
        self._calibration_tx_subscribed = False
        self._calibration_status_subscribed = False

    async def _initialize_calibration(self, *, allow_record_sync: bool = True) -> bool:
        client = self._client
        available = bool(
            client is not None
            and client.is_connected
            and self._calibration_service_found
            and self._calibration_info_uuid
            and self._calibration_tx_uuid
            and self._calibration_rx_uuid
            and self._calibration_status_uuid
        )
        if not available or client is None:
            self._emit({"type": "calibration_subscription", "status": "unavailable"})
            self._set_calibration_gate(
                "unknown",
                message="Firmware does not support calibration summary confirmation; upgrade required",
            )
            return False

        delays = CAL_SECURITY_RETRY_DELAYS_S
        for attempt in range(1, len(delays) + 2):
            self._emit(
                {
                    "type": "calibration_subscription",
                    "status": "waiting_encryption",
                    "attempt": attempt,
                }
            )
            try:
                raw_status = bytes(await self._connection_coordinator.read_gatt_char(
                    self._connection_token, client, self._calibration_status_uuid,
                    name="read_calibration_status",
                ))
                status = parse_calibration_status(raw_status)
                if not self._calibration_status_subscribed:
                    status_callback = self._connection_coordinator.guard_callback(
                        self._connection_token,
                        self._handle_calibration_status_notification,
                    )
                    await self._connection_coordinator.start_notify(
                        self._connection_token, client,
                        self._calibration_status_uuid, status_callback,
                        name="subscribe_calibration_status",
                    )
                    self._calibration_status_subscribed = True
                if not self._calibration_tx_subscribed:
                    tx_callback = self._connection_coordinator.guard_callback(
                        self._connection_token,
                        self._handle_calibration_tx_notification,
                    )
                    await self._connection_coordinator.start_notify(
                        self._connection_token, client,
                        self._calibration_tx_uuid, tx_callback,
                        name="subscribe_calibration_tx",
                    )
                    self._calibration_tx_subscribed = True
                self._log(
                    f"[CAL_SECURITY] ready attempt={attempt} protected_read=1 subscriptions=1"
                )
                self._emit(
                    {
                        "type": "calibration_subscription",
                        "status": "subscribed",
                        "tx_uuid": self._calibration_tx_uuid,
                        "status_uuid": self._calibration_status_uuid,
                    }
                )
                self._handle_calibration_status(status)
                return await self._confirm_calibration_summary(
                    allow_record_sync=allow_record_sync
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                await self._reset_calibration_subscriptions(client)
                security_error = self._is_insufficient_encryption_error(exc)
                self._log(
                    f"[CAL_SECURITY] attempt={attempt} encrypted=0 retry={int(security_error and attempt <= len(delays))} error={exc}"
                )
                if not security_error or attempt > len(delays):
                    self._emit(
                        {"type": "calibration_subscription", "status": "failed", "error": str(exc)}
                    )
                    return False
                await asyncio.sleep(delays[attempt - 1])
        return False

    def _set_calibration_gate(
        self,
        state: str,
        *,
        summary: dict[str, Any] | None = None,
        message: str = "",
    ) -> None:
        self._calibration_gate_state = state
        self._calibration_summary = deepcopy(summary or {})
        self._emit(
            {
                "type": "calibration_gate",
                "state": state,
                "ready": state in {"uncalibrated", "valid"},
                "message": message,
                "summary": deepcopy(self._calibration_summary),
            }
        )

    def _calibration_session_snapshot(self) -> dict[str, Any]:
        snapshot = deepcopy(self._calibration_summary)
        if "quality" in snapshot:
            snapshot["quality_raw"] = snapshot.pop("quality")
        snapshot["state"] = self._calibration_gate_state
        return snapshot

    @staticmethod
    def _calibration_record_matches_summary(
        record: Any,
        summary: dict[str, Any],
    ) -> bool:
        return bool(
            record is not None
            and summary.get("valid")
            and record.generation == int(summary.get("generation") or 0)
            and record.valid_flags == int(summary.get("valid_flags") or 0)
            and record.record_bytes == int(summary.get("record_bytes") or 0)
            and record.crc32 == (int(summary.get("crc32") or 0) & 0xFFFFFFFF)
        )

    def _is_calibration_business_ready(self) -> bool:
        if self._calibration_gate_state == "uncalibrated":
            return True
        return (
            self._calibration_gate_state == "valid"
            and self._calibration_record_matches_summary(
                self._latest_calibration_record,
                self._calibration_summary,
            )
        )

    def _complete_calibration_record_sync(self, ok: bool) -> None:
        future = self._calibration_record_sync_future
        if future is not None and not future.done():
            future.set_result(ok)

    async def _sync_calibration_record(
        self,
        summary: dict[str, Any],
    ) -> bool:
        client = self._client
        if (
            client is None
            or not client.is_connected
            or not self._is_calibration_ready()
            or self._calibration_rx_uuid is None
            or not summary.get("valid")
        ):
            return False

        self._latest_calibration_record = None
        self._calibration_record_expected_summary = deepcopy(summary)
        for attempt in range(1, CAL_RECORD_SYNC_ATTEMPTS + 1):
            self._set_calibration_gate(
                "record_syncing",
                summary=summary,
                message=f"Downloading complete calibration record ({attempt}/{CAL_RECORD_SYNC_ATTEMPTS})",
            )
            self._calibration_receiver.reset()
            self._calibration_record_ack_transaction_id = None
            self._calibration_record_request_active = True
            future: asyncio.Future[bool] = asyncio.get_running_loop().create_future()
            self._calibration_record_sync_future = future
            self._calibration_transaction_id = (
                (self._calibration_transaction_id + 1) & 0xFF
            ) or 1
            request_transaction_id = self._calibration_transaction_id
            try:
                await self._connection_coordinator.write_gatt_char(
                    self._connection_token, client, self._calibration_rx_uuid,
                    build_calibration_request(request_transaction_id),
                    name="calibration_record_request",
                )
                self._log(
                    f"[CAL_GATE] record_request attempt={attempt} request_transaction={request_transaction_id}"
                )
                ok = await asyncio.wait_for(
                    future,
                    timeout=CAL_RECORD_SYNC_TIMEOUT_SECONDS,
                )
                if ok and self._calibration_record_matches_summary(
                    self._latest_calibration_record,
                    summary,
                ):
                    self._set_calibration_gate(
                        "valid",
                        summary=summary,
                        message="Calibration summary and complete record confirmed",
                    )
                    self._log(
                        "[CAL_GATE] complete_record_confirmed "
                        f"generation={int(summary.get('generation') or 0)} "
                        f"bytes={int(summary.get('record_bytes') or 0)} "
                        f"crc=0x{int(summary.get('crc32') or 0) & 0xFFFFFFFF:08X}"
                    )
                    return True
            except (asyncio.TimeoutError, ValueError) as exc:
                self._log(
                    f"[CAL_GATE] record_sync_failed attempt={attempt} error={exc}"
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                self._log(
                    f"[CAL_GATE] record_sync_failed attempt={attempt} error={exc}"
                )
            finally:
                self._calibration_record_request_active = False
                if self._calibration_record_sync_future is future:
                    self._calibration_record_sync_future = None
        self._latest_calibration_record = None
        self._calibration_record_expected_summary = {}
        self._calibration_record_ack_transaction_id = None
        self._set_calibration_gate(
            "unknown",
            summary=summary,
            message="Complete calibration record synchronization failed; retry or reconnect",
        )
        return False

    async def _confirm_cached_calibration_record(
        self,
        info: Any,
        summary: dict[str, Any],
    ) -> bool:
        client = self._client
        device_info = self._connection_coordinator.device_info
        if (
            client is None
            or not client.is_connected
            or self._calibration_rx_uuid is None
            or device_info is None
            or device_info.fields.get("calcache") != "1"
        ):
            return False
        serial = device_info.fields.get("sn", "")
        record = await asyncio.to_thread(
            self._calibration_store.find_verified_record,
            generation=info.generation,
            crc32=info.crc32,
            record_bytes=info.record_bytes,
            valid_flags=info.valid_flags,
            device_name=self._current_device_name,
            device_address=self._connected_address or "",
            device_serial=serial,
        )
        if not self._calibration_record_matches_summary(record, summary):
            self._log(
                "[CAL_CACHE] miss "
                f"sn={serial or '-'} generation={info.generation} crc=0x{info.crc32:08X}"
            )
            return False
        self._calibration_transaction_id = (
            (self._calibration_transaction_id + 1) & 0xFF
        ) or 1
        transaction_id = self._calibration_transaction_id
        future: asyncio.Future[Any] = asyncio.get_running_loop().create_future()
        self._calibration_cache_confirm_transaction_id = transaction_id
        self._calibration_cache_confirm_future = future
        payload = build_calibration_cached_record_confirmation(
            transaction_id,
            generation=record.generation,
            valid_flags=record.valid_flags,
            record_bytes=record.record_bytes,
            record_version=info.record_version,
            crc32=record.crc32,
        )
        try:
            await self._connection_coordinator.write_gatt_char(
                self._connection_token, client, self._calibration_rx_uuid, payload,
                name="calibration_cache_confirm",
            )
            status = await asyncio.wait_for(future, timeout=1.5)
            if (
                status.code == 31
                and status.transaction_id == transaction_id
                and status.generation == record.generation
                and status.crc32 == record.crc32
            ):
                self._latest_calibration_record = record
                self._set_calibration_gate(
                    "valid",
                    summary=summary,
                    message="Calibration cache confirmed by device",
                )
                self._log(
                    "[CAL_CACHE] confirmed "
                    f"sn={serial or '-'} generation={record.generation} "
                    f"bytes={record.record_bytes} crc=0x{record.crc32:08X}"
                )
                return True
            self._log(f"[CAL_CACHE] rejected status={status.name}")
        except (asyncio.TimeoutError, ValueError) as exc:
            self._log(f"[CAL_CACHE] fallback reason={exc}")
        except Exception as exc:
            self._log(f"[CAL_CACHE] fallback reason={exc}")
        finally:
            if self._calibration_cache_confirm_future is future:
                self._calibration_cache_confirm_future = None
                self._calibration_cache_confirm_transaction_id = None
        return False

    async def _confirm_calibration_summary(self, *, allow_record_sync: bool = True) -> bool:
        client = self._client
        if (
            client is None
            or not client.is_connected
            or self._calibration_info_uuid is None
            or self._calibration_rx_uuid is None
        ):
            self._set_calibration_gate("unknown", message="Calibration GATT is not ready")
            return False
        self._set_calibration_gate("unknown", message="Reading calibration summary")
        for attempt in range(2):
            try:
                raw_info = bytes(
                    await self._connection_coordinator.read_gatt_char(
                        self._connection_token, client, self._calibration_info_uuid,
                        name="read_calibration_info",
                    )
                )
                info = parse_calibration_info(raw_info)
                summary = info.to_dict()
                self._emit({"type": "calibration_info", **summary})
                self._calibration_transaction_id = (
                    (self._calibration_transaction_id + 1) & 0xFF
                )
                if self._calibration_transaction_id == 0:
                    self._calibration_transaction_id = 1
                transaction_id = self._calibration_transaction_id
                future = asyncio.get_running_loop().create_future()
                self._calibration_info_confirm_transaction_id = transaction_id
                self._calibration_info_confirm_future = future
                payload = build_calibration_info_confirmation(transaction_id, raw_info)
                await self._connection_coordinator.write_gatt_char(
                    self._connection_token, client, self._calibration_rx_uuid, payload,
                    name="confirm_calibration_info",
                )
                status = await asyncio.wait_for(future, timeout=1.5)
                if status.code == 28:
                    self._latest_calibration_record = None
                    self._log(
                        f"[CAL_GATE] summary_confirmed valid={int(info.valid)} generation={info.generation} "
                        f"crc=0x{info.crc32:08X}"
                    )
                    if not info.valid:
                        self._calibration_record_expected_summary = {}
                        self._set_calibration_gate(
                            "uncalibrated",
                            summary=summary,
                            message="Uncalibrated state confirmed; default parameters are active",
                        )
                        return True
                    if await self._confirm_cached_calibration_record(info, summary):
                        return True
                    if not allow_record_sync:
                        self._set_calibration_gate(
                            "record_deferred",
                            summary=summary,
                            message="Calibration summary confirmed; full record deferred until offline capture ends",
                        )
                        self._log(
                            "[OFF_OBS][DEFER] step=calibration_record reason=cache_miss"
                        )
                        return True
                    return await self._sync_calibration_record(summary)
                self._log(
                    f"[CAL_GATE] confirm rejected attempt={attempt + 1} status={status.name}"
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                self._log(f"[CAL_GATE] confirm failed attempt={attempt + 1}: {exc}")
            finally:
                self._calibration_info_confirm_transaction_id = None
                self._calibration_info_confirm_future = None
        self._set_calibration_gate(
            "unknown",
            message="Calibration summary confirmation failed; upgrade firmware or reconnect",
        )
        return False

    def start_mag_calibration(self) -> None:
        self._submit(self._start_mag_calibration())

    async def _start_mag_calibration(self) -> None:
        client = self._client
        if self._offline_priority_blocked:
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "offline_transfer_busy",
                    "message": "离线数据上传中，地磁校准已暂停",
                }
            )
            return
        if not self._central_business_ready():
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "central_not_ready",
                    "message": "Host-CI Central mode is not ready",
                }
            )
            return
        if not self._is_calibration_business_ready():
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "gate_blocked",
                    "message": "Calibration summary or complete record is not confirmed",
                }
            )
            return
        if (
            client is None
            or not client.is_connected
            or not self._is_calibration_ready()
            or self._calibration_rx_uuid is None
        ):
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "unavailable",
                    "message": "Calibration GATT is not ready",
                }
            )
            return
        if self._calibration_command_pending or self._calibration_active:
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "busy",
                    "message": "Magnetometer calibration is already active",
                }
            )
            return
        if (
            not self._connection_business_ready
            or self._pending_command_active
            or self._feature_config_pending
            or self._ota_entry_operation_active
        ):
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "busy",
                    "message": "设备存在其他独占操作，地磁校准仅允许在 Business Ready / WAIT_START 空闲态启动",
                }
            )
            return
        if self._last_device_state != DEVICE_STATE_WAIT_START:
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "invalid_state",
                    "message": "Device is not in WAIT_START idle state",
                }
            )
            return
        self._calibration_transaction_id = (self._calibration_transaction_id + 1) & 0xFF
        if self._calibration_transaction_id == 0:
            self._calibration_transaction_id = 1
        transaction_id = self._calibration_transaction_id
        payload = build_start_mag_calibration(transaction_id)
        self._calibration_command_pending = True
        self._calibration_receiver.reset()
        self._calibration_diag_request_transaction_id = None
        self._calibration_diag_request_pending = False
        self._calibration_diag_auto_pending = False
        self._calibration_diag_retry_used = False
        self._calibration_diag_generation = 0
        self._calibration_diag_record_crc32 = 0
        try:
            await self._connection_coordinator.write_gatt_char(
                self._connection_token, client, self._calibration_rx_uuid, payload,
                name="start_mag_calibration",
            )
            self._log(f"[CAL_CMD] START_MAG_CAL write_ok transaction={transaction_id}")
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "write_ok",
                    "transaction_id": transaction_id,
                    "message": "Command received by ATT; waiting for CAL_ACCEPTED",
                }
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._calibration_command_pending = False
            self._log(f"[CAL_CMD] START_MAG_CAL write_failed transaction={transaction_id} error={exc}")
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "failed",
                    "transaction_id": transaction_id,
                    "message": str(exc),
                }
            )

    def request_calibration_record(self) -> None:
        self._submit(self._resync_calibration_record())

    async def _resync_calibration_record(self) -> None:
        if self._offline_priority_blocked:
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "offline_priority_busy",
                    "message": "离线采集或自动上传进行中，校准记录同步已暂停",
                }
            )
            return
        summary = deepcopy(self._calibration_summary)
        if not summary.get("valid"):
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "unavailable",
                    "message": "Device has no complete calibration record to synchronize",
                }
            )
            return
        ok = await self._sync_calibration_record(summary)
        self._emit(
            {
                "type": "calibration_command",
                "status": "record_synced" if ok else "failed",
                "message": (
                    "Complete calibration record synchronized"
                    if ok
                    else "Complete calibration record synchronization failed"
                ),
            }
        )

    async def _request_calibration_record(self) -> None:
        client = self._client
        if client is None or not client.is_connected or not self._is_calibration_ready() or not self._calibration_rx_uuid:
            self._emit({"type": "calibration_command", "status": "unavailable", "message": "Calibration GATT is not ready"})
            return
        self._calibration_transaction_id = (self._calibration_transaction_id + 1) & 0xFF or 1
        transaction_id = self._calibration_transaction_id
        self._calibration_receiver.reset()
        try:
            await self._connection_coordinator.write_gatt_char(
                self._connection_token, client, self._calibration_rx_uuid,
                build_calibration_request(transaction_id),
                name="request_calibration_record",
            )
            self._emit(
                {
                    "type": "calibration_command",
                    "status": "request_sent",
                    "transaction_id": transaction_id,
                    "message": "Calibration record requested",
                }
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._emit({"type": "calibration_command", "status": "failed", "message": str(exc)})

    def request_mag_calibration_diagnostics(self) -> None:
        self._submit(self._request_mag_calibration_diagnostics(auto=False))

    async def _request_mag_calibration_diagnostics(
        self,
        *,
        auto: bool,
        retry: bool = False,
    ) -> None:
        if self._offline_priority_blocked:
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "offline_priority_busy",
                    "message": "离线采集或自动上传进行中，地磁诊断已暂停",
                }
            )
            return
        client = self._client
        if (
            client is None
            or not client.is_connected
            or not self._is_calibration_ready()
            or not self._calibration_rx_uuid
        ):
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "unavailable",
                    "message": "Calibration GATT is not ready",
                }
            )
            return
        if self._calibration_active:
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "busy",
                    "message": "Calibration is still active",
                }
            )
            return
        if self._calibration_diag_request_pending and not retry:
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "busy",
                    "message": "Diagnostics request is already pending",
                }
            )
            return
        if not retry:
            self._calibration_transaction_id = (
                (self._calibration_transaction_id + 1) & 0xFF
            ) or 1
            self._calibration_diag_request_transaction_id = (
                self._calibration_transaction_id
            )
            self._calibration_diag_retry_used = False
        transaction_id = self._calibration_diag_request_transaction_id
        if transaction_id is None:
            return
        self._calibration_receiver.reset()
        self._calibration_diag_request_pending = True
        try:
            await self._connection_coordinator.write_gatt_char(
                self._connection_token, client, self._calibration_rx_uuid,
                build_calibration_diagnostics_request(transaction_id),
                name="request_calibration_diagnostics",
            )
            self._log(
                f"[CAL_DIAG_CMD] REQUEST_DIAGNOSTICS write_ok "
                f"transaction={transaction_id} auto={int(auto)} retry={int(retry)}"
            )
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "request_sent",
                    "transaction_id": transaction_id,
                    "auto": auto,
                    "message": "Mag calibration diagnostics requested",
                }
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._calibration_diag_request_pending = False
            self._log(
                f"[CAL_DIAG_CMD] write_failed transaction={transaction_id} error={exc}"
            )
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "failed",
                    "transaction_id": transaction_id,
                    "message": str(exc),
                }
            )

    async def _retry_mag_calibration_diagnostics(self) -> None:
        await asyncio.sleep(0.5)
        if self._calibration_diag_request_transaction_id is not None:
            await self._request_mag_calibration_diagnostics(auto=True, retry=True)

    def _is_calibration_ready(self) -> bool:
        return bool(
            self._calibration_service_found
            and self._calibration_info_uuid
            and self._calibration_tx_uuid
            and self._calibration_rx_uuid
            and self._calibration_status_uuid
            and self._calibration_tx_subscribed
            and self._calibration_status_subscribed
        )

    def _handle_calibration_status_notification(self, sender: Any, data: bytearray) -> None:
        try:
            status = parse_calibration_status(bytes(data))
        except ValueError as exc:
            self._log(f"[CAL_STATUS] invalid error={exc} raw={bytes(data).hex(' ').upper()}")
            self._emit({"type": "calibration_error", "message": str(exc)})
            return
        self._handle_calibration_status(status)

    def _handle_calibration_status(self, status: Any) -> None:
        if (
            self._calibration_cache_confirm_future is not None
            and not self._calibration_cache_confirm_future.done()
            and status.transaction_id
            == self._calibration_cache_confirm_transaction_id
            and status.code in {8, 9, 11, 30, 31}
        ):
            self._calibration_cache_confirm_future.set_result(status)
        if (
            self._calibration_info_confirm_future is not None
            and not self._calibration_info_confirm_future.done()
            and status.transaction_id
            == self._calibration_info_confirm_transaction_id
            and status.code in {8, 11, 28, 29, 30}
        ):
            self._calibration_info_confirm_future.set_result(status)
        if self._calibration_record_request_active:
            if (
                status.code == 4
                and self._calibration_record_ack_transaction_id is not None
                and status.transaction_id
                == self._calibration_record_ack_transaction_id
            ):
                self._complete_calibration_record_sync(
                    self._calibration_record_matches_summary(
                        self._latest_calibration_record,
                        self._calibration_record_expected_summary,
                    )
                )
            elif status.code in {0, 8, 11, 13, 30}:
                self._complete_calibration_record_sync(False)
        if status.transaction_id == self._calibration_transaction_id:
            self._calibration_command_pending = False
        if status.code in CAL_ACTIVE_STATUSES:
            self._calibration_active = True
        else:
            self._calibration_active = False
        self._log(
            f"[CAL_STATUS] name={status.name} transaction={status.transaction_id} "
            f"detail={status.detail} generation={status.generation} crc=0x{status.crc32:08X}"
        )
        is_diag_status = (
            self._calibration_diag_request_transaction_id is not None
            and status.transaction_id
            == self._calibration_diag_request_transaction_id
        )
        if is_diag_status and status.code == 10:
            if not self._calibration_diag_retry_used:
                self._calibration_diag_retry_used = True
                self._calibration_diag_request_pending = False
                self._emit(
                    {
                        "type": "calibration_diagnostics_command",
                        "status": "retrying",
                        "message": "Device busy; retrying diagnostics once",
                    }
                )
                self._loop.create_task(
                    self._retry_mag_calibration_diagnostics()
                )
            else:
                self._calibration_diag_request_pending = False
                self._emit(
                    {
                        "type": "calibration_diagnostics_command",
                        "status": "busy",
                        "message": "Device is still busy; retry manually",
                    }
                )
            return
        if is_diag_status and status.code == 8:
            self._calibration_diag_request_pending = False
            self._calibration_diag_auto_pending = False
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "unsupported",
                    "message": "Current firmware does not support diagnostics summary",
                }
            )
            return
        if is_diag_status and status.code in {0, 11, 13, 25}:
            self._calibration_diag_request_pending = False
            messages = {
                0: "Device has no diagnostics summary; run calibration first",
                11: "Encrypted paired connection is required for diagnostics",
                13: "Device failed to transmit diagnostics",
                25: "Device state does not allow diagnostics request",
            }
            self._emit(
                {
                    "type": "calibration_diagnostics_command",
                    "status": "unavailable" if status.code == 0 else "failed",
                    "message": messages[status.code],
                }
            )
            return
        self._emit({"type": "calibration_status", **status.to_dict()})
        if status.code in {19, 26}:
            self._set_calibration_gate(
                "unknown", message="Calibration parameters changed; confirming new summary"
            )
            self._calibration_diag_generation = status.generation
            self._calibration_diag_record_crc32 = status.crc32
            self._loop.create_task(self._reconfirm_after_calibration())
        elif status.code == 27:
            self._calibration_diag_generation = status.generation
            self._calibration_diag_record_crc32 = status.crc32
            self._loop.create_task(
                self._request_mag_calibration_diagnostics(auto=True)
            )
        elif status.code == 7:
            self._set_calibration_gate(
                "unknown", message="Calibration record changed; confirming new summary"
            )
            self._loop.create_task(self._confirm_calibration_summary())
        elif status.code in {20, 21, 22, 23}:
            self._calibration_diag_auto_pending = False
            self._calibration_diag_generation = status.generation
            self._calibration_diag_record_crc32 = status.crc32
            self._loop.create_task(
                self._request_mag_calibration_diagnostics(auto=True)
            )

    async def _reconfirm_after_calibration(self) -> None:
        confirmed = await self._confirm_calibration_summary()
        if confirmed:
            await self._request_mag_calibration_diagnostics(auto=True)

    def _handle_calibration_tx_notification(self, sender: Any, data: bytearray) -> None:
        try:
            event = self._calibration_receiver.on_notify(bytes(data))
        except ValueError as exc:
            self._calibration_receiver.reset()
            self._log(f"[CAL_TX] invalid error={exc} raw={bytes(data).hex(' ').upper()}")
            self._emit({"type": "calibration_error", "message": str(exc)})
            if self._calibration_record_request_active:
                self._complete_calibration_record_sync(False)
            return
        self._emit(
            {
                "type": "calibration_transfer",
                "status": event.kind,
                "transaction_id": event.transaction_id,
                "received_bytes": event.received_bytes,
                "total_bytes": event.total_bytes,
                "transfer_kind": event.transfer_kind,
            }
        )
        if event.kind == "complete" and event.record is not None:
            self._loop.create_task(
                self._persist_and_ack_calibration(event.transaction_id, event.record)
            )
        elif event.kind == "complete" and event.diagnostics is not None:
            self._loop.create_task(
                self._persist_and_ack_mag_diagnostics(
                    event.transaction_id, event.diagnostics
                )
            )

    async def _persist_and_ack_calibration(self, transaction_id: int, record: Any) -> None:
        client = self._client
        if self._calibration_record_request_active and not self._calibration_record_matches_summary(
            record,
            self._calibration_record_expected_summary,
        ):
            self._log(
                "[CAL_GATE] record_mismatch "
                f"transaction={transaction_id} generation={record.generation} "
                f"bytes={record.record_bytes} crc=0x{record.crc32:08X}"
            )
            self._emit(
                {
                    "type": "calibration_error",
                    "message": "Complete calibration record does not match the confirmed summary",
                }
            )
            self._complete_calibration_record_sync(False)
            return
        try:
            device_info = self._connection_coordinator.device_info
            saved = await asyncio.to_thread(
                self._calibration_store.save_verified_record,
                record,
                device_name=self._current_device_name,
                device_address=self._connected_address or "",
                device_serial=(device_info.fields.get("sn", "") if device_info else ""),
            )
        except Exception as exc:
            self._log(f"[CAL_STORE] failed generation={record.generation} error={exc}")
            self._emit({"type": "calibration_error", "message": f"Calibration save failed: {exc}"})
            if self._calibration_record_request_active:
                self._complete_calibration_record_sync(False)
            return
        self._latest_calibration_record = record
        if client is None or not client.is_connected or not self._calibration_rx_uuid:
            self._emit(
                {
                    "type": "calibration_record",
                    "transaction_id": transaction_id,
                    **record.to_dict(),
                    "saved_path": str(saved.binary_path),
                    "ack_status": "not_connected",
                }
            )
            return
        try:
            if self._calibration_record_request_active:
                self._calibration_record_ack_transaction_id = transaction_id
            await self._connection_coordinator.write_gatt_char(
                self._connection_token, client, self._calibration_rx_uuid,
                build_calibration_ack(transaction_id, record.crc32),
                name="calibration_record_ack",
            )
            ack_status = "sent"
            self._log(
                f"[CAL_STORE] saved_ack generation={record.generation} crc=0x{record.crc32:08X} "
                f"path={saved.binary_path}"
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            ack_status = f"failed: {exc}"
            self._log(f"[CAL_ACK] failed transaction={transaction_id} error={exc}")
        self._emit(
            {
                "type": "calibration_record",
                "transaction_id": transaction_id,
                **record.to_dict(),
                "saved_path": str(saved.binary_path),
                "metadata_path": str(saved.metadata_path),
                "ack_status": ack_status,
            }
        )

    async def _persist_and_ack_mag_diagnostics(
        self, transaction_id: int, diagnostics: Any
    ) -> None:
        client = self._client
        try:
            saved = await asyncio.to_thread(
                self._calibration_store.save_verified_diagnostics,
                diagnostics,
                request_transaction_id=transaction_id,
                device_name=self._current_device_name,
                device_address=self._connected_address or "",
                generation=self._calibration_diag_generation,
                calibration_crc32=self._calibration_diag_record_crc32,
            )
        except Exception as exc:
            self._calibration_diag_request_pending = False
            self._log(
                f"[CAL_DIAG_STORE] failed transaction={transaction_id} error={exc}"
            )
            self._emit(
                {
                    "type": "calibration_diagnostics_error",
                    "message": f"Diagnostics save failed: {exc}",
                }
            )
            return
        ack_status = "not_connected"
        if client is not None and client.is_connected and self._calibration_rx_uuid:
            try:
                await self._connection_coordinator.write_gatt_char(
                    self._connection_token, client, self._calibration_rx_uuid,
                    build_calibration_ack(transaction_id, diagnostics.crc32),
                    name="calibration_diagnostics_ack",
                )
                ack_status = "sent"
                self._log(
                    f"[CAL_DIAG_STORE] saved_ack transaction={transaction_id} "
                    f"cal_transaction={diagnostics.transaction_id} "
                    f"crc=0x{diagnostics.crc32:08X} path={saved.binary_path}"
                )
            except Exception as exc:  # pragma: no cover - host stack dependent
                ack_status = f"failed: {exc}"
                self._log(
                    f"[CAL_DIAG_ACK] failed transaction={transaction_id} error={exc}"
                )
        self._calibration_diag_request_pending = False
        self._calibration_diag_auto_pending = False
        self._emit(
            {
                "type": "calibration_diagnostics",
                "request_transaction_id": transaction_id,
                **diagnostics.to_dict(),
                "saved_path": str(saved.binary_path),
                "metadata_path": str(saved.metadata_path),
                "ack_status": ack_status,
            }
        )

    def probe_stress(self, user_id: int = 1) -> None:
        self._submit(self.stress.probe(user_id))

    def start_stress(self, rate: int, duration: int, user_id: int, training_id: int,
                     firmware: str, host_version: str) -> None:
        self._submit(self.stress.start(rate, duration, user_id, training_id, firmware, host_version))

    def stop_stress(self) -> None:
        self._submit(self.stress.stop())

    def send_zy100_command(
        self,
        cmd: int,
        user_id: int = 1,
        training_id: int = 1,
        device_time_ms: int | None = None,
    ) -> None:
        self._submit(
            self._send_zy100_command(
                cmd=cmd,
                user_id=user_id,
                training_id=training_id,
                device_time_ms=device_time_ms,
            )
        )

    def enter_shipping_mode(self, user_id: int = 1) -> None:
        """Request the ordinary Production Control shipping transaction."""
        self.send_zy100_command(
            cmd=CMD_ENTER_SHIPPING,
            user_id=user_id,
            training_id=0,
            device_time_ms=None,
        )

    def enter_ota_dfu_mode(self) -> None:
        self._submit(self._enter_ota_dfu_mode())

    def prepare_export_high_speed(self, reason: str = "manual_export") -> None:
        self._submit(self._enter_export_high_speed(reason))

    @staticmethod
    def _char_handle(char: Any) -> int | None:
        handle = getattr(char, "handle", None)
        return handle if isinstance(handle, int) else None

    @staticmethod
    def _char_has_property(char: Any, *properties: str) -> bool:
        char_props = {str(prop).lower() for prop in getattr(char, "properties", []) or []}
        return any(prop.lower() in char_props for prop in properties)

    def _find_ota_dfu_entry_target(self, client: BleakClient) -> OtaDfuEntryTarget | None:
        v5_uuid = LEGACY_SIMPLE_CHARACTERISTIC_UUIDS["v5_write_notify"]
        v5_char = self._find_characteristic(client, v5_uuid)
        if (
            v5_char is not None
            and self._char_has_property(v5_char, "write", "write-without-response")
            and self._char_has_property(v5_char, "notify")
        ):
            return OtaDfuEntryTarget(
                write_uuid=v5_char.uuid.lower(),
                notify_uuid=v5_char.uuid.lower(),
                label="Simple BLE V5 B005 write+notify",
                write_handle=self._char_handle(v5_char),
                notify_handle=self._char_handle(v5_char),
            )

        v2_uuid = LEGACY_SIMPLE_CHARACTERISTIC_UUIDS["v2_write"]
        v3_uuid = LEGACY_SIMPLE_CHARACTERISTIC_UUIDS["v3_notify"]
        v2_char = self._find_characteristic(client, v2_uuid)
        v3_char = self._find_characteristic(client, v3_uuid)
        if (
            v2_char is not None
            and v3_char is not None
            and self._char_has_property(v2_char, "write", "write-without-response")
            and self._char_has_property(v3_char, "notify")
        ):
            return OtaDfuEntryTarget(
                write_uuid=v2_char.uuid.lower(),
                notify_uuid=v3_char.uuid.lower(),
                label="Simple BLE V2 B002 write + V3 B003 notify",
                write_handle=self._char_handle(v2_char),
                notify_handle=self._char_handle(v3_char),
            )

        return None

    async def _subscribe_ota_dfu_entry_notify(
        self, client: BleakClient, target: OtaDfuEntryTarget
    ) -> bool:
        if (
            self._ota_dfu_entry_notify_subscribed
            and self._ota_dfu_entry_notify_uuid == target.notify_uuid
        ):
            return True

        if self._ota_dfu_entry_notify_subscribed:
            await self._stop_ota_dfu_entry_notify(client)

        try:
            callback = self._connection_coordinator.guard_callback(
                self._connection_token, self._handle_ota_dfu_entry_notification
            )
            await self._connection_coordinator.start_notify(
                self._connection_token, client, target.notify_uuid, callback,
                name="subscribe_ota_entry",
            )
            self._ota_dfu_entry_notify_uuid = target.notify_uuid
            self._ota_dfu_entry_notify_subscribed = True
            handle = f"0x{target.notify_handle:04X}" if target.notify_handle is not None else "<unknown>"
            self._log(
                "[BLE_OTA_ENTRY] "
                f"notify_subscribed=1 target={target.label} uuid={target.notify_uuid} handle={handle}"
            )
            return True
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._ota_dfu_entry_notify_uuid = None
            self._ota_dfu_entry_notify_subscribed = False
            self._log(f"[BLE_OTA_ENTRY][ERR] subscribe notify failed: {exc}")
            return False

    async def _stop_ota_dfu_entry_notify(self, client: BleakClient) -> None:
        notify_uuid = self._ota_dfu_entry_notify_uuid
        subscribed = self._ota_dfu_entry_notify_subscribed
        self._ota_dfu_entry_notify_uuid = None
        self._ota_dfu_entry_notify_subscribed = False
        if not subscribed or not notify_uuid:
            return
        try:
            await self._connection_coordinator.stop_notify(
                self._connection_token, client, notify_uuid,
                name="stop_ota_entry_notify",
            )
            self._log(f"[BLE_OTA_ENTRY] notify_stopped uuid={notify_uuid}")
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_OTA_ENTRY][WARN] stop notify failed uuid={notify_uuid}: {exc}")

    def _ota_active_link_is_ready(self, client: BleakClient) -> bool:
        ci, latency = self._central_profiles.read_actual_units(client)
        return ci == 12 and latency == 0

    async def _wait_ota_active_link(self, client: BleakClient) -> bool:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self._ota_active_link_timeout_s
        while self._client is client and client.is_connected:
            if self._ota_active_link_is_ready(client):
                ci, latency = self._central_profiles.read_actual_units(client)
                self._log(
                    "[BLE_OTA_CI] high_speed_ready=1 "
                    f"ci={ci} latency={latency}"
                )
                return True
            if loop.time() >= deadline:
                break
            await asyncio.sleep(OTA_ACTIVE_LINK_POLL_SECONDS)
        ci, latency = self._central_profiles.read_actual_units(client)
        self._log(
            "[BLE_OTA_CI][ERR] high_speed_ready=0 "
            f"ci={ci} latency={latency} timeout_s={self._ota_active_link_timeout_s:.1f}"
        )
        return False

    async def _request_ota_link_intent(
        self,
        client: BleakClient,
    ) -> tuple[str, str]:
        if self._command_uuid is None:
            return "legacy", "ZY100 Control Service unavailable"
        if (
            self._ota_link_intent_ack_future is not None
            and not self._ota_link_intent_ack_future.done()
        ):
            self._ota_link_intent_ack_future.cancel()
        seq = self._reserve_control_seq()
        session_id = self._central_session_id or self._active_user_id or 1
        payload = build_ota_link_intent_command(seq, session_id)
        future: asyncio.Future[ZY100Ack] = asyncio.get_running_loop().create_future()
        self._ota_link_intent_seq = seq
        self._ota_link_intent_ack_future = future
        try:
            await self._write_central_control_frame(payload, label="OTA_LINK_INTENT")
            self._log(
                "[BLE_OTA_INTENT] request_sent "
                f"seq={seq} session={session_id} "
                f"central_mode={int(self._central_mode_enabled)} payload={hex_bytes(payload)}"
            )
            ack = await asyncio.wait_for(
                asyncio.shield(future), timeout=self._ota_link_intent_ack_timeout_s
            )
        except asyncio.TimeoutError:
            self._log(
                "[BLE_OTA_INTENT][ERR] ack_timeout=1 "
                f"seq={seq} timeout_s={self._ota_link_intent_ack_timeout_s:.1f}"
            )
            return "failed_disconnect", "OTA_LINK_INTENT ACK timeout"
        except Exception as exc:
            self._log(f"[BLE_OTA_INTENT][ERR] request_failed error={exc}")
            return "failed_disconnect", f"OTA_LINK_INTENT write failed: {exc}"
        finally:
            if self._ota_link_intent_ack_future is future:
                self._ota_link_intent_ack_future = None
                self._ota_link_intent_seq = None

        if ack.status == STATUS_UNSUPPORTED_CMD:
            ci, latency = self._central_profiles.read_actual_units(client)
            if latency == 0 and (ci == 12 or (ci is not None and 24 <= ci <= 48)):
                self._log(
                    "[BLE_OTA_INTENT] unsupported=1 legacy_active=1 "
                    f"ci={ci} latency={latency}"
                )
                return "legacy_active", "Old firmware Active link compatibility"
            self._log(
                "[BLE_OTA_INTENT][ERR] unsupported=1 legacy_standby=1 "
                f"ci={ci} latency={latency}"
            )
            return (
                "failed",
                "旧固件处于Standby，请先短按唤醒到Active后再升级",
            )
        if ack.status != STATUS_OK:
            message = (
                "OTA_LINK_INTENT rejected by device: "
                f"status=0x{ack.status:02X} state=0x{ack.device_state:02X} "
                f"detail=0x{ack.detail:08X}"
            )
            self._log(f"[BLE_OTA_INTENT][ERR] {message}")
            return "failed", message
        if ack.detail != OTA_LINK_INTENT_WINDOW_MS:
            message = f"OTA_LINK_INTENT invalid ACK detail={ack.detail}"
            self._log(f"[BLE_OTA_INTENT][ERR] {message}")
            return "failed_disconnect", message
        self._log(
            "[BLE_OTA_INTENT] accepted=1 "
            f"seq={seq} window_ms={ack.detail}"
        )
        return "ready", "OTA_LINK_INTENT accepted"

    async def _request_ota_high_speed_link(
        self,
        client: BleakClient,
    ) -> tuple[bool, str]:
        task = self._central_profile_task
        if task is not None and not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self._central_profile_task = None
        self._central_profile_key = None

        self._ota_high_speed_counter = (self._ota_high_speed_counter + 1) & 0xFFFFFFFF
        if self._ota_high_speed_counter == 0:
            self._ota_high_speed_counter = 1
        operation_key = (
            OTA_HIGH_SPEED_OPERATION_TAG,
            self._ota_high_speed_counter,
            0,
            int(WindowsCentralProfile.THROUGHPUT_OPTIMIZED),
        )
        self._ota_high_speed_key = operation_key
        self._log(
            "[BLE_OTA_CI] request profile=THROUGHPUT_OPTIMIZED "
            f"operation={operation_key[1]} central_mode={int(self._central_mode_enabled)}"
        )
        result = await self._central_profiles.activate(
            client,
            WindowsCentralProfile.THROUGHPUT_OPTIMIZED,
            operation_key=operation_key,
        )
        if not result.accepted or not result.actual_stable:
            await self._central_profiles.release_if_owned(
                operation_key=operation_key,
                reason="ota_high_speed_failed",
            )
            if self._ota_high_speed_key == operation_key:
                self._ota_high_speed_key = None
            message = (
                "上位机高速连接参数请求失败："
                f"reason={result.reason}, ci={result.actual_ci}, latency={result.actual_latency}"
            )
            self._log(
                "[BLE_OTA_CI][ERR] request_failed "
                f"accepted={int(result.accepted)} stable={int(result.actual_stable)} "
                f"reason={result.reason} ci={result.actual_ci} latency={result.actual_latency} "
                "ota_command_sent=0"
            )
            return False, message
        self._log(
            "[BLE_OTA_CI] actual_stable profile=THROUGHPUT_OPTIMIZED "
            f"ci={result.actual_ci} latency={result.actual_latency} "
            f"operation={operation_key[1]}"
        )
        return True, "上位机高速连接已稳定：CI=12，latency=0"

    async def _release_ota_high_speed_link(self, reason: str) -> None:
        operation_key = self._ota_high_speed_key
        if operation_key is None:
            return
        await self._central_profiles.release_if_owned(
            operation_key=operation_key,
            reason=reason,
        )
        if self._ota_high_speed_key == operation_key:
            self._ota_high_speed_key = None

    async def _prepare_ota_active_link(self, client: BleakClient) -> tuple[str, str]:
        if self._command_uuid is None:
            self._log(
                "[BLE_OTA_PREP] skipped=control_service_unavailable "
                f"central_mode={int(self._central_mode_enabled)}"
            )
            return "legacy", "ZY100 Control Service unavailable"

        if self._ota_prepare_ack_future is not None and not self._ota_prepare_ack_future.done():
            self._ota_prepare_ack_future.cancel()
        seq = self._reserve_control_seq()
        session_id = self._central_session_id or self._active_user_id or 1
        payload = build_ota_prepare_command(seq, session_id)
        future: asyncio.Future[ZY100Ack] = asyncio.get_running_loop().create_future()
        self._ota_prepare_seq = seq
        self._ota_prepare_ack_future = future
        try:
            await self._write_central_control_frame(payload, label="OTA_PREPARE")
            self._log(
                "[BLE_OTA_PREP] request_sent "
                f"seq={seq} session={session_id} "
                f"central_mode={int(self._central_mode_enabled)} payload={hex_bytes(payload)}"
            )
            ack = await asyncio.wait_for(
                asyncio.shield(future), timeout=self._ota_prepare_ack_timeout_s
            )
        except asyncio.TimeoutError:
            self._log(
                "[BLE_OTA_PREP][ERR] ack_timeout=1 fallback=0 "
                f"seq={seq} timeout_s={self._ota_prepare_ack_timeout_s:.1f}"
            )
            return "failed", "OTA_PREPARE ACK timeout; legacy fallback is not allowed"
        except Exception as exc:
            self._log(f"[BLE_OTA_PREP][ERR] request_failed error={exc}")
            return "failed", f"OTA_PREPARE write failed: {exc}"
        finally:
            if self._ota_prepare_ack_future is future:
                self._ota_prepare_ack_future = None
                self._ota_prepare_seq = None

        if ack.status == STATUS_UNSUPPORTED_CMD:
            self._log("[BLE_OTA_PREP] unsupported=1 fallback=legacy")
            return "legacy", "Firmware does not support OTA_PREPARE"
        if ack.status != STATUS_OK:
            message = (
                "OTA_PREPARE rejected by device: "
                f"status=0x{ack.status:02X} state=0x{ack.device_state:02X}"
            )
            self._log(f"[BLE_OTA_PREP][ERR] {message}")
            return "failed", message
        if not await self._wait_ota_active_link(client):
            return "failed", "OTA_PREPARE accepted, but the Host high-speed link was lost"
        return "ready", "Host high-speed link ready"

    async def _write_ota_commit_once(
        self, client: BleakClient, token: int, payload: bytes,
    ) -> None:
        # COMMIT can reboot before ATT completes. Never resend it through the
        # generic write-without-response fallback after a native write error.
        async with self._control_command_lock:
            if (self._client is not client or self._connection_token != token
                    or not self._connection_coordinator.is_current(token)
                    or not client.is_connected or self._command_uuid is None):
                raise RuntimeError("OTA_COMMIT connection changed before write")
            command_uuid = self._command_uuid
            await self._connection_coordinator.write_gatt_char(
                token, client, command_uuid, payload, response=True,
                timeout=self._ota_commit_ack_timeout_s,
                priority=GattPriority.CRITICAL, name="OTA_COMMIT",
            )

    async def _commit_ota_entry(self, client: BleakClient) -> tuple[str, str]:
        if self._command_uuid is None:
            return "failed", "ZY100 Control Service disappeared before OTA_COMMIT"

        if self._ota_commit_ack_future is not None and not self._ota_commit_ack_future.done():
            self._ota_commit_ack_future.cancel()
        seq = self._reserve_control_seq()
        session_id = self._central_session_id or self._active_user_id or 1
        payload = build_ota_commit_command(seq, session_id)
        future: asyncio.Future[ZY100Ack] = asyncio.get_running_loop().create_future()
        self._ota_commit_seq = seq
        self._ota_commit_ack_future = future
        connection = (client, self._connection_token)
        self._ota_commit_connection = connection
        self._ota_commit_accepted_connection = None
        deadline = asyncio.get_running_loop().time() + self._ota_commit_ack_timeout_s
        try:
            self._log(
                "[BLE_OTA_COMMIT] request_start "
                f"seq={seq} session={session_id} payload={hex_bytes(payload)}"
            )
            try:
                await asyncio.wait_for(
                    self._write_ota_commit_once(client, connection[1], payload),
                    timeout=self._ota_commit_ack_timeout_s,
                )
                if not future.done():
                    await asyncio.wait_for(
                        asyncio.shield(future),
                        timeout=max(0.0, deadline - asyncio.get_running_loop().time()),
                    )
            except asyncio.CancelledError:
                # Queue invalidation cancels its waiter on disconnect. Preserve
                # an ACK already delivered to this transaction's local future;
                # explicit cancellation of the OTA task must still propagate.
                task = asyncio.current_task()
                if task is not None and task.cancelling():
                    raise
                if not future.done() or future.cancelled():
                    return "failed", "OTA_COMMIT 期间连接中断，未收到匹配 ACK；升级结果未确认"
                self._log(f"[BLE_OTA_COMMIT] ack_preserved=1 write_cancelled=1 seq={seq}")
            except Exception as exc:
                if not future.done() or future.cancelled():
                    self._log(f"[BLE_OTA_COMMIT][ERR] request_failed fallback=0 seq={seq} error={type(exc).__name__}: {exc}")
                    return "failed", "OTA_COMMIT 写入或 ACK 超时/失败，未确认进入 DFU；请检查连接后重试"
                self._log(f"[BLE_OTA_COMMIT] ack_preserved=1 write_error={type(exc).__name__} seq={seq}")
            ack = future.result()
        finally:
            if not future.done():
                future.cancel()
            if self._ota_commit_ack_future is future:
                self._ota_commit_ack_future = None
                self._ota_commit_seq = None
            if self._ota_commit_connection == connection:
                self._ota_commit_connection = None

        if ack.status == STATUS_UNSUPPORTED_CMD:
            self._log("[BLE_OTA_COMMIT] unsupported=1 fallback=legacy")
            return "legacy", "Firmware does not support OTA_COMMIT; using legacy OTA entry"
        if ack.status != STATUS_OK:
            message = (
                "OTA_COMMIT rejected by device: "
                f"status=0x{ack.status:02X} state=0x{ack.device_state:02X} "
                f"detail=0x{ack.detail:08X}"
            )
            self._log(f"[BLE_OTA_COMMIT][ERR] {message} fallback=0")
            return "failed", message

        self._log(
            "[BLE_OTA_COMMIT] accepted=1 normal_disconnect_expected=1 "
            f"seq={seq}"
        )
        return "committed", "OTA_COMMIT accepted; device is switching to DFU"

    async def _settle_ota_notify_subscription(self, client: BleakClient) -> None:
        ci, _latency = self._central_profiles.read_actual_units(client)
        if ci is None:
            delay_s = self._ota_notify_settle_min_s
        else:
            delay_s = ci * 1.25 / 1000.0 * OTA_NOTIFY_SETTLE_EVENT_COUNT
            delay_s = max(self._ota_notify_settle_min_s, delay_s)
        delay_s = min(self._ota_notify_settle_max_s, delay_s)
        if delay_s > 0:
            self._log(
                "[BLE_OTA_ENTRY] notify_settle "
                f"ci={ci} events={OTA_NOTIFY_SETTLE_EVENT_COUNT} delay_ms={delay_s * 1000:.0f}"
            )
            await asyncio.sleep(delay_s)

    async def _enter_ota_dfu_mode(self) -> None:
        if self._offline_priority_blocked:
            self._emit(
                {
                    "type": "error",
                    "message": "离线数据上传中，OTA 已暂停",
                }
            )
            return
        if self._ota_entry_operation_active:
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ready_for_dfu_scan": False,
                    "message": "OTA operation is already active",
                }
            )
            return
        self._ota_entry_operation_active = True
        recovery_timer_paused = self._recovery_only
        if recovery_timer_paused:
            self._cancel_recovery_idle_timer()
        try:
            await self._enter_ota_dfu_mode_once()
        except asyncio.CancelledError:
            self._log("[BLE_OTA_ENTRY] cancelled=1 terminal=failed")
            self._emit({"type": "ota_dfu_entry_result", "ok": False,
                        "ready_for_dfu_scan": False,
                        "message": "OTA 模式切换已中断，请检查连接后重试"})
            raise
        except Exception as exc:
            self._log(f"[BLE_OTA_ENTRY][ERR] terminal=failed error={type(exc).__name__}: {exc}")
            self._emit({"type": "ota_dfu_entry_result", "ok": False,
                        "ready_for_dfu_scan": False,
                        "message": f"OTA 模式切换失败：{exc}"})
        finally:
            self._ota_entry_operation_active = False
            if recovery_timer_paused and self._recovery_only:
                self._schedule_recovery_idle_disconnect()

    async def _enter_ota_dfu_mode_once(self) -> None:
        if not self._central_business_ready():
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": "Host-CI Central mode is not ready",
                }
            )
            return
        if self._calibration_active:
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": "Magnetometer calibration is active; OTA entry is blocked",
                }
            )
            return
        if (
            not self._connection_business_ready
            or self._last_device_state != DEVICE_STATE_WAIT_START
            or self._pending_command_active
            or self._feature_config_pending
            or self._online_active
            or self._online_start_operation_state != START_OP_IDLE
        ):
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": "设备存在采集、配置或其他独占操作；OTA 仅允许在 Business Ready / WAIT_START 空闲态启动",
                }
            )
            return
        client = self._client
        if client is None or not client.is_connected:
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ready_for_dfu_scan": False,
                    "message": "未连接普通 BLE 设备，无法发送进入 DFU 指令",
                }
            )
            return

        intent_state, intent_message = await self._request_ota_link_intent(client)
        if intent_state.startswith("failed"):
            if intent_state == "failed_disconnect":
                await self._safe_disconnect()
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": intent_message,
                }
            )
            return
        self._emit(
            {
                "type": "ota_dfu_entry_status",
                "status": "link_intent_ready",
                "message": intent_message,
            }
        )

        high_speed_ready, high_speed_message = await self._request_ota_high_speed_link(
            client
        )
        if not high_speed_ready:
            await self._safe_disconnect()
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": high_speed_message,
                }
            )
            return
        self._emit(
            {
                "type": "ota_dfu_entry_status",
                "status": "host_high_speed_ready",
                "message": high_speed_message,
            }
        )

        self._emit(
            {
                "type": "ota_dfu_entry_status",
                "status": "starting",
                "message": "正在发送普通 BLE 进入 DFU 指令",
            }
        )

        prepare_state, prepare_message = await self._prepare_ota_active_link(client)
        if prepare_state == "failed":
            await self._release_ota_high_speed_link("ota_prepare_failed")
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ack_received": False,
                    "ready_for_dfu_scan": False,
                    "message": prepare_message,
                }
            )
            return
        self._emit(
            {
                "type": "ota_dfu_entry_status",
                "status": "link_ready" if prepare_state == "ready" else "legacy_compat",
                "message": prepare_message,
            }
        )

        if prepare_state == "ready":
            commit_state, commit_message = await self._commit_ota_entry(client)
            if commit_state == "failed":
                await self._release_ota_high_speed_link("ota_commit_failed")
                self._emit(
                    {
                        "type": "ota_dfu_entry_result",
                        "ok": False,
                        "ack_received": False,
                        "ready_for_dfu_scan": False,
                        "message": commit_message,
                    }
                )
                return
            if commit_state == "committed":
                self._emit(
                    {
                        "type": "ota_dfu_entry_result",
                        "ok": True,
                        "ack_received": True,
                        "ready_for_dfu_scan": True,
                        "message": commit_message,
                    }
                )
                return
            prepare_state = "legacy"
            prepare_message = commit_message
            self._emit(
                {
                    "type": "ota_dfu_entry_status",
                    "status": "legacy_compat",
                    "message": prepare_message,
                }
            )

        target = self._find_ota_dfu_entry_target(client)
        if target is None:
            await self._release_ota_high_speed_link("ota_legacy_target_missing")
            message = (
                "进入 DFU 指令未发送：未发现 Simple BLE OTA 写入通道 "
                "(优先 B005 write+notify，兼容 B002 write + B003 notify)"
            )
            self._log(f"[BLE_OTA_ENTRY][ERR] {message}")
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ready_for_dfu_scan": False,
                    "message": message,
                }
            )
            return

        if self._ota_dfu_entry_ack_future is not None and not self._ota_dfu_entry_ack_future.done():
            self._ota_dfu_entry_ack_future.cancel()
        ack_future = asyncio.get_running_loop().create_future()
        self._ota_dfu_entry_ack_future = ack_future

        if not await self._subscribe_ota_dfu_entry_notify(client, target):
            if self._ota_dfu_entry_ack_future is ack_future:
                self._ota_dfu_entry_ack_future = None
            await self._release_ota_high_speed_link("ota_legacy_notify_failed")
            message = "进入 DFU 指令未发送：Simple BLE OTA ACK Notify 未订阅成功"
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ready_for_dfu_scan": False,
                    "message": message,
                }
            )
            return

        await self._settle_ota_notify_subscription(client)

        used_response = True
        try:
            used_response = await self._connection_coordinator.write_gatt_char_with_fallback(
                self._connection_token, client, target.write_uuid,
                OTA_DFU_ENTRY_REQUEST, name="ota_dfu_entry",
            )
        except Exception as exc:
            if self._ota_dfu_entry_ack_future is ack_future:
                self._ota_dfu_entry_ack_future = None
            await self._stop_ota_dfu_entry_notify(client)
            await self._release_ota_high_speed_link("ota_legacy_write_failed")
            message = f"进入 DFU 指令写入失败：{exc}"
            self._log(f"[BLE_OTA_ENTRY][ERR] {message}")
            self._emit(
                {
                    "type": "ota_dfu_entry_write_result",
                    "ok": False,
                    "response": used_response,
                    "target": target.label,
                    "uuid": target.write_uuid,
                    "payload_hex": hex_bytes(OTA_DFU_ENTRY_REQUEST),
                    "error": str(exc),
                }
            )
            self._emit(
                {
                    "type": "ota_dfu_entry_result",
                    "ok": False,
                    "ready_for_dfu_scan": False,
                    "message": message,
                }
            )
            return

        write_handle = f"0x{target.write_handle:04X}" if target.write_handle is not None else "<unknown>"
        self._log(
            "[BLE_OTA_ENTRY] "
            f"request_sent target={target.label} uuid={target.write_uuid} handle={write_handle} "
            f"response={1 if used_response else 0} payload={hex_bytes(OTA_DFU_ENTRY_REQUEST)}"
        )
        self._emit(
            {
                "type": "ota_dfu_entry_write_result",
                "ok": True,
                "response": used_response,
                "target": target.label,
                "uuid": target.write_uuid,
                "payload_hex": hex_bytes(OTA_DFU_ENTRY_REQUEST),
            }
        )

        if prepare_state == "legacy" and not ack_future.done():
            try:
                await asyncio.wait_for(
                    asyncio.shield(ack_future),
                    timeout=self._ota_legacy_retry_wait_s,
                )
            except asyncio.TimeoutError:
                if self._client is client and client.is_connected:
                    retry_response = True
                    try:
                        retry_response = await self._connection_coordinator.write_gatt_char_with_fallback(
                            self._connection_token, client, target.write_uuid,
                            OTA_DFU_ENTRY_REQUEST, name="ota_dfu_entry_retry",
                        )
                        self._log(
                            "[BLE_OTA_ENTRY] legacy_auto_retry=1 "
                            f"response={int(retry_response)} wait_ms={self._ota_legacy_retry_wait_s * 1000:.0f}"
                        )
                    except Exception as exc:
                        self._log(
                            f"[BLE_OTA_ENTRY][ERR] legacy_auto_retry_failed error={exc}"
                        )

        ack_received = False
        try:
            ack_result = await asyncio.wait_for(ack_future, timeout=self._ota_dfu_entry_ack_timeout_s)
            if not ack_result.get("ok", False):
                await self._stop_ota_dfu_entry_notify(client)
                await self._release_ota_high_speed_link("ota_legacy_rejected")
                message = str(ack_result.get("message") or "进入 DFU 指令被设备拒绝")
                self._log(f"[BLE_OTA_ENTRY][ERR] {message}")
                self._emit(
                    {
                        "type": "ota_dfu_entry_result",
                        "ok": False,
                        "ack_received": False,
                        "ready_for_dfu_scan": False,
                        "message": message,
                    }
                )
                return
            ack_received = True
        except asyncio.TimeoutError:
            if self._client is client and client.is_connected:
                await self._stop_ota_dfu_entry_notify(client)
                await self._release_ota_high_speed_link("ota_legacy_ack_timeout")
                message = "进入 DFU ACK 超时，设备仍保持普通 BLE 连接，未启动 DFU 扫描"
                self._log(
                    "[BLE_OTA_ENTRY][ERR] "
                    f"ACK timeout after {self._ota_dfu_entry_ack_timeout_s:.1f}s; still connected"
                )
                self._emit(
                    {
                        "type": "ota_warning",
                        "message": message,
                    }
                )
                self._emit(
                    {
                        "type": "ota_dfu_entry_result",
                        "ok": False,
                        "ack_received": False,
                        "ready_for_dfu_scan": False,
                        "message": message,
                    }
                )
                return
            self._log(
                "[BLE_OTA_ENTRY] "
                f"ACK timeout after {self._ota_dfu_entry_ack_timeout_s:.1f}s; normal BLE already disconnected"
            )
        except asyncio.CancelledError:
            if self._client is not None and self._client is client and client.is_connected:
                raise
            self._log("[BLE_OTA_ENTRY] ACK wait cancelled after normal BLE disconnect; start DFU scan")
        finally:
            if self._ota_dfu_entry_ack_future is ack_future:
                self._ota_dfu_entry_ack_future = None

        if ack_received and self._ota_dfu_entry_disconnect_delay_s > 0:
            await asyncio.sleep(self._ota_dfu_entry_disconnect_delay_s)

        if ack_received and self._client is client and client.is_connected:
            await self._safe_disconnect()

        message = "设备已收到进入 DFU ACK，开始扫描 ZY100_OTA" if ack_received else "普通 BLE 已断开，开始扫描 ZY100_OTA"
        self._emit(
            {
                "type": "ota_dfu_entry_result",
                "ok": True,
                "ack_received": ack_received,
                "ready_for_dfu_scan": True,
                "message": message,
            }
        )

    def _set_online_start_operation_state(self, state: str, *, reason: str) -> None:
        old_state = self._online_start_operation_state
        if old_state == state:
            return
        self._online_start_operation_state = state
        pending = state in START_OP_BUSY_STATES
        self._log(
            "[START_OP] "
            f"op={self._online_start_operation_id} "
            f"attempt={self._online_start_attempt} "
            f"from={old_state} to={state} reason={reason} "
            f"pending={1 if pending else 0}"
        )
        self._emit_online_status(
            None,
            start_operation_state=state,
            start_operation_pending=pending,
            operation_id=self._online_start_operation_id,
            attempt=self._online_start_attempt,
        )

    def _claim_online_start_operation(self, *, recovery_retry: bool) -> bool:
        if recovery_retry:
            if (
                self._online_start_operation_state != START_OP_RECOVERING
                or self._online_start_operation_id == 0
            ):
                self._log(
                    "[START_OP][ERR] recovery_claim_rejected "
                    f"state={self._online_start_operation_state} "
                    f"op={self._online_start_operation_id}"
                )
                return False
            self._online_start_attempt = 2
            self._set_online_start_operation_state(
                START_OP_PREPARING,
                reason="recovery_retry_claimed",
            )
            return True

        if self._online_start_operation_state != START_OP_IDLE:
            self._log(
                "[START_OP] duplicate_start_rejected "
                f"state={self._online_start_operation_state} "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt}"
            )
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "START is already in progress; wait for its final "
                        "result before starting another capture"
                    ),
                }
            )
            self._emit_online_status(
                None,
                start_operation_state=self._online_start_operation_state,
                start_operation_pending=True,
                start_rejected_busy=True,
            )
            return False

        self._online_start_ci_recovery_used = False
        self._online_start_operation_counter = (
            self._online_start_operation_counter + 1
        ) & 0xFFFFFFFF
        if self._online_start_operation_counter == 0:
            self._online_start_operation_counter = 1
        self._online_start_operation_id = self._online_start_operation_counter
        self._online_start_attempt = 1
        self._set_online_start_operation_state(
            START_OP_PREPARING,
            reason="user_start_claimed",
        )
        return True

    def _finish_online_start_operation(self, *, reason: str) -> None:
        self._cancel_online_start_final_timeout()
        self._cancel_online_start_ci48_restore_wait()
        self._online_start_seq = None
        if self._online_start_operation_state != START_OP_IDLE:
            self._set_online_start_operation_state(
                START_OP_TERMINAL,
                reason=reason,
            )
            self._set_online_start_operation_state(
                START_OP_IDLE,
                reason=f"{reason}_cleanup_complete",
            )

    async def _send_zy100_command(
        self,
        *,
        cmd: int,
        user_id: int,
        training_id: int,
        device_time_ms: int | None,
        _ci_recovery_retry: bool = False,
    ) -> None:
        if self.stress.preparing and cmd != CMD_PAUSE_CAPTURE:
            self._emit({"type": "error", "message": "链路压测配置进行中"})
            return
        claimed = False
        if cmd == CMD_START_CAPTURE:
            if self._online_fault_reason or (self._online_fault_task is not None and not self._online_fault_task.done()):
                self._log("[ONLINE_STATE][ERR] start blocked: fault stop pending")
                return
            claimed = self._claim_online_start_operation(
                recovery_retry=_ci_recovery_retry,
            )
            if not claimed:
                return
            self._log(
                "[CI_RECOVERY] "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt} stage=send_start"
            )
        try:
            await self._send_zy100_command_impl(
                cmd=cmd,
                user_id=user_id,
                training_id=training_id,
                device_time_ms=device_time_ms,
                _ci_recovery_retry=_ci_recovery_retry,
            )
        except asyncio.CancelledError:
            if claimed and self._online_start_operation_state == START_OP_PREPARING:
                self._finish_online_start_operation(reason="send_cancelled")
            raise
        except Exception:
            if claimed and self._online_start_operation_state == START_OP_PREPARING:
                self._finish_online_start_operation(reason="send_exception")
            raise
        if claimed and self._online_start_operation_state == START_OP_PREPARING:
            self._finish_online_start_operation(reason="start_not_sent")

    async def _send_zy100_command_impl(
        self,
        *,
        cmd: int,
        user_id: int,
        training_id: int,
        device_time_ms: int | None,
        _ci_recovery_retry: bool = False,
    ) -> None:
        if cmd == CMD_START_CAPTURE:
            assert self._online_start_operation_state == START_OP_PREPARING

        client = self._client
        if client is None or not client.is_connected:
            self._emit({"type": "error", "message": "No connected device"})
            return

        if cmd != CMD_OFFLINE_CAPTURE_STOP and not self._central_business_ready():
            self._log(
                "[HOST_CI_CENTRAL][ERR] business_blocked "
                f"command={command_name(cmd)} reason=central_not_ready"
            )
            self._emit(
                {
                    "type": "error",
                    "message": f"Host-CI Central mode is not ready; {command_name(cmd)} was not sent",
                }
            )
            return

        if self._shipping_operation_active and cmd != CMD_PING:
            self._emit(
                {
                    "type": "error",
                    "message": "进入船运模式事务进行中，等待设备断开",
                }
            )
            return

        if cmd == CMD_ENTER_SHIPPING:
            shipping_state = self._last_device_state
            pending = self._pending_command_active
            if shipping_state not in {None, 0, DEVICE_STATE_WAIT_START} or pending:
                self._log(
                    "[SHIPPING_GUARD] blocked "
                    f"state={shipping_state if shipping_state is not None else 'unknown'} "
                    f"pending={int(pending)}"
                )
                self._emit(
                    {
                        "type": "error",
                        "message": "船运模式仅允许在空闲/Standby 且无 pending 任务时执行",
                    }
                )
                return
            self._log(
                "[SHIPPING_GUARD] allowed "
                f"state={shipping_state if shipping_state is not None else 'unknown'} "
                f"pending={int(pending)}"
            )

        if (
            cmd in {CMD_ONLINE_STREAM_READY, CMD_START_CAPTURE}
            and not self._is_calibration_business_ready()
        ):
            self._emit(
                {
                    "type": "error",
                    "message": "Calibration summary or complete record is not confirmed; online start is blocked",
                }
            )
            return

        if self._calibration_active and cmd not in {CMD_PING, CMD_OFFLINE_CAPTURE_STOP}:
            self._emit(
                {
                    "type": "error",
                    "message": f"Magnetometer calibration is active; {command_name(cmd)} is blocked",
                }
            )
            return

        if self._feature_config_pending and cmd != CMD_PING:
            self._emit(
                {
                    "type": "error",
                    "message": f"功能配置同步进行中；{command_name(cmd)} 已被互斥门禁阻止",
                }
            )
            return

        if self._offline_priority_blocked and cmd not in {CMD_PING, CMD_OFFLINE_CAPTURE_STOP}:
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "离线数据上传中，在线采集、清空 Flash、地磁校准和 OTA 已暂停"
                    ),
                }
            )
            return

        if self._command_uuid is None or self._ack_uuid is None or self._export_uuid is None:
            self._inspect_control_service(client)

        if not self._has_required_gatt():
            self._emit_gatt_mismatch(action=f"{command_name(cmd)} not sent")
            return

        if not self._ack_subscribed:
            await self._subscribe_ack()
        if not self._export_subscribed:
            await self._subscribe_export()
        if not self._is_gatt_ready():
            self._emit_gatt_mismatch(action=f"{command_name(cmd)} not sent", include_subscriptions=True)
            return

        if self._export_commands_blocked and cmd in {
            CMD_START_CAPTURE,
            CMD_CLEAR_FLASH,
            CMD_ENTER_SHIPPING,
        }:
            self._emit(
                {
                    "type": "error",
                    "message": "Data transfer is not device-ready; START/CLEAR are blocked until reconnect or WAIT_START",
                }
            )
            return

        if (
            cmd in {CMD_START_CAPTURE, CMD_OFFLINE_CAPTURE_START}
            and self._feature_config_supported
            and (not self._feature_config_synced or self._feature_config_pending)
        ):
            self._emit(
                {
                    "type": "error",
                    "message": "功能配置尚未与设备确认，START 已锁定",
                }
            )
            return

        if user_id <= 0:
            self._emit({"type": "error", "message": "user_id must be 1..0xFFFFFFFF"})
            return
        if training_id <= 0 and cmd not in {
            CMD_OFFLINE_CAPTURE_START,
            CMD_OFFLINE_CAPTURE_STOP,
            CMD_ENTER_SHIPPING,
        }:
            training_id = 1

        if cmd in {CMD_OFFLINE_CAPTURE_START, CMD_OFFLINE_CAPTURE_STOP}:
            info = self._connection_coordinator.device_info
            if info is None or info.fields.get("offctl") != "1":
                self._emit(
                    {
                        "type": "error",
                        "message": "当前固件未声明 offctl=1，需升级固件后使用离线 BLE 控制",
                    }
                )
                return
            if cmd == CMD_OFFLINE_CAPTURE_START and self._legacy_offline_intent_bug_present():
                self._log(
                    "[OFFLINE_CTRL][LEGACY_BLOCK] command=OFFLINE_CAPTURE_START "
                    f"code={self._device_version_code()} fixed_from={LEGACY_OFFLINE_INTENT_FIX_CODE}"
                )
                self._emit(
                    {
                        "type": "error",
                        "message": (
                            f"固件 {self._device_version_code()} 存在离线启动 0x0E 卡死问题，"
                            "上位机已禁止 BLE 离线开始；请使用设备实体双击开始，"
                            f"或升级到 {LEGACY_OFFLINE_INTENT_FIX_CODE} 及以上固件。"
                        ),
                    }
                )
                return
            if cmd == CMD_OFFLINE_CAPTURE_STOP and self._offline_stop_any_supported():
                requester = self._connection_requested_user_id
            else:
                requester = self._connection_user_id if self._connection_user_synced else None
            if requester is None:
                self._emit({"type": "error", "message": (
                    "旧固件需要先确认同一用户身份才能暂停；请等待采集结束或升级固件"
                    if cmd == CMD_OFFLINE_CAPTURE_STOP else "当前连接 user ID 尚未同步，离线控制未发送")})
                return
            user_id = requester
            training_id = 0
            device_time_ms = 0
            if cmd == CMD_OFFLINE_CAPTURE_START and (
                self._last_device_state != DEVICE_STATE_WAIT_START
                or self._offline_start_seq is not None
                or self._pending_command_active
            ):
                self._emit(
                    {
                        "type": "error",
                        "message": "离线开始仅在 Business Ready / WAIT_START 且无待处理命令时开放",
                    }
                )
                return
            if cmd == CMD_OFFLINE_CAPTURE_STOP and self._last_device_state not in {
                DEVICE_STATE_OFFLINE_INTENT_READY,
                DEVICE_STATE_OFFLINE_CAPTURING,
                DEVICE_STATE_OFFLINE_FINALIZING,
            }:
                self._emit(
                    {
                        "type": "error",
                        "message": "当前没有可停止的离线采集场次",
                    }
                )
                return

        if cmd == CMD_START_CAPTURE and not self._is_active_training(user_id, training_id):
            self._log("[BLE_SYNC] START requires fresh TIME_SYNC; syncing before START_CAPTURE")
            await self._sync_rtc_after_notify(user_id=user_id, training_id=training_id)
            if not self._is_active_training(user_id, training_id):
                self._emit({"type": "error", "message": "RTC time sync is not complete; START_CAPTURE not sent"})
                return

        if cmd == CMD_START_CAPTURE:
            online_ready_before_start = await self._ensure_online_ready_before_start()
            if not online_ready_before_start:
                self._log("[ONLINE_STATE] prestart_ready_not_ok; START not sent")
                self._emit_online_status("start_blocked_ready_not_ok")
                return
            await self._enter_export_high_speed("before_start_capture")

        if cmd == CMD_PAUSE_CAPTURE and self._active_user_id is not None and self._active_training_id is not None:
            user_id = self._active_user_id
            training_id = self._active_training_id

        if cmd == CMD_START_CAPTURE:
            user_id = self._active_user_id if self._active_user_id is not None else user_id
            training_id = self._active_training_id if self._active_training_id is not None else training_id

        if cmd == CMD_PAUSE_CAPTURE:
            await self._enter_export_high_speed("before_pause_capture")
            self._schedule_online_summary_partial_ack(force=True)

        seq = self._reserve_control_seq()
        if device_time_ms is None:
            device_time_ms = int(time.time() * 1000)
        if cmd == CMD_ENTER_SHIPPING:
            payload = build_enter_shipping_command(
                seq=seq,
                user_id=user_id,
                device_time_ms=device_time_ms,
            )
        else:
            payload = build_zy100_command(
                cmd=cmd,
                seq=seq,
                user_id=user_id,
                training_id=training_id,
                device_time_ms=device_time_ms,
            )
        if cmd == CMD_START_CAPTURE:
            self._cancel_online_start_final_timeout()
            self._online_start_seq = seq
            self._set_online_start_operation_state(
                START_OP_WAIT_FINAL,
                reason=f"start_frame_ready_seq_{seq}",
            )
            self._emit_online_status("start_command_pending", start_seq=seq)
        elif cmd == CMD_OFFLINE_CAPTURE_START:
            self._cancel_offline_start_final_timeout()
            self._offline_start_seq = seq
        elif cmd == CMD_ENTER_SHIPPING:
            self._shipping_operation_active = True
            self._shipping_seq = seq
            self._shipping_accepted = False

        used_response = True
        clear_transaction = None
        self._pending_command_active = True
        try:
            try:
                if cmd == CMD_CLEAR_FLASH:
                    clear_transaction = _ClearFlashTransaction(seq, self._connection_token, self._client)
                    await self._track_clear_flash_request(seq, transaction=clear_transaction)
                    if not self._clear_flash_is_current(clear_transaction):
                        return
                used_response = await self._write_central_control_frame(
                    payload, label=command_name(cmd)
                )
            except asyncio.CancelledError:
                if clear_transaction is not None and not clear_transaction.ack_received:
                    await self._clear_flash_send_failed(clear_transaction, "CLEAR_FLASH send cancelled")
                raise
            except Exception as exc:  # pragma: no cover - host stack dependent
                if clear_transaction is not None:
                    if clear_transaction.ack_received:
                        self._log(f"[BLE_CLEAR] transport_error_after_ack seq={seq} error={exc}")
                        return  # Device ACK remains authoritative after an ATT failure.
                    await self._clear_flash_send_failed(clear_transaction, str(exc))
                self._log(f"Write command failed: {exc}")
                self._emit(
                    {
                        "type": "command_write_result",
                        "ok": False,
                        "cmd": cmd & 0xFF,
                        "cmd_name": command_name(cmd),
                        "seq": seq,
                        "error": str(exc),
                        "payload_hex": hex_bytes(payload),
                    }
                )
                self._emit({"type": "error", "message": f"Write command failed: {exc}"})
                if cmd == CMD_START_CAPTURE:
                    self._finish_online_start_operation(
                        reason="start_write_failed",
                    )
                elif cmd == CMD_OFFLINE_CAPTURE_START:
                    self._offline_start_seq = None
                elif cmd == CMD_ENTER_SHIPPING:
                    self._shipping_operation_active = False
                    self._shipping_seq = None
                    self._shipping_accepted = False
                return
        finally:
            self._pending_command_active = False

        self._log(
            "[BLE_TX] "
            f"{command_name(cmd)} seq={seq} user_id={user_id} training_id={training_id} "
            f"unix_time_ms={device_time_ms} response={1 if used_response else 0} "
            f"op={self._online_start_operation_id if cmd == CMD_START_CAPTURE else 0} "
            f"attempt={self._online_start_attempt if cmd == CMD_START_CAPTURE else 0} "
            f"payload={hex_bytes(payload)}"
        )
        self._emit(
            {
                "type": "command_write_result",
                "ok": True,
                "cmd": cmd & 0xFF,
                "cmd_name": command_name(cmd),
                "seq": seq,
                "user_id": user_id,
                "training_id": training_id,
                "response": used_response,
                "payload_hex": hex_bytes(payload),
            }
        )
        if cmd == CMD_START_CAPTURE and self._online_start_seq == seq:
            self._online_start_final_timeout_task = self._loop.create_task(
                self._online_start_final_timeout(seq)
            )
        elif cmd == CMD_OFFLINE_CAPTURE_START and self._offline_start_seq == seq:
            self._offline_start_final_timeout_task = self._loop.create_task(
                self._offline_start_final_timeout(seq)
            )

    def _is_active_training(self, user_id: int, training_id: int) -> bool:
        return (
            self._rtc_sync_ok
            and self._active_user_id == (user_id & 0xFFFFFFFF)
            and self._active_training_id == (training_id & 0xFFFFFFFF)
        )

    def _offline_stop_any_supported(self) -> bool:
        info = self._connection_coordinator.device_info
        return bool(info is not None and info.fields.get("offstop_any") == "1")

    def _defer_connection_user_sync(self) -> bool:
        if self._user_sync_result != UserSyncResult.WAITING_CAPTURE:
            self._log("[USER_CTX] waiting_capture=1 polling=0")
            self._emit({"type": "connection_user_sync", "status": "waiting_capture",
                        "message": "正在离线采集，用户同步将在结束后进行"})
        self._user_sync_result = UserSyncResult.WAITING_CAPTURE
        return True

    async def _prepare_connection_user_context(self, user_id: int) -> bool:
        self._connection_requested_user_id = user_id
        info = self._connection_coordinator.device_info
        self._feature_config_supported = bool(
            info is not None and info.fields.get("fcfg") == "1"
        )
        if not self._feature_config_supported:
            self._emit_feature_config_status(
                "unsupported",
                message="固件不支持配置同步，设备使用固件默认值",
            )
        self._connection_user_context_supported = bool(
            info is not None and info.usrctx == 1
        )
        if not self._connection_user_context_supported:
            self._log(
                "[USER_CTX][LEGACY] usrctx_absent=1 command_0x1A_skipped=1 "
                "mode=administrator_compatibility"
            )
            self._emit(
                {
                    "type": "legacy_user_isolation_warning",
                    "message": (
                        "旧固件不支持设备端用户隔离，当前为管理员兼容模式；"
                        "设备暴露的离线数据会按旧流程自动上传。"
                    ),
                }
            )
            return True
        if self._offline_v2_capture_in_progress() and self._offline_stop_any_supported():
            return self._defer_connection_user_sync()
        self._connection_coordinator.transition(ConnectionStage.SYNCING_USER)
        return await self._sync_connection_user(user_id, query_only=False)

    def _connection_user_context_ready_for(self, user_id: int) -> bool:
        info = self._connection_coordinator.device_info
        if info is None or info.usrctx != 1:
            return True
        return self._connection_user_synced and self._connection_user_id == user_id

    async def _ensure_connection_user_context_ready(self, user_id: int) -> bool:
        if self._connection_user_context_ready_for(user_id):
            self._user_sync_result = UserSyncResult.COMPLETE
            return True
        if self._offline_v2_capture_in_progress() and (
            self._offline_stop_any_supported() or self._capture_identity_attempted
        ):
            self._defer_connection_user_sync()
            return False
        if self._connection_user_id == user_id:
            ok = await self._sync_connection_user(user_id, query_only=True)
        else:
            ok = await self._prepare_connection_user_context(user_id)
            if ok and self._user_sync_result != UserSyncResult.WAITING_CAPTURE and self._connection_user_context_supported and not self._connection_user_synced:
                ok = await self._sync_connection_user(user_id, query_only=True)
        return ok and self._user_sync_result != UserSyncResult.WAITING_CAPTURE

    async def _sync_connection_user(self, user_id: int, *, query_only: bool) -> bool:
        if self._offline_v2_capture_in_progress():
            if self._offline_stop_any_supported() or self._capture_identity_attempted:
                return self._defer_connection_user_sync()
            self._capture_identity_attempted = True
        self._user_sync_result = UserSyncResult.FAILED
        token = self._connection_token
        if not 1 <= user_id <= 0xFFFFFFFF:
            return False
        client = self._client
        if client is None or not client.is_connected or self._command_uuid is None:
            return False
        deadline = asyncio.get_running_loop().time() + 15.0
        attempt = 0
        while asyncio.get_running_loop().time() < deadline:
            if attempt and self._offline_v2_capture_in_progress():
                return self._defer_connection_user_sync()
            attempt += 1
            seq = self._reserve_control_seq()
            future: asyncio.Future[ZY100Ack] = asyncio.get_running_loop().create_future()
            self._connection_user_sync_ack_future = future
            self._connection_user_sync_seq = seq
            payload = build_connection_user_sync_command(seq, user_id)
            try:
                await self._write_central_control_frame(
                    payload,
                    label="CONNECTION_USER_SYNC_QUERY" if query_only else "CONNECTION_USER_SYNC",
                )
                ack = await asyncio.wait_for(future, timeout=3.0)
            except asyncio.TimeoutError:
                if self._offline_v2_capture_in_progress():
                    return self._defer_connection_user_sync()
                self._log(f"[USER_CTX] timeout seq={seq} attempt={attempt}")
                continue
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._log(f"[USER_CTX][ERR] write_failed reason={exc}")
                return False
            finally:
                if self._connection_user_sync_ack_future is future:
                    self._connection_user_sync_ack_future = None
                    self._connection_user_sync_seq = None

            if (self._client is not client or not client.is_connected
                    or not self._connection_coordinator.is_current(token)):
                return False
            capture_state = ack.device_state in {
                DEVICE_STATE_OFFLINE_INTENT_READY, DEVICE_STATE_OFFLINE_CAPTURING,
                DEVICE_STATE_OFFLINE_FINALIZING,
            }
            if ack.status == STATUS_BUSY and capture_state:
                self._last_device_state = ack.device_state
                self._capture_identity_attempted = True
                return self._defer_connection_user_sync()
            if ack.status == STATUS_BUSY:
                self._log(
                    f"[USER_CTX] busy seq={seq} attempt={attempt} device_state=0x{ack.device_state:02X}"
                )
                await asyncio.sleep(0.25)
                continue
            if (
                ack.status == STATUS_OK
                and ack.reserved == 1
                and ack.user_id == user_id
                and ack.exec_mode == EXEC_MODE_ACCEPTED_ASYNC
            ):
                self._connection_user_id = user_id
                self._connection_user_session_count = ack.training_id
                self._foreign_session_count = ack.detail
                self._foreign_purge_device_active = (
                    ack.device_state == DEVICE_STATE_OFFLINE_RECLAIMING
                )
                self._active_user_id = user_id
                self._log(
                    "[USER_CTX] accepted_async=1 "
                    f"user_id={user_id} owned={ack.training_id} foreign={ack.detail}"
                )
                if capture_state or self._offline_v2_capture_in_progress():
                    self._last_device_state = ack.device_state
                    self._capture_identity_attempted = True
                    return self._defer_connection_user_sync()
                if not query_only:
                    return True
                await asyncio.sleep(0.25)
                continue
            if (
                ack.status != STATUS_OK
                or ack.reserved != 1
                or ack.user_id != user_id
                or ack.exec_mode not in {0x01, EXEC_MODE_REAL_ACTION}
            ):
                self._log(
                    "[USER_CTX][ERR] rejected "
                    f"seq={seq} status=0x{ack.status:02X} exec=0x{ack.exec_mode:02X} "
                    f"version={ack.reserved} echo={ack.user_id}"
                )
                return False
            self._connection_user_synced = True
            self._user_sync_result = UserSyncResult.COMPLETE
            self._connection_user_id = user_id
            self._connection_user_session_count = ack.training_id
            self._foreign_session_count = ack.detail
            self._foreign_purge_device_active = (
                ack.device_state == DEVICE_STATE_OFFLINE_RECLAIMING
            )
            self._active_user_id = user_id
            self._log(
                "[USER_CTX] synced=1 "
                f"user_id={user_id} owned={ack.training_id} foreign={ack.detail} "
                f"query={int(query_only)}"
            )
            self._emit(
                {
                    "type": "connection_user_sync",
                    "status": "ok",
                    "user_id": user_id,
                    "current_user_sessions": ack.training_id,
                    "other_or_unknown_sessions": ack.detail,
                    "query": query_only,
                }
            )
            return True
        self._log("[USER_CTX][ERR] safe_window_timeout=1")
        return False

    async def _prompt_foreign_sessions_if_needed(self) -> bool:
        user_id = self._connection_user_id
        if not self._connection_user_context_supported or user_id is None:
            return True
        if not await self._sync_connection_user(user_id, query_only=True):
            return False
        info = self._connection_coordinator.device_info
        offpurge_supported = bool(
            info is not None and info.fields.get("offpurge") == "1"
        )
        if self._foreign_purge_device_active:
            self._foreign_purge_expected_user_id = user_id
            return await self._execute_foreign_purge()
        if self._foreign_session_count <= 0 or self._foreign_prompt_shown:
            return True
        if not offpurge_supported:
            message = (
                "当前固件不支持按 session 定向清理；其他用户数据已保留。"
                "请升级到 VERSION_CODE 10446 或更高版本，工具不会回退到全量 CLEAR_FLASH。"
            )
            self._log("[USER_CTX] offpurge_missing=1 fallback_global_clear=0")
            self._emit(
                {
                    "type": "foreign_offline_data_retained",
                    "count": self._foreign_session_count,
                    "message": message,
                    "upgrade_required": True,
                }
            )
            return True
        self._foreign_prompt_shown = True
        future: asyncio.Future[bool] = asyncio.get_running_loop().create_future()
        self._foreign_prompt_future = future
        self._emit(
            {
                "type": "foreign_offline_data_prompt",
                "count": self._foreign_session_count,
                "message": "检测到 Flash 中存在不属于您的数据，是否清理？",
            }
        )
        try:
            confirm = await future
        except asyncio.CancelledError:
            return False
        finally:
            if self._foreign_prompt_future is future:
                self._foreign_prompt_future = None
        if not confirm:
            self._log(
                f"[USER_CTX] foreign_cleanup_cancelled count={self._foreign_session_count}"
            )
            self._emit(
                {
                    "type": "foreign_offline_data_retained",
                    "count": self._foreign_session_count,
                    "message": "其他用户数据已保留；中间空洞不可复用，可用容量会相应减少。",
                }
            )
            return True
        self._foreign_purge_expected_user_id = user_id
        cleared = await self._execute_foreign_purge()
        if not cleared:
            self._emit(
                {
                    "type": "foreign_offline_data_clear_failed",
                    "message": "其他用户定向清理状态未知或失败；不会自动执行全量清除，可重连后继续查询。",
                }
            )
            return True
        if not await self._sync_connection_user(user_id, query_only=True):
            return False
        return self._foreign_session_count == 0

    async def _execute_foreign_purge(self) -> bool:
        user_id = self._connection_user_id or self._foreign_purge_expected_user_id
        if user_id is None or user_id <= 0:
            return False
        while not self._foreign_purge_ack_queue.empty():
            try:
                self._foreign_purge_ack_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        last_progress: tuple[int, int, int] | None = None
        last_progress_at = asyncio.get_running_loop().time()
        unknown_reported = False

        async def send_query() -> None:
            seq = self._reserve_control_seq()
            await self._write_central_control_frame(
                build_zy100_command(
                    CMD_OFFLINE_FOREIGN_PURGE,
                    seq=seq,
                    user_id=user_id,
                    training_id=0,
                    device_time_ms=0,
                ),
                label="OFFLINE_FOREIGN_PURGE",
            )

        try:
            await send_query()
            while True:
                try:
                    ack = await asyncio.wait_for(
                        self._foreign_purge_ack_queue.get(), timeout=10.0
                    )
                except asyncio.TimeoutError:
                    await send_query()
                    if asyncio.get_running_loop().time() - last_progress_at >= 60.0:
                        if not unknown_reported:
                            unknown_reported = True
                            self._log(
                                "[USER_CTX] foreign_purge_no_progress_60s=1 state=unknown transaction_cancelled=0"
                            )
                            self._emit(
                                {
                                    "type": "foreign_offline_data_purge_progress",
                                    "status": "unknown",
                                    "message": "定向清理60秒无进度，设备事务仍保留；可断开重连后继续查询。",
                                }
                            )
                    continue

                if ack.status != STATUS_OK:
                    self._log(
                        "[USER_CTX][ERR] foreign_purge_rejected "
                        f"status=0x{ack.status:02X} detail=0x{ack.detail:08X}"
                    )
                    return False
                total_sectors = (ack.detail >> 16) & 0xFFFF
                erased_sectors = ack.detail & 0xFFFF
                progress = (ack.training_id, total_sectors, erased_sectors)
                if progress != last_progress:
                    last_progress = progress
                    last_progress_at = asyncio.get_running_loop().time()
                    unknown_reported = False
                    if ack.exec_mode not in {
                        EXEC_MODE_DRY_RUN_NO_ACTION,
                        EXEC_MODE_ASYNC_DONE,
                    }:
                        self._emit(
                            {
                                "type": "foreign_offline_data_purge_progress",
                                "status": "active",
                                "remaining_sessions": ack.training_id,
                                "total_sectors": total_sectors,
                                "erased_sectors": erased_sectors,
                                "message": (
                                    f"正在定向清理：剩余 {ack.training_id} 场，"
                                    f"已擦 {erased_sectors}/{total_sectors} 个4 KiB扇区。"
                                ),
                            }
                        )
                if ack.exec_mode in {
                    EXEC_MODE_DRY_RUN_NO_ACTION,
                    EXEC_MODE_ASYNC_DONE,
                }:
                    self._foreign_purge_expected_user_id = None
                    self._foreign_purge_device_active = False
                    self._emit(
                        {
                            "type": "foreign_offline_data_purge_progress",
                            "status": "done",
                            "remaining_sessions": 0,
                            "total_sectors": total_sectors,
                            "erased_sectors": erased_sectors,
                            "message": "其他用户或owner未知的离线session已完成定向清理。",
                        }
                    )
                    return True
                if ack.exec_mode != EXEC_MODE_ACCEPTED_ASYNC:
                    return False
        except asyncio.CancelledError:
            self._log(
                "[USER_CTX] foreign_purge_host_cancelled=1 device_transaction_cancelled=0"
            )
            raise
        except Exception as exc:
            self._log(f"[USER_CTX][ERR] foreign_purge_write_failed reason={exc}")
            return False

    async def _sync_rtc_after_notify(self, *, user_id: int, training_id: int) -> bool:
        self._connection_coordinator.transition(ConnectionStage.SYNCING_TIME)
        if user_id <= 0:
            self._rtc_sync_ok = False
            self._last_rtc_sync_ms = None
            self._emit_time_sync_failed("user_id must be 1..0xFFFFFFFF")
            return False
        if training_id <= 0:
            training_id = 1
        self._rtc_sync_ok = False
        self._last_rtc_sync_ms = None

        if not self._is_gatt_ready():
            self._emit_gatt_mismatch(action="TIME_SYNC not sent", include_subscriptions=True)
            return False

        self._emit({"type": "rtc_sync", "status": "syncing", "user_id": user_id, "training_id": training_id})

        for attempt in range(1, 4):
            client = self._client
            if client is None or not client.is_connected:
                self._emit_time_sync_failed("Device disconnected before RTC time sync completed")
                return False
            if self._command_uuid is None:
                self._emit_time_sync_failed(f"Command characteristic not found: {ZY100_COMMAND_UUID}")
                return False

            seq = self._reserve_control_seq()
            unix_time_ms = int(time.time() * 1000)
            payload = build_time_sync_command(
                seq=seq,
                user_id=user_id,
                training_id=training_id,
                unix_time_ms=unix_time_ms,
            )
            future: asyncio.Future[ZY100Ack] = self._loop.create_future()
            self._time_sync_ack_future = future
            self._time_sync_seq = seq
            self._time_sync_user_id = user_id
            self._time_sync_training_id = training_id

            used_response = True
            self._pending_command_active = True
            try:
                try:
                    used_response = await self._write_central_control_frame(
                        payload, label="TIME_SYNC"
                    )
                except Exception as exc:  # pragma: no cover - host stack dependent
                    self._clear_pending_time_sync(future)
                    self._log(f"Write TIME_SYNC failed: {exc}")
                    self._emit(
                        {
                            "type": "command_write_result",
                            "ok": False,
                            "cmd": CMD_TIME_SYNC,
                            "cmd_name": command_name(CMD_TIME_SYNC),
                            "seq": seq,
                            "error": str(exc),
                            "payload_hex": hex_bytes(payload),
                        }
                    )
                    self._emit_time_sync_failed(f"Write TIME_SYNC failed: {exc}")
                    return False
            finally:
                self._pending_command_active = False

            local_time = datetime.fromtimestamp(unix_time_ms / 1000).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            payload_hex = hex_bytes(payload)
            self._log(
                "[BLE_TX] "
                f"TIME_SYNC seq={seq} user_id={user_id} training_id={training_id} "
                f"unix_time_ms={unix_time_ms} local_time={local_time}"
            )
            self._log(f"[BLE_TX] payload={payload_hex}")
            self._emit(
                {
                    "type": "command_write_result",
                    "ok": True,
                    "cmd": CMD_TIME_SYNC,
                    "cmd_name": command_name(CMD_TIME_SYNC),
                    "seq": seq,
                    "user_id": user_id,
                    "training_id": training_id,
                    "unix_time_ms": unix_time_ms,
                    "response": used_response,
                    "payload_hex": payload_hex,
                }
            )

            try:
                ack = await asyncio.wait_for(future, timeout=3.0)
            except asyncio.TimeoutError:
                self._clear_pending_time_sync(future)
                self._log(f"[BLE_SYNC][ERR] timeout seq={seq} retry={attempt}")
                self._emit({"type": "rtc_sync", "status": "retry", "seq": seq, "retry": attempt})
                continue
            except asyncio.CancelledError:
                self._clear_pending_time_sync(future)
                self._emit_time_sync_failed("RTC time sync was cancelled")
                return False

            self._clear_pending_time_sync(future)
            if is_time_sync_ack_ok(
                ack,
                expected_seq=seq,
                expected_user_id=user_id,
                expected_training_id=training_id,
            ):
                self._rtc_sync_ok = True
                self._active_user_id = ack.user_id
                self._active_training_id = ack.training_id
                self._last_rtc_sync_ms = unix_time_ms
                self._log(f"[BLE_SYNC] rtc_sync_ok=1 user_id={ack.user_id} training_id={ack.training_id}")
                self._emit(
                    {
                        "type": "rtc_sync",
                        "status": "ok",
                        "user_id": ack.user_id,
                        "training_id": ack.training_id,
                        "seq": seq,
                        "unix_time_ms": unix_time_ms,
                    }
                )
                return True

            self._log(
                "[BLE_SYNC][ERR] "
                f"ack_failed seq={seq} status=0x{ack.status:02X} exec_mode=0x{ack.exec_mode:02X} "
                f"user_id={ack.user_id} training_id={ack.training_id} detail={ack.detail} raw={hex_bytes(ack.raw)}"
            )
            self._emit_time_sync_failed("Device RTC time sync ACK was not OK")
            return False

        self._emit_time_sync_failed("设备时间同步失败，请重新连接设备")

        return False

    def _clear_pending_time_sync(self, future: asyncio.Future[ZY100Ack] | None = None) -> None:
        if future is None or self._time_sync_ack_future is future:
            self._time_sync_ack_future = None
            self._time_sync_seq = None
            self._time_sync_user_id = None
            self._time_sync_training_id = None

    def _emit_time_sync_failed(self, message: str) -> None:
        self._rtc_sync_ok = False
        self._last_rtc_sync_ms = None
        self._log(f"[BLE_SYNC][ERR] {message}")
        self._emit({"type": "rtc_sync", "status": "failed", "message": "设备时间同步失败，请重新连接设备"})

    async def _subscribe_ack(self) -> None:
        self._connection_coordinator.transition(ConnectionStage.SUBSCRIBING_ACK)
        client = self._client
        if client is None or not client.is_connected:
            self._emit({"type": "error", "message": "No connected device"})
            return

        if self._ack_subscribed:
            self._notify_ready = True
            self._emit({"type": "ack_subscription", "status": "subscribed", "uuid": self._ack_uuid or ""})
            return

        if self._ack_uuid is None:
            ack_char = self._find_characteristic(client, ZY100_ACK_UUID)
            self._ack_uuid = ack_char.uuid.lower() if ack_char is not None else None
        if self._ack_uuid is None:
            self._notify_ready = False
            self._emit({"type": "ack_subscription", "status": "missing", "uuid": ""})
            self._emit({"type": "error", "message": f"ACK characteristic not found: {ZY100_ACK_UUID}"})
            return

        try:
            callback = self._connection_coordinator.guard_callback(
                self._connection_token, self._handle_ack_notification
            )
            await self._connection_coordinator.start_notify(
                self._connection_token, client, self._ack_uuid, callback,
                name="subscribe_ack",
            )
            self._ack_subscribed = True
            self._notify_ready = True
            self._log(f"[BLE_NOTIFY] ack_subscribed=1 uuid={self._ack_uuid}")
            self._emit({"type": "ack_subscription", "status": "subscribed", "uuid": self._ack_uuid})
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._ack_subscribed = False
            self._notify_ready = False
            self._log(f"Subscribe ACK failed: {exc}")
            self._emit({"type": "ack_subscription", "status": "failed", "uuid": self._ack_uuid, "error": str(exc)})
            self._emit({"type": "error", "message": f"ACK subscribe failed: {exc}"})

    def _reserve_control_seq(self) -> int:
        """Reserve one control-channel sequence before any coroutine can yield."""
        seq = self._next_seq & 0xFF
        self._next_seq = (seq + 1) & 0xFF
        return seq

    async def _write_central_control_frame(
        self, payload: bytes, *, label: str, log_tx: bool = True,
        expected_client: BleakClient | None = None, expected_token: int | None = None,
        expected_guard: Callable[[], bool] | None = None,
        allow_write_fallback: bool = True,
    ) -> bool:
        client = self._client
        if client is None or not client.is_connected or self._command_uuid is None:
            raise RuntimeError("central control link is not ready")
        started = time.monotonic()
        lock_started = started
        queue_before = self._connection_coordinator.gatt_diagnostic_snapshot()
        try:
            async with self._control_command_lock:
                if expected_token is not None and (
                        self._connection_token != expected_token or self._client is not expected_client
                        or client is not expected_client or not client.is_connected):
                    return False
                if expected_guard is not None and not expected_guard():
                    return False
                lock_wait_ms = (time.monotonic() - lock_started) * 1000.0
                if allow_write_fallback:
                    used_response = await self._connection_coordinator.write_gatt_char_with_fallback(
                        self._connection_token, client, self._command_uuid, payload,
                        priority=GattPriority.CRITICAL, name=label,
                        **({"guard": expected_guard} if expected_guard is not None else {}),
                    )
                else:
                    # A diagnostic click must not reapply twice after an ambiguous write error.
                    await self._connection_coordinator.write_gatt_char(
                        self._connection_token, client, self._command_uuid, payload,
                        response=True, priority=GattPriority.CRITICAL, name=label,
                    )
                    used_response = True
        except Exception as exc:
            self._link_trace.record_exception(
                "gatt_write_failed",
                exc,
                label=label,
                duration_ms=(time.monotonic() - started) * 1000.0,
                queue_before=queue_before,
                queue_after=self._connection_coordinator.gatt_diagnostic_snapshot(),
            )
            raise
        self._link_trace.record(
            "gatt_write_complete",
            label=label,
            duration_ms=(time.monotonic() - started) * 1000.0,
            control_lock_wait_ms=lock_wait_ms,
            write_with_response=used_response,
            write_without_response_fallback=not used_response,
            queue_before=queue_before,
            queue_after=self._connection_coordinator.gatt_diagnostic_snapshot(),
        )
        if log_tx:
            self._log(f"[HOST_CI_CENTRAL][TX] {label} payload={hex_bytes(payload)}")
        return used_response

    async def _send_offline_v2_command(
        self,
        builder: Callable[[int], bytes],
        label: str,
        on_seq_reserved: Callable[[int], None] | None = None,
    ) -> int | None:
        """Write one exact OFFLINE V2 command without legacy ID normalization."""
        client = self._client
        if client is None or not client.is_connected:
            self._log(f"[OFFLINE_V2][TX_ERR] command={label} reason=disconnected")
            return None
        if not self._central_business_ready() or not self._offline_v2_commands_ready:
            self._log(f"[OFFLINE_V2][TX_ERR] command={label} reason=host_ci_not_ready")
            return None
        if self._command_uuid is None or not self._ack_subscribed or not self._export_subscribed:
            self._log(f"[OFFLINE_V2][TX_ERR] command={label} reason=gatt_not_ready")
            return None

        seq = self._reserve_control_seq()
        if on_seq_reserved is not None:
            on_seq_reserved(seq)
        payload = bytes(builder(seq))
        if len(payload) != 20:
            raise ValueError(f"{label} command must be exactly 20 bytes")
        epoch, token = self._offline_v2_sync.epoch, self._connection_token
        await self._write_central_control_frame(
            payload, label=label, expected_client=client, expected_token=token,
            expected_guard=lambda: self._offline_v2_sync.epoch == epoch and
            self._offline_v2_sync.state != OfflineV2SyncState.WAIT_CAPTURE,
        )
        if self._offline_v2_sync.epoch != epoch or self._connection_token != token:
            return None
        self._log(f"[OFFLINE_V2][TX] command={label} seq={seq}")
        return seq

    def _central_business_ready(self) -> bool:
        client = self._client
        return bool(
            client is not None
            and client.is_connected
            and self._central_mode_enabled
            and self._central_session_state == "HOST_ACTIVE"
        )

    async def _wait_for_offline_balanced_restore(self) -> bool:
        if not self._central_mode_enabled:
            return False
        try:
            await asyncio.wait_for(
                self._central_balanced_applied_event.wait(),
                timeout=HOST_CI_ENABLE_SETTLE_TIMEOUT_SECONDS,
            )
        except asyncio.TimeoutError:
            self._log("[OFFLINE_V2][CI][ERR] balanced_restore_timeout=1")
            return False
        self._log("[OFFLINE_V2][CI] balanced_restore_applied=1")
        return True

    def _offline_v2_capture_in_progress(self) -> bool:
        return self._last_device_state in {
            DEVICE_STATE_OFFLINE_INTENT_READY,
            DEVICE_STATE_OFFLINE_CAPTURING,
            DEVICE_STATE_OFFLINE_FINALIZING,
        }

    def _device_version_code(self) -> int | None:
        info = self._connection_coordinator.device_info
        return info.code if info is not None else None

    def _legacy_offline_intent_bug_present(self) -> bool:
        code = self._device_version_code()
        return bool(
            code is not None
            and LEGACY_OFFLINE_INTENT_BUG_MIN_CODE <= code < LEGACY_OFFLINE_INTENT_FIX_CODE
        )

    def _legacy_offline_intent_recovery_required(self) -> bool:
        return bool(
            self._offline_v2_host_ci_deferred
            and self._offline_v2_host_ci_defer_detail
            == HOST_CI_ENABLE_DETAIL_DEVICE_NOT_IDLE
            and self._last_device_state == DEVICE_STATE_OFFLINE_INTENT_READY
            and self._legacy_offline_intent_bug_present()
        )

    def _offline_event_v2_supported(self) -> bool:
        info = self._connection_coordinator.device_info
        return bool(
            info is not None
            and info.offmeta in OFFLINE_MANIFEST_SUPPORTED_VERSIONS
        )

    def _offline_v2_observer_required(self) -> bool:
        return self._offline_v2_capture_in_progress() or (
            self._last_device_state == DEVICE_STATE_OFFLINE_SESSION_READY
        )

    async def _wait_for_central_enable_settled(self, deadline: float) -> bool:
        """Require two identical valid link observations before Host-CI enable."""
        stable_count = 0
        last_sample: tuple[int, int] | None = None
        loop = asyncio.get_running_loop()
        while loop.time() <= deadline:
            ci, latency = self._central_profiles.read_actual_units(self._client)
            sample = (ci, latency) if ci is not None and ci > 0 and latency is not None else None
            if sample is not None:
                if sample == last_sample:
                    stable_count += 1
                else:
                    if last_sample is not None:
                        self._log(
                            "[HOST_CI_CENTRAL] enable_settle_reset "
                            f"ci={ci} latency={latency}"
                        )
                    last_sample = sample
                    stable_count = 1
                if stable_count >= HOST_CI_ENABLE_SETTLE_SAMPLE_COUNT:
                    self._log(
                        "[HOST_CI_CENTRAL] enable_settled "
                        f"ci={ci} latency={latency} samples={stable_count}"
                    )
                    return True
            else:
                if stable_count:
                    self._log(
                        "[HOST_CI_CENTRAL] enable_settle_reset "
                        f"ci={ci} latency={latency}"
                    )
                stable_count = 0
                last_sample = None
            await asyncio.sleep(HOST_CI_ENABLE_SETTLE_SAMPLE_INTERVAL_SECONDS)
        return False

    async def _send_host_ci_mode_enable_attempt(
        self,
        *,
        session_id: int,
        windows_build: int,
        profile_signature: int,
        profile_mask: int,
        protocol_version: int,
        label: str,
    ) -> ZY100Ack:
        seq = self._reserve_control_seq()
        future: asyncio.Future[ZY100Ack] = self._loop.create_future()
        self._central_enable_seq = seq
        self._central_enable_future = future
        payload = build_host_ci_mode_enable_command(
            seq,
            session_id,
            windows_build,
            profile_signature=profile_signature,
            profile_mask=profile_mask,
            protocol_version=protocol_version,
        )
        try:
            await self._write_central_control_frame(payload, label=label)
            return await asyncio.wait_for(future, timeout=2.0)
        finally:
            if self._central_enable_seq == seq:
                self._central_enable_seq = None
                self._central_enable_future = None

    async def _retry_central_enable_connection(
        self,
        address: str,
        *,
        user_id: int,
        training_id: int,
    ) -> None:
        if self._central_enable_reconnect_used:
            return
        self._central_enable_reconnect_used = True
        self._central_enable_reconnect_requested = False
        self._log("[HOST_CI_CENTRAL] enable_settle_timeout action=controlled_reconnect attempt=1")
        self._emit(
            {
                "type": "connection_stage",
                "stage": "host_ci_reconnect",
                "message": "Waiting for a settled BLE link before Host-CI v2 retry",
            }
        )
        await self._safe_disconnect(emit=False)
        await asyncio.sleep(1.0)
        await self._connect(address, user_id=user_id, training_id=training_id)

    async def _fail_required_central_mode(self, context: str) -> None:
        self._central_enable_reconnect_requested = False
        self._log(
            "[HOST_CI_CENTRAL][ERR] required_not_ready "
            f"context={context} action=disconnect"
        )
        self._emit(
            {
                "type": "ci_mode",
                "mode": "HOST_REQUIRED_FAILED",
                "reason": context,
            }
        )
        self._emit(
            {
                "type": "error",
                "message": "Host-CI Central mode is required; the BLE connection was closed",
            }
        )
        await self._safe_disconnect(emit=False)

    def _note_central_enable_failure(self, reason: str) -> None:
        reconnect = not self._central_enable_reconnect_used
        self._central_enable_reconnect_requested = reconnect
        if self._connection_recovery.task is not None:
            self._central_enable_reconnect_requested = False
            definitive = reason.startswith("enable_rejected_") or reason in {
                "unsupported_windows_protocol_strategy", "not_winrt_backend",
            }
            raise ConnectionFailure("host_ci" if definitive else "initialization",
                                    f"Host-CI 初始化失败: {reason}")
        self._log(
            "[HOST_CI_CENTRAL][ERR] enable_failed "
            f"reason={reason} action={'controlled_reconnect' if reconnect else 'disconnect'}"
        )

    async def _enable_windows_central_mode(self) -> bool:
        if self._central_mode_enabled:
            return True
        self._central_enable_reconnect_requested = False
        strategy = self._connection_coordinator.strategy
        if strategy is None:
            raise ConnectionFailure("protocol", "尚未读取 Device Info，禁止推测 Host-CI 版本")
        protocol_version = strategy.host_ci_version
        if protocol_version not in (1, 3):
            self._note_central_enable_failure("unsupported_windows_protocol_strategy")
            return False
        self._connection_coordinator.transition(ConnectionStage.NEGOTIATING_LINK_POLICY)
        if WinRtExportSpeedAdapter._get_winrt_requester(self._client) is None:
            self._log("[HOST_CI_CENTRAL] unavailable reason=not_winrt_backend")
            self._note_central_enable_failure("not_winrt_backend")
            return False
        valid, specs, signature, reason = self._central_profiles.inspect_runtime_profiles()
        build = self._central_profiles.windows_build()
        self._log(
            "[HOST_CI_CENTRAL] "
            f"capability build={build} valid={int(valid)} signature=0x{signature:08X} "
            f"profiles={specs} reason={reason}"
        )
        if not valid or not self._ack_subscribed:
            self._emit(
                {
                    "type": "ci_mode",
                    "mode": "HOST_REQUIRED_FAILED",
                    "reason": reason,
                }
            )
            self._note_central_enable_failure(
                reason if not valid else "ack_notify_not_subscribed"
            )
            return False
        session_id = int(time.time_ns()) & 0xFFFFFFFF
        if session_id == 0:
            session_id = 1
        self._central_session_id = session_id
        self._central_link_intent_event.clear()
        self._central_early_link_ack = None
        self._central_applied_future = asyncio.get_running_loop().create_future()
        deadline = asyncio.get_running_loop().time() + HOST_CI_ENABLE_SETTLE_TIMEOUT_SECONDS
        if not await self._wait_for_central_enable_settled(deadline):
            self._note_central_enable_failure("link_not_settled")
            self._central_session_id = 0
            return False
        try:
            ack = await self._send_host_ci_mode_enable_attempt(
                session_id=session_id,
                windows_build=build,
                profile_signature=signature,
                profile_mask=HOST_PROFILE_MASK_ALL,
                protocol_version=protocol_version,
                label=f"HOST_CI_MODE_ENABLE_V{protocol_version}",
            )
        except Exception as exc:
            self._log(
                f"[HOST_CI_CENTRAL] v{protocol_version}_enable_failed reason={exc}"
            )
            self._note_central_enable_failure("enable_write_or_ack_failed")
            self._central_session_id = 0
            return False
        if ack.status != STATUS_OK:
            if (
                ack.status in {STATUS_NOT_READY, STATUS_BUSY}
                and ack.device_state
                in {
                    DEVICE_STATE_OFFLINE_INTENT_READY,
                    DEVICE_STATE_OFFLINE_CAPTURING,
                    DEVICE_STATE_OFFLINE_FINALIZING,
                }
            ):
                self._offline_v2_host_ci_deferred = True
                self._offline_v2_host_ci_defer_detail = ack.detail
                self._central_enable_reconnect_requested = False
                self._central_session_id = 0
                self._log(
                    "[OFF_OBS][DEFER] step=ci_balanced "
                    f"state=0x{ack.device_state:02X} status=0x{ack.status:02X} "
                    f"detail=0x{ack.detail:08X} action=keep_connected_battery_only"
                )
                self._emit(
                    {
                        "type": "ci_mode",
                        "mode": "DEFERRED_OFFLINE_CAPTURE",
                        "device_state": ack.device_state,
                    }
                )
                return False
            self._note_central_enable_failure(
                f"enable_rejected_0x{ack.status:02X}_0x{ack.detail:04X}"
            )
            self._log(
                f"[HOST_CI_CENTRAL][ERR] v{protocol_version}_rejected "
                f"status=0x{ack.status:02X} detail=0x{ack.detail:04X} "
                f"action={'controlled_reconnect' if self._central_enable_reconnect_requested else 'disconnect'}"
            )
            self._central_session_id = 0
            return False
        self._central_mode_enabled = True
        self._offline_v2_host_ci_deferred = False
        self._offline_v2_host_ci_defer_detail = 0
        self._central_hybrid_enabled = False
        self._central_session_state = "ENABLING"
        self._central_generation = ack.detail
        self._central_profile_key = None
        self._cancel_ci_observer()
        early_link_ack = self._central_early_link_ack
        self._central_early_link_ack = None
        if early_link_ack is not None:
            self._handle_central_link_notify(early_link_ack)
        try:
            await asyncio.wait_for(
                self._central_link_intent_event.wait(), timeout=0.5
            )
            self._log("[HOST_CI_CENTRAL] proactive_link_notify=1 query_skipped=1")
        except asyncio.TimeoutError:
            get_link_seq = self._reserve_control_seq()
            await self._write_central_control_frame(
                build_get_link_state_command(get_link_seq, session_id),
                label="GET_LINK_STATE",
            )
        try:
            applied = await asyncio.wait_for(
                asyncio.shield(self._central_applied_future),
                timeout=HOST_CI_ENABLE_SETTLE_TIMEOUT_SECONDS,
            )
        except (asyncio.TimeoutError, asyncio.CancelledError):
            applied = False
        if not applied:
            self._central_mode_enabled = False
            self._central_session_state = "TERMINAL"
            self._central_profiles.force_release(reason="initial_balanced_not_applied")
            self._note_central_enable_failure("initial_balanced_not_applied")
            self._central_session_id = 0
            return False
        self._central_session_state = "HOST_ACTIVE"
        self._start_central_heartbeat()
        self._emit(
            {
                "type": "ci_mode",
                "mode": "WINDOWS_CENTRAL",
                "hybrid_standby_exact": False,
                "session_id": session_id,
                "generation": ack.detail,
            }
        )
        self._emit(
            {
                "type": "connection_stage",
                "stage": "ci_balanced_applied",
                "message": "BALANCED CI applied; starting state synchronization",
            }
        )
        return True

    def _start_central_heartbeat(self) -> None:
        task = self._central_heartbeat_task
        if task is not None and not task.done():
            task.cancel()
        self._central_heartbeat_task = self._loop.create_task(
            self._central_heartbeat_loop()
        )

    def _stop_central_heartbeat(self) -> None:
        task = self._central_heartbeat_task
        if task is not None and not task.done():
            task.cancel()
        self._central_heartbeat_task = None

    async def _central_heartbeat_loop(self) -> None:
        try:
            while (
                self._central_mode_enabled
                and self._central_session_state == "HOST_ACTIVE"
            ):
                sleep_started = self._loop.time()
                await asyncio.sleep(5.0)
                wake_error_ms = max(0.0, (self._loop.time() - sleep_started - 5.0) * 1000.0)
                self._link_trace.record(
                    "heartbeat_wake",
                    wake_error_ms=wake_error_ms,
                    gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
                )
                if (
                    not self._central_mode_enabled
                    or self._central_session_state != "HOST_ACTIVE"
                ):
                    return
                seq = self._reserve_control_seq()
                payload = build_zy100_command(
                    CMD_PING,
                    seq=seq,
                    user_id=self._central_session_id,
                    training_id=0,
                    device_time_ms=int(time.time() * 1000),
                )
                self._link_trace.note_ping_sent(
                    seq,
                    gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
                )
                await self._write_central_control_frame(
                    payload, label="CENTRAL_LEASE_PING", log_tx=False
                )
        except asyncio.CancelledError:
            return
        except Exception as exc:
            self._log(f"[HOST_CI_CENTRAL] heartbeat_failed reason={exc}")

    async def _subscribe_export(self) -> None:
        self._connection_coordinator.transition(ConnectionStage.SUBSCRIBING_EXPORT)
        client = self._client
        if client is None or not client.is_connected:
            self._emit({"type": "error", "message": "No connected device"})
            return

        if self._export_subscribed:
            self._emit({"type": "export_subscription", "status": "subscribed", "uuid": self._export_uuid or ""})
            return

        if self._export_uuid is None:
            export_char = self._find_characteristic(client, ZY100_EXPORT_DATA_UUID)
            self._export_uuid = export_char.uuid.lower() if export_char is not None else None
        if self._export_uuid is None:
            self._emit({"type": "export_subscription", "status": "missing", "uuid": ""})
            self._emit({"type": "error", "message": f"Export Data characteristic not found: {ZY100_EXPORT_DATA_UUID}"})
            return

        try:
            callback = self._connection_coordinator.guard_callback(
                self._connection_token, self._handle_export_notification
            )
            await self._connection_coordinator.start_notify(
                self._connection_token, client, self._export_uuid, callback,
                name="subscribe_export",
            )
            self._export_subscribed = True
            self._log(f"[BLE_NOTIFY] export_subscribed=1 uuid={self._export_uuid}")
            self._emit({"type": "export_subscription", "status": "subscribed", "uuid": self._export_uuid})
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._export_subscribed = False
            self._log(f"Subscribe Export Data failed: {exc}")
            self._emit({"type": "export_subscription", "status": "failed", "uuid": self._export_uuid, "error": str(exc)})
            self._emit({"type": "error", "message": f"Export Data subscribe failed: {exc}"})

    async def _initialize_battery(self) -> None:
        client = self._client
        battery_char = self._battery_level_char
        if client is None or not client.is_connected:
            return
        if battery_char is None:
            detail = f"Battery Level characteristic not found: {BATTERY_LEVEL_UUID}"
            self._battery_subscribed = False
            self._log(f"[BLE_BATTERY] battery_subscribe_failed error={detail}")
            self._emit({"type": "battery_subscription", "status": "missing", "uuid": BATTERY_LEVEL_UUID})
            return

        await self._read_battery_level(client, battery_char)
        if not self._is_current_battery_target(client, battery_char):
            return
        await self._subscribe_battery(client, battery_char)

    async def _read_battery_level(
        self,
        client: BleakClient,
        battery_char: BleakGATTCharacteristic,
    ) -> None:
        try:
            raw = await self._connection_coordinator.read_gatt_char(
                self._connection_token, client, battery_char,
                priority=GattPriority.BACKGROUND,
                name="read_battery",
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"[BLE_BATTERY] battery_read_failed error={exc}")
            return

        if not self._is_current_battery_target(client, battery_char):
            return
        payload = bytes(raw)
        try:
            percent = self._parse_battery_level_payload(payload)
        except ValueError as exc:
            self._log(f"[BLE_BATTERY] battery_read_failed error={exc} raw={hex_bytes(payload)}")
            return

        uuid = battery_char.uuid.lower()
        self._battery_percent = percent
        self._log(f"[BLE_BATTERY] battery_read percent={percent}")
        self._emit({"type": "battery_level", "percent": percent, "source": "read", "uuid": uuid})

    async def _subscribe_battery(
        self,
        client: BleakClient,
        battery_char: BleakGATTCharacteristic,
    ) -> None:
        uuid = battery_char.uuid.lower()
        if self._battery_subscribed:
            self._emit({"type": "battery_subscription", "status": "subscribed", "uuid": uuid})
            return

        try:
            callback = self._connection_coordinator.guard_callback(
                self._connection_token, self._handle_battery_notification
            )
            await self._connection_coordinator.start_notify(
                self._connection_token, client, battery_char, callback,
                priority=GattPriority.BACKGROUND,
                name="subscribe_battery",
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._battery_subscribed = False
            self._log(f"[BLE_BATTERY] battery_subscribe_failed error={exc}")
            self._emit({"type": "battery_subscription", "status": "failed", "uuid": uuid, "error": str(exc)})
            return

        if not self._is_current_battery_target(client, battery_char):
            return
        self._battery_subscribed = True
        self._log(f"[BLE_BATTERY] battery_subscribed=1 uuid={uuid}")
        self._emit({"type": "battery_subscription", "status": "subscribed", "uuid": uuid})

    def _handle_battery_notification(self, sender: Any, data: bytearray) -> None:
        battery_char = self._battery_level_char
        if battery_char is None:
            return
        if sender is not battery_char:
            if isinstance(sender, BleakGATTCharacteristic):
                return
            sender_uuid = self._resolve_sender_uuid(sender, self._battery_level_uuid)
            if self._normalize_uuid(sender_uuid) != self._normalize_uuid(self._battery_level_uuid or ""):
                return

        payload = bytes(data)
        try:
            percent = self._parse_battery_level_payload(payload)
        except ValueError as exc:
            self._log(f"[BLE_BATTERY] battery_notify_invalid error={exc} raw={hex_bytes(payload)}")
            return

        uuid = battery_char.uuid.lower()
        self._battery_percent = percent
        self._log(f"[BLE_BATTERY] battery_notify percent={percent}")
        self._emit({"type": "battery_level", "percent": percent, "source": "notify", "uuid": uuid})

    def _parse_battery_level_payload(self, payload: bytes) -> int:
        if len(payload) != 1:
            raise ValueError(f"Battery Level payload length error: {len(payload)}")
        percent = payload[0]
        if percent < 0 or percent > 100:
            raise ValueError(f"Battery Level percent out of range: {percent}")
        return percent

    def _is_current_battery_target(
        self,
        client: BleakClient,
        battery_char: BleakGATTCharacteristic,
    ) -> bool:
        return (
            client is self._client
            and client.is_connected
            and battery_char is self._battery_level_char
            and self._normalize_uuid(battery_char.uuid) == BATTERY_LEVEL_UUID
        )

    async def _enter_export_high_speed(self, reason: str) -> BleSpeedSnapshot | None:
        client = self._client
        if client is None or not client.is_connected:
            return None
        if not self._central_business_ready():
            self._log(
                "[HOST_CI_CENTRAL][ERR] export_speed_blocked "
                f"action=enter reason={reason}"
            )
            return None
        self._log(
            "[HOST_CI_CENTRAL] "
            f"legacy_speed_skip=1 action=enter reason={reason}"
        )
        return None

    async def _restore_export_high_speed(self, reason: str) -> None:
        if not self._central_business_ready():
            self._log(
                "[HOST_CI_CENTRAL][ERR] export_speed_blocked "
                f"action=restore reason={reason}"
            )
            return
        self._log(
            "[HOST_CI_CENTRAL] "
            f"legacy_speed_skip=1 action=restore reason={reason}"
        )

    def _cancel_ci_observer(self) -> None:
        task = self._ci_observer_task
        self._ci_observer_task = None
        if (
            task is not None
            and not task.done()
            and task is not asyncio.current_task()
        ):
            task.cancel()

    def _start_ci_observer(self, client: BleakClient) -> None:
        self._cancel_ci_observer()
        self._ci_observer_task = self._loop.create_task(
            self._run_ci_observer(client)
        )

    async def _run_ci_observer(self, client: BleakClient) -> None:
        try:
            while self._client is client and client.is_connected:
                try:
                    self._ci_state.observe(
                        self._export_speed.read_link_snapshot(client),
                        source="periodic_link_poll",
                    )
                except Exception as exc:  # pragma: no cover - WinRT diagnostics
                    self._log(f"[HOST_CI_SM] observer_error reason={exc}")
                await asyncio.sleep(CI_OBSERVER_POLL_SECONDS)
        except asyncio.CancelledError:
            return
        finally:
            if self._ci_observer_task is asyncio.current_task():
                self._ci_observer_task = None

    async def _run_host_self_check_offline(self) -> None:
        try:
            await run_host_ble_self_check_offline(
                log=self._log,
                scan=False,
                write_report=True,
            )
        except Exception as exc:  # pragma: no cover - diagnostic path must not affect UI
            self._log(f"[HOST_BLE_SELFTEST] offline failed reason={exc}")

    def _schedule_host_self_check_online(self) -> None:
        if not self._host_self_check_enabled:
            return
        if self._host_self_check_online_task is not None and not self._host_self_check_online_task.done():
            return
        self._host_self_check_online_task = self._loop.create_task(self._run_host_self_check_online())

    async def _run_host_self_check_online(self) -> None:
        client = self._client
        if client is None or not client.is_connected:
            return
        try:
            report = await run_host_ble_self_check_online(
                client=client,
                state=self._host_self_check_state(),
                log=self._log,
                write_report=True,
            )
            capability = report.get("capability", {})
            if isinstance(capability, dict):
                self._host_ble_capability = dict(capability)
        except Exception as exc:  # pragma: no cover - diagnostic path must not affect export
            self._log(f"[HOST_BLE_SELFTEST] online failed reason={exc}")

    def _host_self_check_state(self) -> dict[str, Any]:
        client = self._client
        queue = self._export_pipeline.queue
        stats = self._export_pipeline.stats
        parser_active = self._export_parse_task is not None and not self._export_parse_task.done()
        return {
            "connected": bool(client is not None and client.is_connected),
            "service_ready": self._has_required_gatt(),
            "gatt_ready": self._is_gatt_ready(),
            "ack_subscribed": self._ack_subscribed,
            "export_subscribed": self._export_subscribed,
            "busy_reason": self._host_self_check_busy_reason(),
            "queue_capacity": getattr(queue, "maxsize", 0) or 0,
            "queue_depth": queue.qsize(),
            "parser_worker": "active" if parser_active else "inactive",
            "ui_throttle_ms": 200,
            "queue_overflow": stats.queue_overflow_count,
            "callback_avg_us": stats.callback_cost_us.avg,
            "mtu_effective": self._safe_mtu(client),
            "last_device_state": self._last_device_state if self._last_device_state is not None else "unknown",
        }

    def _host_self_check_busy_reason(self) -> str:
        client = self._client
        if client is None or not client.is_connected:
            return "no_connection"
        if not self._is_gatt_ready():
            return "gatt_not_ready"
        if self._pending_command_active:
            return "pending_command"
        if self._time_sync_ack_future is not None and not self._time_sync_ack_future.done():
            return "time_sync_pending"
        if self._pending_confirm_session_key is not None:
            return "confirm_reclaim_pending"
        if self._clear_flash_active:
            return "clear_active"
        if self._export_commands_blocked:
            return "export_or_reclaim_blocked"
        if self._export_receiver.receiving and not self._export_receiver.end_received:
            return "export_active"
        if self._export_receiver.failed and not self._export_receiver.end_received:
            return "export_failed_pending_end"
        if self._export_pipeline.queue.qsize() > 0:
            return "export_queue_pending"
        if self._failed_export_timeout_task is not None and not self._failed_export_timeout_task.done():
            return "failed_export_timeout_active"
        if self._last_device_state in {
            0x02,
            0x03,
            DEVICE_STATE_CLEARING_FLASH,
            DEVICE_STATE_EXPORT_READY,
            DEVICE_STATE_BLE_EXPORTING,
            DEVICE_STATE_BLE_EXPORT_WAIT_CONFIRM,
            DEVICE_STATE_ONLINE_STREAMING,
            DEVICE_STATE_ONLINE_END_WAIT_ACK,
        }:
            return f"device_state_0x{self._last_device_state:02X}"
        return ""

    def _start_online_diag(self, session_id: int) -> None:
        self._stop_online_diag()
        self._online_diag.start(session_id)
        self._online_diag_task = self._loop.create_task(
            self._online_diag_heartbeat()
        )

    def _stop_online_diag(self) -> None:
        task = self._online_diag_task
        try:
            current_task = asyncio.current_task()
        except RuntimeError:
            current_task = None
        if (
            task is not None
            and not task.done()
            and task is not current_task
        ):
            task.cancel()
        self._online_diag_task = None
        self._online_diag.stop()

    def _log_online_diag_final(self, reason: str) -> None:
        line = self._online_diag.final_line(
            reason=reason,
            queue_depth=self._export_pipeline.queue.qsize(),
        )
        if line is not None:
            self._log(line)

    async def _online_diag_heartbeat(self) -> None:
        previous_ns = time.perf_counter_ns()
        expected_ns = int(ONLINE_DIAG_HEARTBEAT_SECONDS * 1_000_000_000)
        try:
            while self._online_diag.active:
                await asyncio.sleep(ONLINE_DIAG_HEARTBEAT_SECONDS)
                now_ns = time.perf_counter_ns()
                self._online_diag.note_loop_slip(
                    max(0, now_ns - previous_ns - expected_ns) / 1_000_000.0
                )
                previous_ns = now_ns
                line = self._online_diag.periodic_line(
                    now_ns=now_ns,
                    queue_depth=self._export_pipeline.queue.qsize(),
                )
                if line is not None:
                    self._log(line)
        except asyncio.CancelledError:
            return
        finally:
            try:
                current_task = asyncio.current_task()
            except RuntimeError:
                current_task = None
            if self._online_diag_task is current_task:
                self._online_diag_task = None

    @staticmethod
    def _new_online_stats() -> dict[str, Any]:
        return {
            "raw_count": 0,
            "summary_count": 0,
            "event_count": 0,
            "stress_count": 0,
            "stress_bytes": 0,
            "stress_payload_bytes": 0,
            "stress_ack": 0,
            "raw_bytes": 0,
            "summary_bytes": 0,
            "event_bytes": 0,
            "raw_ack": 0,
            "summary_ack": 0,
            "event_ack": 0,
            "end_ack": 0,
            "packet_count": 0,
            "mag_sample_count": 0,
            "mag_read_error_count": 0,
            "mag_missed_deadline_count": 0,
            "last_record_type": "",
            "last_record_id": 0,
            "last_error": "",
            "bytes_total": 0,
        }

    @staticmethod
    def _online_stat_prefix(record_type: int) -> str:
        if record_type == ONLINE_RECORD_RAW:
            return "raw"
        if record_type == ONLINE_RECORD_SUMMARY:
            return "summary"
        if record_type == ONLINE_RECORD_EVENT:
            return "event"
        if record_type == ONLINE_RECORD_END:
            return "end"
        if record_type == RECORD_STRESS:
            return "stress"
        return f"type_{record_type:02x}"

    def _emit_online_status(self, status: str | None = None, **extra: Any) -> None:
        if status is not None:
            self._online_status = status
        elapsed = max(0.001, time.perf_counter() - self._online_started_perf) if self._online_started_perf else 0.0
        bytes_total = int(self._online_stats.get("bytes_total") or 0)
        payload: dict[str, Any] = {
            "type": "online_stream_status",
            "status": self._online_status,
            "ready_sent": self._online_ready_sent,
            "ready_ack_ok": self._online_ready_ack_ok,
            "active": self._online_active,
            "session_id": self._online_session_id or 0,
            "store_dir": str(self._online_store_state.session_dir or ""),
            "session_metadata": self._online_store_state.metadata,
            "bps": int(bytes_total / elapsed) if elapsed else 0,
            "summary_pending": len(self._online_summary_pending),
            "operation_id": self._online_start_operation_id,
            "attempt": self._online_start_attempt,
            "start_operation_state": self._online_start_operation_state,
            "start_operation_pending": (
                self._online_start_operation_state in START_OP_BUSY_STATES
            ),
        }
        payload.update(self._online_stats)
        payload.update(extra)
        self._emit(payload)

    def _set_offline_priority_blocked(
        self,
        blocked: bool,
        *,
        reason: str,
        phase: str | None = None,
    ) -> None:
        self._offline_priority_blocked = blocked
        self._export_commands_blocked = blocked
        if phase is None:
            phase = "syncing" if blocked else "ready"
        self._offline_priority_phase = phase
        messages = {
            "capturing": "正在离线采集中，其他操作已暂停",
            "finalizing": "离线采集正在规范收尾，其他操作已暂停",
            "syncing": "离线数据正在自动上传，其他操作已暂停",
            "recovery": "离线数据恢复失败，其他操作保持锁定",
            "ready": "离线数据已全部确认并回收，在线采集已开放",
        }
        self._emit(
            {
                "type": "offline_priority_gate",
                "active": blocked,
                "reason": reason,
                "phase": phase,
                "message": messages.get(
                    phase,
                    "离线数据处理中，其他操作已暂停"
                    if blocked
                    else "离线数据处理已完成",
                ),
            }
        )

    async def _run_offline_priority(self, reason: str) -> bool:
        if (
            isinstance(self._connection_coordinator.strategy, BootstrapV1Strategy)
            and not self._connection_user_context_supported
            and self._bootstrap_generation
            and (self._bootstrap_ready_bits & BOOTSTRAP_READY_OFFLINE_GATE)
        ):
            self._set_offline_priority_blocked(
                True,
                reason=f"{reason}_snapshot_clear",
                phase="syncing",
            )
            self._log(
                "[OFFLINE_V2][GATE] list_skipped=1 "
                f"generation={self._bootstrap_generation} ready_bits=0x{self._bootstrap_ready_bits:08X}"
            )
            return True
        if not self._offline_event_v2_supported():
            info = self._connection_coordinator.device_info
            actual = info.offmeta if info is not None else None
            self._set_offline_priority_blocked(
                True,
                reason=f"{reason}_unsupported_offmeta",
                phase="recovery",
            )
            self._log(
                "[OFFLINE_V2][GATE] sync_blocked=1 "
                "expected_offmeta="
                f"{sorted(OFFLINE_MANIFEST_SUPPORTED_VERSIONS)} actual={actual!r}"
            )
            self._emit(
                {
                    "type": "offline_v2_status",
                    "status": "unsupported_contract",
                    "expected_offmeta": sorted(OFFLINE_MANIFEST_SUPPORTED_VERSIONS),
                    "actual_offmeta": actual,
                    "message": "设备存在待上传离线数据，但上位机与设备离线格式不匹配",
                }
            )
            return False
        self._set_offline_priority_blocked(True, reason=f"{reason}_list")
        self._offline_v2_sync.local_priority_supported = self._offline_priority_supported()
        await self._offline_v2_sync.start_sync(reason=f"{reason}_offline_first")
        completed = await self._offline_v2_sync.wait_for_priority_completion()
        if not completed:
            self._log(
                f"[OFFLINE_V2][GATE] online remains blocked reason={reason} "
                f"state={self._offline_v2_sync.state.value}"
            )
            return False
        return True

    async def _run_offline_priority_then_online(self, reason: str) -> bool:
        if not await self._run_offline_priority(reason):
            return False
        online_ready = await self._send_online_stream_ready_if_possible(
            f"{reason}_after_offline",
            wait_ack=True,
            allow_offline_gate=True,
        )
        if online_ready and type(
            self._connection_coordinator.strategy
        ) is not BootstrapV1Strategy:
            self._set_offline_priority_blocked(
                False,
                reason=f"{reason}_complete",
                phase="ready",
            )
        return online_ready

    def _new_feature_config_transaction(self, config: FeatureConfig) -> _FeatureConfigTransaction:
        self._feature_config_transaction_id += 1
        return _FeatureConfigTransaction(
            self._connection_token, self._feature_config_transaction_id,
            self._client, self._connection_user_id, config, self._feature_config_ack_queue)

    def _feature_config_transaction_current(self, transaction: _FeatureConfigTransaction) -> bool:
        token, transaction_id, client, user_id, _config, queue = transaction
        return (token == self._connection_token
                and transaction_id == self._feature_config_transaction_id
                and client is self._client and client is not None and client.is_connected
                and user_id == self._connection_user_id == self._feature_config_user_id
                and queue is self._feature_config_ack_queue)

    def feature_config_event_current(self, event: dict[str, Any]) -> bool:
        return (event.get("connection_token") == self._connection_token
                and event.get("transaction_id") == self._feature_config_transaction_id
                and event.get("user_id") == (self._feature_config_user_id or 0))

    def _confirm_feature_config(self, config: FeatureConfig, user_id: int) -> None:
        self._feature_config_confirmed = {
            "user_id": user_id, "config": config.to_dict(),
            "crc32": config.crc32(user_id), "generation": self._feature_config_generation,
        }

    def _emit_feature_config_status(
        self,
        status: str,
        *,
        message: str = "",
        candidate: FeatureConfig | None = None,
        transaction: _FeatureConfigTransaction | None = None,
    ) -> None:
        if transaction is not None and not self._feature_config_transaction_current(transaction):
            return
        user_id = (transaction.user_id if transaction else self._feature_config_user_id) or 0
        info = self._connection_coordinator.device_info
        runtime_supported = bool(
            self._feature_config_supported and info and info.code is not None and info.code >= 10523)
        self._emit(
            {
                "type": "feature_config_status",
                "connection_token": transaction.connection_token if transaction else self._connection_token,
                "transaction_id": transaction.transaction_id if transaction else self._feature_config_transaction_id,
                "candidate": candidate.to_dict() if candidate is not None else None,
                "confirmed": self._feature_config_confirmed,
                "auto_capture_runtime_supported": runtime_supported,
                "hardware_apply_supported": bool(runtime_supported and info.code >= 10524),
                "status": status,
                "message": message,
                "supported": self._feature_config_supported,
                "synced": self._feature_config_synced,
                "pending": self._feature_config_pending,
                "user_id": user_id,
                "crc32": self._feature_config_crc32,
                "generation": self._feature_config_generation,
            }
        )

    def _schedule_feature_config_retry(
        self, reason: str, transaction: _FeatureConfigTransaction,
    ) -> None:
        if not self._feature_config_pending or not self._feature_config_supported:
            return
        if self._feature_config_retry_task is not None and not self._feature_config_retry_task.done():
            return

        retry_transaction = transaction

        async def retry() -> None:
            try:
                await asyncio.sleep(0.3)
                while self._feature_config_pending and self._feature_config_supported:
                    if not self._feature_config_transaction_current(retry_transaction):
                        return
                    client = self._client
                    if client is None or not client.is_connected:
                        return
                    if self._last_device_state == DEVICE_STATE_WAIT_START:
                        if await self._sync_feature_config(f"retry_{reason}", transaction=retry_transaction):
                            return
                    await asyncio.sleep(0.5)
            finally:
                if self._feature_config_retry_task is asyncio.current_task():
                    self._feature_config_retry_task = None

        self._feature_config_retry_task = self._loop.create_task(retry())

    async def _sync_feature_config(
        self,
        reason: str,
        *,
        config_override: FeatureConfig | None = None,
        persist_on_success: bool = False,
        transaction: _FeatureConfigTransaction | None = None,
    ) -> bool:
        transaction = transaction or self._new_feature_config_transaction(
            config_override or self._feature_config_snapshot)
        if not self._feature_config_transaction_current(transaction):
            return False
        token, _transaction_id, client, user_id, config, ack_queue = transaction
        reapply = reason == "ui_reapply_confirmed"
        manual = reason in ("ui_save_and_sync", "ui_reapply_confirmed")
        previous_synced = self._feature_config_synced
        apply_failed = False
        info = self._connection_coordinator.device_info
        self._feature_config_supported = bool(
            info is not None and info.fields.get("fcfg") == "1"
        )
        if not self._feature_config_supported:
            self._feature_config_synced = False
            self._feature_config_pending = False
            self._emit_feature_config_status(
                "unsupported", transaction=transaction,
                message="固件不支持配置同步，设备使用固件默认值",
            )
            return True
        if (
            user_id is None
            or self._feature_config_user_id != user_id
            or client is None
            or not client.is_connected
            or self._command_uuid is None
        ):
            self._feature_config_pending = not manual
            self._feature_config_synced = previous_synced if manual else False
            self._emit_feature_config_status(
                "sync_failed" if manual else "pending", transaction=transaction,
                message=(
                    "当前连接用户或 BLE 状态已变化，配置未发送且不会后台重试"
                    if manual
                    else "当前连接用户或 BLE 状态未就绪，配置待同步"
                ),
            )
            return False

        config_bytes = config.wire_bytes()
        config_crc32 = config.crc32(user_id)
        self._feature_config_crc32 = config_crc32
        deadline = asyncio.get_running_loop().time() + (
            2.0 if manual else 15.0
        )
        self._feature_config_pending = True
        self._emit_feature_config_status(
            "syncing", transaction=transaction, candidate=config,
            message=("正在同值重新应用，WOM 配置保持开启；等待完成回执"
                     if reapply else "正在保存并应用配置"))

        # Keep duplicate feature sync attempts serialized, while each actual
        # control write acquires the shared control-channel lock normally.
        # Reusing _control_command_lock here would self-deadlock because
        # _write_central_control_frame() acquires that non-reentrant lock too.
        async with self._feature_config_sync_lock:
            if not self._feature_config_transaction_current(transaction):
                return False
            while asyncio.get_running_loop().time() < deadline:
                if not self._feature_config_transaction_current(transaction):
                    return False
                while not ack_queue.empty():
                    try:
                        ack_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        break
                seq = self._reserve_control_seq()
                self._feature_config_seq = seq
                payload = build_feature_config_sync_command(
                    seq, user_id, config_bytes
                )
                if reapply:
                    self._log(
                        f"[FEATURE_REAPPLY] submit token={token} txn={_transaction_id} "
                        f"seq={seq} user={user_id} crc=0x{config_crc32:08X} "
                        f"wom={int(config.auto_capture)} config={config_bytes.hex()}")
                try:
                    written = await self._write_central_control_frame(
                        payload, label="FEATURE_CONFIG_SYNC",
                        expected_client=client, expected_token=token,
                        expected_guard=lambda: self._feature_config_transaction_current(transaction),
                        allow_write_fallback=not reapply,
                    )
                    if not self._feature_config_transaction_current(transaction):
                        return False
                    if written is False:
                        if reapply:
                            break
                        return False
                    ack = await asyncio.wait_for(
                        ack_queue.get(), timeout=3.0
                    )
                except asyncio.TimeoutError:
                    if reapply:
                        break
                    continue
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    self._log(f"[FEATURE_CFG][ERR] write_failed reason={exc}")
                    break
                if not self._feature_config_transaction_current(transaction):
                    return False
                if (
                    ack.seq_echo != seq
                    or ack.reserved != 1
                    or ack.user_id != user_id
                    or ack.training_id != config_crc32
                ):
                    self._log(
                        "[FEATURE_CFG][ERR] ack_contract "
                        f"seq={ack.seq_echo}/{seq} reserved={ack.reserved} "
                        f"owner={ack.user_id}/{user_id} crc=0x{ack.training_id:08X}"
                    )
                    break
                if ack.status == STATUS_BUSY or ack.detail == 0x4006:
                    if manual:
                        self._feature_config_pending = False
                        self._feature_config_synced = previous_synced
                        self._feature_config_crc32 = self._feature_config_snapshot.crc32(user_id)
                    self._emit_feature_config_status(
                        "busy_rejected" if manual else "pending", transaction=transaction,
                        message=(
                            "当前存在独占操作，请回到蓝牙连接空闲态后重新提交"
                            if manual
                            else "连接初始化等待设备空闲后继续配置同步"
                        ),
                    )
                    if manual:
                        if reapply:
                            self._log(f"[FEATURE_REAPPLY] result=busy txn={_transaction_id} seq={seq}")
                        return False
                    await asyncio.sleep(0.35)
                    continue
                if ack.status != STATUS_OK:
                    self._log(
                        f"[FEATURE_CFG][ERR] rejected status=0x{ack.status:02X} detail=0x{ack.detail:08X}"
                    )
                    break
                if ack.exec_mode == 0x01:
                    self._feature_config_generation = ack.detail
                    self._feature_config_synced = True
                    self._feature_config_pending = False
                    self._feature_config_snapshot = config
                    self._confirm_feature_config(config, user_id)
                    if persist_on_success:
                        try:
                            self._feature_config_store.save(user_id, config)
                        except OSError as exc:
                            self._emit_feature_config_status(
                                "profile_save_failed", transaction=transaction,
                                message=f"设备配置已确认，但本地档案保存失败：{exc}",
                            )
                            return True
                    self._emit_feature_config_status(
                        "matched", transaction=transaction,
                        message=(
                            "设备仅确认配置相同，未报告硬件重新应用；本次验证未完成"
                            if reapply else
                            "设备配置已匹配，本地档案已保存"
                            if persist_on_success
                            else "设备配置已匹配，无 Flash 写入"
                        ),
                    )
                    if reapply:
                        self._log(f"[FEATURE_REAPPLY] result=matched_only txn={_transaction_id} seq={seq}")
                    return True
                if ack.exec_mode != EXEC_MODE_ACCEPTED_ASYNC:
                    break
                try:
                    final_ack = await asyncio.wait_for(
                        ack_queue.get(), timeout=5.0
                    )
                except asyncio.TimeoutError:
                    if reapply:
                        break
                    continue
                if not self._feature_config_transaction_current(transaction):
                    return False
                if (
                    final_ack.seq_echo == seq
                    and final_ack.status == STATUS_OK
                    and final_ack.exec_mode == 0x04
                    and final_ack.reserved == 1
                    and final_ack.user_id == user_id
                    and final_ack.training_id == config_crc32
                ):
                    self._feature_config_generation = final_ack.detail
                    self._feature_config_synced = True
                    self._feature_config_pending = False
                    self._feature_config_snapshot = config
                    self._confirm_feature_config(config, user_id)
                    if persist_on_success:
                        try:
                            self._feature_config_store.save(user_id, config)
                        except OSError as exc:
                            self._emit_feature_config_status(
                                "profile_save_failed", transaction=transaction,
                                message=f"设备配置已确认，但本地档案保存失败：{exc}",
                            )
                            return True
                    self._emit_feature_config_status(
                        "synced", transaction=transaction,
                        message=(
                            "已重新应用已确认配置，WOM 配置保持开启；等待 PWRD_READY 后观察平台电流"
                            if reapply else
                            ("配置已保存、硬件已应用并保存本地档案" if info.code is not None and info.code >= 10524
                             else "功能配置已写入、回读确认并保存本地档案")
                            if persist_on_success
                            else ("配置已保存、硬件已应用" if info.code is not None and info.code >= 10524
                                  else "功能配置已写入并回读确认")
                        ),
                    )
                    if reapply:
                        self._log(f"[FEATURE_REAPPLY] result=applied txn={_transaction_id} seq={seq} crc=0x{config_crc32:08X}")
                    return True
                apply_failed = (final_ack.seq_echo == seq and final_ack.reserved == 1
                                and final_ack.user_id == user_id and final_ack.training_id == config_crc32
                                and final_ack.exec_mode == 0x04 and final_ack.status != STATUS_OK
                                and final_ack.detail == 0x4007)
                self._log(
                    "[FEATURE_CFG][ERR] async_failed "
                    f"status=0x{final_ack.status:02X} detail=0x{final_ack.detail:08X}"
                )
                break
        if not self._feature_config_transaction_current(transaction):
            return False
        if manual or apply_failed:
            self._feature_config_pending = False
            # Either ACK can be lost after a device commit. Keep the last
            # confirmation as history, but do not treat it as current readiness.
            self._feature_config_synced = False
            self._connection_business_ready = False
            self._feature_config_crc32 = self._feature_config_snapshot.crc32(user_id)
            if reapply:
                self._log(f"[FEATURE_REAPPLY] result={'apply_failed' if apply_failed else 'unconfirmed'} txn={_transaction_id}")
            self._emit_feature_config_status(
                "apply_failed" if apply_failed else "sync_failed", transaction=transaction,
                message=(("设备报告重新应用失败" if apply_failed else "重新应用结果未确认")
                         + "；未修改本地档案，不会自动重发，请保留日志后重新连接确认"
                         if reapply else
                         "配置已保存，但硬件应用未完成；设备将安全关机，本地档案未更新"
                         if apply_failed else "设备结果未确认，本地档案未更新；不会后台重试，请重新连接后确认配置"),
            )
            return False
        self._feature_config_synced = False
        self._feature_config_pending = True
        self._emit_feature_config_status(
            "pending", transaction=transaction, message="功能配置尚未确认，START 已锁定"
        )
        self._schedule_feature_config_retry(reason, transaction)
        return False

    async def _run_user_isolated_offline_then_online(
        self,
        reason: str,
        *,
        user_id: int,
        training_id: int,
    ) -> bool:
        if not await self._run_offline_priority(reason):
            return False
        if not await self._prompt_foreign_sessions_if_needed():
            return False
        if not await self._sync_rtc_after_notify(
            user_id=user_id,
            training_id=training_id,
        ):
            return False
        online_ready = await self._send_online_stream_ready_if_possible(
            f"{reason}_after_user_isolated_offline",
            wait_ack=True,
            allow_offline_gate=True,
        )
        if not online_ready:
            return False
        self._connection_coordinator.transition(
            ConnectionStage.SYNCING_FEATURE_CONFIG
        )
        return await self._sync_feature_config(reason)

    async def _send_online_stream_ready_if_possible(
        self,
        reason: str,
        *,
        wait_ack: bool = False,
        allow_offline_gate: bool = False,
    ) -> bool:
        client = self._client
        if client is None or not client.is_connected:
            return False
        if self._offline_priority_blocked and not allow_offline_gate:
            self._log(
                f"[OFFLINE_V2][GATE] ONLINE_STREAM_READY blocked reason={reason}"
            )
            return False
        if not self._is_calibration_business_ready():
            self._log(f"[CAL_GATE] ONLINE_STREAM_READY blocked state={self._calibration_gate_state}")
            return False
        if self._online_ready_sent and self._online_ready_ack_ok:
            return True
        if not self._is_gatt_ready() or not self._rtc_sync_ok:
            return False
        if self._command_uuid is None:
            return False

        capability = ONLINE_CAPABILITY_CURRENT | (CAP_STRESS if self.stress.config is not None else 0)
        seq = self._reserve_control_seq()
        host_time_ms = int(time.time() * 1000)
        payload = build_online_stream_ready_command(
            seq=seq,
            capability_mask=capability,
            host_time_ms=host_time_ms,
        )
        is_retry = reason.startswith("retry_")
        payload_hex = "" if is_retry else hex_bytes(payload)
        used_response = True
        self._online_ready_sent = True
        self._online_ready_ack_ok = False
        self._online_ready_seq = seq
        if self._online_ready_ack_future is not None and not self._online_ready_ack_future.done():
            self._online_ready_ack_future.cancel()
        ack_future: asyncio.Future[ZY100Ack] | None = None
        if wait_ack:
            ack_future = asyncio.get_running_loop().create_future()
            self._online_ready_ack_future = ack_future
        try:
            used_response = await self._write_central_control_frame(
                payload, label="ONLINE_STREAM_READY"
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            if self._online_ready_seq == seq:
                self._online_ready_sent = False
                self._online_ready_seq = None
            if self._online_ready_ack_future is ack_future:
                self._online_ready_ack_future = None
            self._log(f"[ONLINE_STATE] ready_write_failed reason={reason} error={exc}")
            self._emit_online_status("ready_failed", last_error=str(exc))
            return False

        if is_retry:
            self._log(
                "[BLE_TX] "
                f"ONLINE_STREAM_READY retry seq={seq} reason={reason} response={1 if used_response else 0}"
            )
        else:
            self._log(
                "[BLE_TX] "
                f"ONLINE_STREAM_READY seq={seq} capability=0x{capability:08X} "
                f"host_time_ms={host_time_ms} reason={reason} response={1 if used_response else 0} "
                f"payload={payload_hex}"
            )
        self._emit(
            {
                "type": "command_write_result",
                "ok": True,
                "cmd": CMD_ONLINE_STREAM_READY,
                "cmd_name": command_name(CMD_ONLINE_STREAM_READY),
                "seq": seq,
                "user_id": capability,
                "training_id": 0,
                "response": used_response,
                "payload_hex": payload_hex,
            }
        )
        self._emit_online_status("ready_sent")
        if not wait_ack or ack_future is None:
            return True
        try:
            ack = await asyncio.wait_for(ack_future, timeout=ONLINE_READY_ACK_TIMEOUT_SECONDS)
        except asyncio.TimeoutError:
            if self._online_ready_ack_future is ack_future:
                self._online_ready_ack_future = None
            self._online_ready_sent = False
            self._log(f"[ONLINE_STATE] ready_ack_timeout seq={seq} reason={reason}")
            self._emit_online_status("ready_ack_timeout")
            return False
        except asyncio.CancelledError:
            if self._online_ready_ack_future is ack_future:
                self._online_ready_ack_future = None
            raise
        if self._online_ready_ack_future is ack_future:
            self._online_ready_ack_future = None
        return ack.status == STATUS_OK

    async def _ensure_online_ready_before_start(self) -> bool:
        if self._offline_priority_blocked:
            self._emit(
                {
                    "type": "error",
                    "message": "离线数据尚未全部上传、确认并回收；在线采集未开放",
                }
            )
            return False
        was_ready = self._online_ready_sent and self._online_ready_ack_ok
        if was_ready:
            return True
        self._cancel_online_ready_retry()
        ready_ok = await self._send_online_stream_ready_if_possible("before_start_capture", wait_ack=True)
        if ready_ok:
            self._log("[ONLINE_STATE] prestart_ready_ok firmware_owns_async_preparation=1")
        return ready_ok

    def _cancel_online_start_final_timeout(self) -> None:
        task = self._online_start_final_timeout_task
        if (
            task is not None
            and not task.done()
            and task is not asyncio.current_task()
        ):
            task.cancel()
        self._online_start_final_timeout_task = None

    def _cancel_offline_start_final_timeout(self) -> None:
        task = self._offline_start_final_timeout_task
        if (
            task is not None
            and not task.done()
            and task is not asyncio.current_task()
        ):
            task.cancel()
        self._offline_start_final_timeout_task = None

    async def _offline_start_final_timeout(self, seq: int) -> None:
        try:
            await asyncio.sleep(OFFLINE_START_FINAL_ACK_TIMEOUT_SECONDS)
            if self._offline_start_seq != seq:
                return
            self._offline_start_seq = None
            self._log(
                f"[OFFLINE_CONTROL][ERR] start_final_timeout seq={seq} "
                "capture_may_continue=1 reconnect_observer=1"
            )
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "离线开始最终 ACK 超时；设备可能仍会继续采集，"
                        "请观察状态或重连恢复 Observer"
                    ),
                }
            )
        except asyncio.CancelledError:
            return
        finally:
            current = asyncio.current_task()
            if self._offline_start_final_timeout_task is current:
                self._offline_start_final_timeout_task = None

    def _cancel_online_start_ci48_restore_wait(self) -> None:
        task = self._online_start_ci48_restore_task
        if (
            task is not None
            and not task.done()
            and task is not asyncio.current_task()
        ):
            task.cancel()
        self._online_start_ci48_restore_task = None

    async def _online_start_final_timeout(self, seq: int) -> None:
        try:
            await asyncio.sleep(START_FINAL_ACK_TIMEOUT_SECONDS)
            if self._online_start_seq != seq:
                return
            self._online_active = False
            self._log(
                f"[ONLINE_STATE][ERR] start_final_timeout seq={seq} "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt}; "
                "disconnect_required=1 auto_retry=0"
            )
            self._emit_online_status("start_final_timeout", start_seq=seq)
            self._emit(
                {
                    "type": "error",
                    "message": (
                        f"START final ACK timed out after "
                        f"{START_FINAL_ACK_TIMEOUT_SECONDS:.0f} seconds; the BLE "
                        "link will be rebuilt only after a manual retry"
                    ),
                }
            )
            self._finish_online_start_operation(reason="final_ack_timeout")
            await self._safe_disconnect()
        except asyncio.CancelledError:
            return
        finally:
            current = asyncio.current_task()
            if self._online_start_final_timeout_task is current:
                self._online_start_final_timeout_task = None

    async def _wait_for_ci48_after_start_failure(self) -> None:
        deadline = time.monotonic() + START_CI48_RESTORE_TIMEOUT_SECONDS
        stable_samples = 0
        try:
            # Do not accept the snapshot captured by the failure handler.  The
            # old CI9 LLCP may still complete immediately after that transient
            # CI48 observation.
            await asyncio.sleep(CI_OBSERVER_POLL_SECONDS)
            while time.monotonic() < deadline:
                client = self._client
                if client is None or not client.is_connected:
                    self._finish_online_start_operation(
                        reason="ci48_restore_link_lost",
                    )
                    return
                snapshot = self._export_speed.read_link_snapshot(client)
                actual = self._ci_state.observe(
                    snapshot,
                    source="ci9_not_ready_restore_wait",
                )
                if actual == HostCiState.ACTIVE_IDLE:
                    stable_samples += 1
                    self._log(
                        "[START_OP] firmware_ci48_restore_sample "
                        f"op={self._online_start_operation_id} "
                        f"attempt={self._online_start_attempt} "
                        f"stable={stable_samples}/"
                        f"{START_CI48_STABLE_SAMPLE_COUNT}"
                    )
                    if stable_samples >= START_CI48_STABLE_SAMPLE_COUNT:
                        self._log(
                            "[START_OP] firmware_ci48_restore_stable "
                            f"op={self._online_start_operation_id} "
                            f"attempt={self._online_start_attempt}"
                        )
                        self._finish_online_start_operation(
                            reason="ci48_restore_stable",
                        )
                        return
                else:
                    if stable_samples > 0:
                        self._log(
                            "[START_OP] firmware_ci48_restore_reset "
                            f"op={self._online_start_operation_id} "
                            f"attempt={self._online_start_attempt} "
                            f"observed={actual.value}"
                        )
                    stable_samples = 0
                await asyncio.sleep(CI_OBSERVER_POLL_SECONDS)
            self._log(
                "[START_OP][ERR] ci48_restore_stability_timeout "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt} "
                "disconnect_required=1 auto_retry=0"
            )
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "CI 回滚在 25 秒内未稳定到 CI48，连接已重置；"
                        "请重新连接后重试 START"
                    ),
                }
            )
            await self._safe_disconnect()
            client = self._client
            if (
                (client is None or not client.is_connected)
                and self._online_start_operation_state != START_OP_IDLE
            ):
                self._finish_online_start_operation(
                    reason="ci48_restore_stability_timeout",
                )
            elif client is not None and client.is_connected:
                self._log(
                    "[START_OP][ERR] ci48_restore_disconnect_failed "
                    f"op={self._online_start_operation_id} "
                    f"attempt={self._online_start_attempt} "
                    "operation_lock_retained=1"
                )
        except asyncio.CancelledError:
            return
        finally:
            current = asyncio.current_task()
            if self._online_start_ci48_restore_task is current:
                self._online_start_ci48_restore_task = None

    def _handle_online_start_failure(self, ack: ZY100Ack, reason: str) -> None:
        self._cancel_online_start_final_timeout()
        stress_start = self.stress.running or self.stress.config is not None
        if self.stress.running or self.stress.config is not None:
            self.stress.terminal("failed", message=f"START 失败：{reason}")
        self._online_start_seq = None
        self._online_active = False
        self._online_session_id = None
        if reason == "ci9_not_ready":
            snapshot = self._export_speed.read_link_snapshot(self._client)
            self._ci_state.transition(
                HostCiState.ACTIVE_IDLE,
                reason="firmware_ci9_not_ready_restore",
                snapshot=snapshot,
            )
            self._log(
                "[ONLINE_STATE][CI_DIAG] "
                f"ci9_not_ready seq={ack.seq_echo}; "
                "disconnect_required=0 connection_preserved=1 retry_allowed=1"
            )

        reconnect_required = reason == "ci_llcp_stuck"
        disconnect_required = reason != "ci9_not_ready"
        self._log(
            "[ONLINE_STATE][ERR] "
            f"start_final_failed reason={reason} seq={ack.seq_echo} "
            f"status=0x{ack.status:02X} exec=0x{ack.exec_mode:02X} "
            f"state=0x{ack.device_state:02X} detail=0x{ack.detail:08X}; "
            f"disconnect_required={1 if disconnect_required else 0} "
            f"op={self._online_start_operation_id} "
            f"attempt={self._online_start_attempt}"
        )
        self._emit_online_status("start_final_failed", failure_reason=reason, start_ack=ack.to_dict())
        if not disconnect_required:
            self._set_online_start_operation_state(
                START_OP_TERMINAL,
                reason="ci9_not_ready_wait_ci48",
            )
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "START failed (ci9_not_ready); connection preserved, "
                        "wait for ACTIVE_IDLE restoration and retry"
                    ),
                }
            )
            self._cancel_online_start_ci48_restore_wait()
            self._online_start_ci48_restore_task = self._loop.create_task(
                self._wait_for_ci48_after_start_failure()
            )
            return
        if reconnect_required and not stress_start and not self._online_start_ci_recovery_used:
            self._online_start_ci_recovery_used = True
            self._set_online_start_operation_state(
                START_OP_RECOVERING,
                reason="ci_llcp_stuck_first_attempt",
            )
            self._log(
                "[CI_RECOVERY] "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt} reason=llcp_stuck "
                "action=disconnect_reconnect retry_limit=1"
            )
            self._emit_online_status(
                "start_ci_recovery",
                operation_id=self._online_start_operation_id,
                attempt=self._online_start_attempt,
                failure_reason=reason,
            )
            self._online_start_ci_recovery_task = self._loop.create_task(
                self._recover_online_start_after_ci_stuck(ack)
            )
            return
        if reconnect_required:
            self._log(
                "[CI_RECOVERY][ERR] "
                f"op={self._online_start_operation_id} attempt=2 "
                "reason=llcp_stuck action=disconnect_stop retry_limit_reached=1"
            )
            self._emit(
                {
                    "type": "error",
                    "message": (
                        "START recovery failed: connection-parameter LLCP "
                        "stuck again; automatic retry limit reached"
                    ),
                }
            )
            self._finish_online_start_operation(
                reason="ci_llcp_stuck_retry_limit",
            )
            self._loop.create_task(self._safe_disconnect())
            return
        self._emit({"type": "error", "message": f"START failed ({reason}); reconnect before retry"})
        self._finish_online_start_operation(reason=f"start_failed_{reason}")
        self._loop.create_task(self._safe_disconnect())

    async def _recover_online_start_after_ci_stuck(self, ack: ZY100Ack) -> None:
        address = self._connected_address
        user_id = ack.user_id if ack.user_id > 0 else (self._active_user_id or 1)
        training_id = ack.training_id if ack.training_id > 0 else (self._active_training_id or 1)
        self._online_start_recovery_preserve = True
        scan_device: BLEDevice | None = None
        try:
            if not address:
                raise RuntimeError("device address unavailable")
            await self._safe_disconnect(
                emit=False,
                preserve_start_operation=True,
            )
            await asyncio.sleep(0.35)
            for scan_attempt in range(1, self._ci_recovery_scan_attempts + 1):
                self._log(
                    "[CI_RECOVERY] "
                    f"op={self._online_start_operation_id} "
                    f"attempt={self._online_start_attempt} "
                    f"scan_attempt={scan_attempt}/"
                    f"{self._ci_recovery_scan_attempts} stage=scan_start"
                )
                scan_device = await self._scan_for_device_by_address(
                    address,
                    timeout=self._gatt_hard_reconnect_scan_s,
                )
                self._log(
                    "[CI_RECOVERY] "
                    f"op={self._online_start_operation_id} "
                    f"attempt={self._online_start_attempt} "
                    f"scan_attempt={scan_attempt}/"
                    f"{self._ci_recovery_scan_attempts} "
                    f"device_seen={1 if scan_device is not None else 0}"
                )
                if scan_device is not None:
                    break
                if scan_attempt < self._ci_recovery_scan_attempts:
                    await asyncio.sleep(self._ci_recovery_scan_retry_delay_s)
            if scan_device is None:
                raise RuntimeError(
                    "device was not found after "
                    f"{self._ci_recovery_scan_attempts} recovery scans"
                )
            self._devices[address] = scan_device
            self._log(
                "[CI_RECOVERY] "
                f"op={self._online_start_operation_id} attempt=2 "
                f"stage=scan_found address={address} user_id={user_id} "
                f"training_id={training_id}"
            )
            await self._connect(address, user_id=user_id, training_id=training_id)
            client = self._client
            if (
                client is None
                or not client.is_connected
                or not self._is_gatt_ready()
                or not self._is_calibration_business_ready()
                or not self._rtc_sync_ok
            ):
                raise RuntimeError("reconnect prerequisites not ready")
            self._log(
                "[CI_RECOVERY] "
                f"op={self._online_start_operation_id} attempt=2 "
                "stage=reconnect_ready retry_start=1"
            )
            await self._send_zy100_command(
                cmd=CMD_START_CAPTURE,
                user_id=user_id,
                training_id=training_id,
                device_time_ms=None,
                _ci_recovery_retry=True,
            )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._log(
                "[CI_RECOVERY][ERR] "
                f"op={self._online_start_operation_id} "
                f"attempt={self._online_start_attempt} failed reason={exc}"
            )
            self._emit_online_status("start_ci_recovery_failed", last_error=str(exc))
            self._emit(
                {
                    "type": "error",
                    "message": f"START recovery reconnect failed: {exc}",
                }
            )
            self._finish_online_start_operation(
                reason="ci_recovery_failed",
            )
            await self._safe_disconnect()
        finally:
            self._online_start_recovery_preserve = False
            current = asyncio.current_task()
            if self._online_start_ci_recovery_task is current:
                self._online_start_ci_recovery_task = None

    def _cancel_online_ready_retry(self) -> None:
        if self._online_ready_retry_task is not None and not self._online_ready_retry_task.done():
            self._online_ready_retry_task.cancel()
        self._online_ready_retry_task = None

    def _schedule_online_ready_retry(self, reason: str) -> None:
        if self._online_ready_ack_ok:
            return
        if self._client is None or not self._client.is_connected:
            return
        if self._online_ready_retry_task is not None and not self._online_ready_retry_task.done():
            return
        self._online_ready_retry_task = self._loop.create_task(self._online_ready_retry_once(reason))

    async def _online_ready_retry_once(self, reason: str) -> None:
        try:
            while not self._online_ready_ack_ok:
                client = self._client
                if client is None or not client.is_connected:
                    return
                await asyncio.sleep(ONLINE_READY_RETRY_DELAY_SECONDS)
                if self._online_ready_ack_ok:
                    return
                await self._send_online_stream_ready_if_possible(f"retry_{reason}", wait_ack=True)
        except asyncio.CancelledError:
            return
        finally:
            current = asyncio.current_task()
            if self._online_ready_retry_task is current:
                self._online_ready_retry_task = None

    async def _send_online_record_ack(
        self,
        *,
        record_type: int,
        record_id: int,
        ack_count: int,
        status: int = 0,
    ) -> bool:
        client = self._client
        session_id = self._online_session_id
        epoch = self._online_io_epoch
        if record_type != ONLINE_RECORD_END and not self._online_active:
            return False
        if client is None or not client.is_connected or self._command_uuid is None or session_id is None:
            self._emit_online_status("ack_failed", last_error="not connected for online ACK")
            return False

        seq = self._reserve_control_seq()
        payload = build_online_record_ack_command(
            seq=seq,
            session_id=session_id,
            record_type=record_type,
            record_id=record_id,
            ack_count=ack_count,
            status=status,
        )
        used_response = True
        if self._online_saved_ns:
            self._online_diag.note_saved_to_ack((time.perf_counter_ns() - self._online_saved_ns) / 1e6)
            self._online_saved_ns = 0
        ack_started_ns = time.perf_counter_ns()
        try:
            used_response = await self._write_central_control_frame(
                payload, label="ONLINE_RECORD_ACK", expected_client=client,
                expected_token=self._connection_token,
                expected_guard=lambda: (self._client is client and self._online_io_epoch == epoch
                                        and self._online_session_id == session_id),
            )
        except Exception as exc:  # pragma: no cover - host stack dependent
            if client is not self._client or epoch != self._online_io_epoch:
                return False
            self._online_diag.note_ack_write(
                (time.perf_counter_ns() - ack_started_ns) / 1_000_000.0,
                fallback=not used_response,
            )
            self._log(
                "[ONLINE_ACK][ERR] "
                f"type={online_record_type_name(record_type)} id={record_id} count={ack_count} error={exc}"
            )
            self._emit_online_status("ack_failed", last_error=str(exc))
            self._log_online_exception("ack_send", exc, record_id)
            return False

        if client is not self._client or epoch != self._online_io_epoch or session_id != self._online_session_id:
            return False
        self._online_diag.note_ack_write(
            (time.perf_counter_ns() - ack_started_ns) / 1_000_000.0,
            fallback=not used_response,
        )
        prefix = self._online_stat_prefix(record_type)
        ack_key = f"{prefix}_ack"
        if ack_key in self._online_stats:
            self._online_stats[ack_key] = int(self._online_stats.get(ack_key) or 0) + max(1, ack_count)
        metadata_started_ns = time.perf_counter_ns()
        try:
            await self._online_store_call("note_ack", record_type, ack_count)
        except Exception as exc:
            self._log_online_exception("ack_metadata", exc, record_id)
            if self._online_metadata_retry_task is None or self._online_metadata_retry_task.done():
                self._online_metadata_retry_task = self._loop.create_task(self._retry_online_metadata())
        self._online_diag.note_ack_metadata(
            (time.perf_counter_ns() - metadata_started_ns) / 1_000_000.0
        )
        self._emit_online_status("ack_sent")
        return True

    @staticmethod
    def _safe_mtu(client: Any | None) -> str:
        if client is None:
            return "unknown"
        try:
            return str(getattr(client, "mtu_size"))
        except Exception:
            return "unknown"

    def _ensure_export_parse_task(self) -> None:
        if self._export_parse_task is None or self._export_parse_task.done():
            self._export_parse_task = self._loop.create_task(self._export_parse_worker())

    def _cancel_export_parse_task(self) -> None:
        if self._export_parse_task is not None and not self._export_parse_task.done():
            self._export_parse_task.cancel()
        self._export_parse_task = None

    async def _export_parse_worker(self) -> None:
        try:
            while True:
                item = await self._export_pipeline.queue.get()
                try:
                    await self._parse_export_notification(item)
                finally:
                    self._export_pipeline.queue.task_done()
        except asyncio.CancelledError:
            return

    @property
    def _online_store_state(self):
        return self._online_storage.snapshot(self._online_store)

    async def _online_store_call(self, method: str, *args, **kwargs):
        store, epoch, client = self._online_store, self._online_io_epoch, self._client
        future = self._online_storage.submit(store, method, *args, **kwargs)
        result = await self._online_storage.wait(future)
        if store is not self._online_store or epoch != self._online_io_epoch or client is not self._client:
            raise asyncio.CancelledError("stale online storage completion")
        self._online_diag.note_storage(result.queue_ms, result.work_ms)
        if method == "save_record":
            self._online_saved_ns = result.finished_ns
        if result.error is not None:
            raise result.error
        return result.value

    def _queue_online_abort(self, reason: str, **kwargs) -> None:
        store = self._online_store
        if store in self._online_abort_tasks:
            return
        try:
            # Submit now: the abort is ordered behind any running write and
            # before a later session's START, even if this waiter is cancelled.
            future = self._online_storage.submit(store, "abort", reason, **kwargs)
        except Exception as exc:
            self._log_online_exception("abort_submit", exc)
            return

        async def complete() -> None:
            try:
                result = await self._online_storage.wait(future)
                if result.error is not None:
                    self._log(f"[ONLINE_EXCEPTION] stage=abort_save path={self._online_storage.snapshot(store).session_dir} reason={result.error}")
                snapshot = self._online_storage.snapshot(store)
                self._emit({"type": "online_storage_settled", "ok": result.error is None,
                            "stress_report": snapshot.metadata.get("stress_report"),
                            "stress_verdict": snapshot.metadata.get("stress_verdict"),
                            "session_id": snapshot.metadata.get("session_id")})
            finally:
                self._online_abort_tasks.pop(store, None)

        self._online_abort_tasks[store] = self._loop.create_task(complete())

    async def _drain_online_storage(self) -> None:
        await self._online_storage.drain()
        tasks = tuple(self._online_abort_tasks.values())
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def _log_online_exception(self, stage: str, exc: Exception, record_id: int | None = None) -> None:
        path = getattr(exc, "path", self._online_store_state.session_dir)
        self._log(
            f"[ONLINE_EXCEPTION] stage={stage} type={type(exc).__name__} "
            f"sid={self._online_session_id} rid={record_id} path={path} reason={exc}"
        )
        for line in traceback.format_exception(type(exc), exc, exc.__traceback__):
            for part in line.rstrip().splitlines():
                self._log(f"[ONLINE_EXCEPTION][TRACE] {part}")

    def _cancel_online_fault_tasks(self) -> None:
        try:
            current = asyncio.current_task()
        except RuntimeError:
            current = None
        for task in (self._online_metadata_retry_task, self._online_fault_task):
            if task is not None and task is not current and not task.done():
                task.cancel()
        self._online_metadata_retry_task = None
        self._online_fault_task = None
        self._online_fault_reason = ""
        self._online_fault_can_drain = False
        self._online_fault_terminal.clear()

    async def _retry_online_metadata(self) -> None:
        client, session = self._client, self._online_store_state.session_dir
        for attempt in range(1, 4):
            await asyncio.sleep(1.0)
            if (client is not self._client or session != self._online_store_state.session_dir
                    or self._online_fault_reason or not self._online_store_state.metadata_dirty):
                return
            try:
                await self._online_store_call("flush_metadata")
                self._log(f"[ONLINE_METADATA] retry={attempt} saved=1 path={session}")
                return
            except Exception as exc:
                self._log_online_exception("ack_metadata_retry", exc)
                if attempt == 3:
                    self._begin_online_fault("ack_metadata_exhausted", exc)

    def _begin_online_fault(self, stage: str, exc: Exception, record_id: int | None = None) -> None:
        self._log_online_exception(stage, exc, record_id)
        if self._online_fault_reason:
            if stage != "ack_metadata_exhausted":
                self._online_fault_can_drain = False
            return
        self._cancel_failed_export_timeout()
        self._online_fault_reason = f"{stage}: {exc}"
        self._online_fault_can_drain = stage == "ack_metadata_exhausted"
        self._online_fault_terminal.clear()
        if self._online_fault_can_drain:
            # All data was saved and ACKed. Keep draining the normal STOP tail.
            self._online_stats["last_error"] = self._online_fault_reason
            self._queue_online_abort(self._online_fault_reason, terminal_source="host_online_fault")
            self._emit_online_status("error", last_error=self._online_fault_reason)
        else:
            self._handle_online_error(self._online_fault_reason, terminal_source="host_online_fault")
        self._online_fault_task = self._loop.create_task(self._stop_faulted_online())

    async def _stop_faulted_online(self) -> None:
        client, session = self._client, self._online_store_state.session_dir
        if client is None or not client.is_connected:
            return
        try:
            async def stop_and_wait() -> None:
                payload = build_zy100_command(
                    cmd=CMD_PAUSE_CAPTURE, seq=self._reserve_control_seq(),
                    user_id=self._active_user_id or 0,
                    training_id=self._active_training_id or 0, device_time_ms=None,
                )
                await self._write_central_control_frame(payload, label="ONLINE_FAULT_STOP")
                if self._online_fault_can_drain:
                    self._schedule_online_summary_partial_ack(force=True)
                await self._online_fault_terminal.wait()

            await asyncio.wait_for(stop_and_wait(), timeout=10.0)
            self._log("[ONLINE_FAULT] device terminal received; failed session retained")
            self._online_fault_reason = ""
            self._online_fault_can_drain = False
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._log_online_exception("fault_stop", exc)
            if client is self._client and session == self._online_store_state.session_dir:
                self._log("[ONLINE_FAULT] safe stop incomplete; disconnecting (online 10s limit)")
                await self._safe_disconnect()

    async def _parse_export_notification(self, item: QueuedExportNotification) -> None:
        parse_started_ns = time.perf_counter_ns()
        # Online failures must never enter the legacy EXPORT_END watchdog.
        online_hint = len(item.payload) >= 3 and is_online_export_frame_type(item.payload[2])
        if online_hint or self._online_active or self._online_fault_reason:
            try:
                frame = parse_export_frame(item.payload)
            except ValueError as exc:
                self._begin_online_fault("frame_parse", exc)
                return
            if not is_online_export_frame_type(frame.frame_type):
                self._begin_online_fault("frame_dispatch", ValueError("non-online frame during online session"))
                return
            self._online_diag.note_queue_wait((parse_started_ns - item.received_perf_ns) / 1_000_000.0)
            self._online_diag.note_parse((time.perf_counter_ns() - parse_started_ns) / 1_000_000.0)
            try:
                await self._process_online_notification(item, frame)
            except Exception as exc:
                self._begin_online_fault("online_processing", exc)
            return
        if len(item.payload) >= 3 and is_offline_v2_frame_type(item.payload[2]):
            stage = "frame_parse"
            try:
                if (self._offline_v2_sync.local_priority_supported and
                        item.received_perf_ns < self._offline_v2_sync.notification_floor_ns):
                    return
                offline_frame = parse_offline_outer_frame(item.payload)
                stage = "processing"
                await self._offline_v2_sync.on_frame(offline_frame)
            except Exception as exc:
                self._export_pipeline.stats.note_parser_error()
                self._log(f"[OFFLINE_V2][PROTOCOL_ERR] stage={stage} type={type(exc).__name__} reason={exc}")
                self._emit(
                    {
                        "type": "offline_v2_status",
                        "status": "error",
                        "state": self._offline_v2_sync.state.value,
                        "error_category": "protocol_error",
                        "reason": str(exc),
                    }
                )
            return
        try:
            frame = parse_export_frame(item.payload)
            if frame.frame_type in {0x01, 0x02, 0x03, 0x04}:
                self._log(
                    "[OFFLINE_V2][LEGACY_UNSUPPORTED] "
                    f"frame_type=0x{frame.frame_type:02X} len={len(item.payload)}"
                )
                self._emit(
                    {
                        "type": "offline_v2_status",
                        "status": "legacy_unsupported",
                        "state": self._offline_v2_sync.state.value,
                        "reason": "旧离线 FEUF 协议不受 v1.1.55 运行态支持",
                    }
                )
                return
            raise ValueError(f"unknown Export Data frame type: 0x{frame.frame_type:02X}")
        except ValueError as exc:
            self._export_pipeline.stats.note_parser_error()
            self._export_receiver.failed = True
            self._export_receiver.fail_reason = str(exc)
            self._export_commands_blocked = True
            self._log(f"[BLE_EXPORT_ERR] reason={exc} raw={hex_bytes(item.payload)}")
            self._emit_transfer_status(
                "transfer_failed",
                uuid=item.uuid,
                reason=str(exc),
            )
            await self._restore_export_high_speed("parse_error")
            self._log_export_perf_summary("fail")
            self._ensure_failed_export_timeout()
            return

    def _handle_export_notification(self, sender: Any, data: bytearray) -> None:
        callback_started_ns = time.perf_counter_ns()
        received_wall_ms = int(time.time() * 1000)
        payload = bytes(data)
        uuid = self._resolve_sender_uuid(sender, self._export_uuid)
        callback_cost_us = (time.perf_counter_ns() - callback_started_ns) / 1000.0
        enqueued = self._export_pipeline.enqueue(
            uuid=uuid,
            payload=payload,
            callback_cost_us=callback_cost_us,
            received_perf_ns=callback_started_ns,
            received_wall_ms=received_wall_ms,
        )
        self._online_diag.note_callback(
            len(payload),
            callback_started_ns,
            self._export_pipeline.queue.qsize(),
        )
        if not enqueued:
            if self._online_active or self._online_session_id is not None:
                self._begin_online_fault("notify_queue_overflow", RuntimeError("host online notify queue overflow"))
                return
            self._export_commands_blocked = True
            if not self._export_pipeline.stats.overflow_reported:
                self._export_pipeline.stats.overflow_reported = True
                self._log(
                    "[BLE_EXPORT_ERR] "
                    f"reason=notify_queue_overflow overflow={self._export_pipeline.stats.queue_overflow_count}"
                )
                self._emit_transfer_status(
                    "transfer_failed",
                    uuid=uuid,
                    reason="host notify queue overflow",
                )
                self._loop.create_task(self._restore_export_high_speed("queue_overflow"))
            return
        self._ensure_export_parse_task()

    async def _process_online_notification(self, item: QueuedExportNotification, frame: Any) -> None:
        if not self._online_active and frame.frame_type in {0x12, 0x13}:
            if not (frame.frame_type == 0x13 and self._online_store_state.metadata.get("status") in {"ending", "ended"}):
                return  # Late queued records cannot reopen an aborted session.
        if self._online_fault_reason:
            if frame.frame_type == 0x14:  # Existing ONLINE_ABORT terminal frame.
                self._handle_online_error(self._online_fault_reason, terminal_source="host_online_fault")
                await self._drain_online_storage()
                self._online_fault_terminal.set()
                return
            if not self._online_fault_can_drain or frame.frame_type == 0x11:
                return  # Never ACK data after a failed Record save or replace the failed session.
        try:
            event = self._online_receiver.on_notify(item.payload)
        except ValueError as exc:
            self._begin_online_fault("receiver", exc)
            return
        if event is None:
            return
        if event.kind == "start" and event.start is not None:
            self._start_online_diag(event.start.session_id)
        self._online_diag.note_frame(frame.frame_type, frame.chunk_seq)
        if event.fragment is not None:
            self._online_diag.note_fragment(
                event.fragment.record_id,
                event.fragment.offset,
            )
        elif event.record is not None:
            self._online_diag.note_fragment(
                event.record.record_id,
                event.record.record_bytes,
            )
        await self._process_online_event(event, item.uuid, frame.frame_type)

    async def _process_online_event(self, event: OnlineReceiverEvent, uuid: str, frame_type: int) -> None:
        if event.kind == "start" and event.start is not None:
            self._online_io_epoch += 1
            if self._online_store_state.session_dir is not None:
                if self._online_store_state.metadata.get("status") in {"active", "ending"}:
                    self._queue_online_abort("superseded_online_start", terminal_source="host_online_fault")
                old = self._online_store
                self._online_store = OnlineSessionStore(old.root, calibration_store=old.calibration_store)
            self._cancel_online_summary_ack_tasks()
            self._cancel_failed_export_timeout()
            self._cancel_online_fault_tasks()
            self._online_active = True
            self._online_session_id = event.start.session_id
            self._online_summary_ack_batch = event.start.summary_ack_batch or ONLINE_SUMMARY_ACK_BATCH_DEFAULT
            self._online_summary_pending = []
            self._online_started_perf = time.perf_counter()
            self._online_stats = self._new_online_stats()
            session_dir = await self._online_store_call("start",
                event.start,
                device_name=self._current_device_name,
                device_address=self._connected_address or "",
                capability_mask=ONLINE_CAPABILITY_CURRENT | (CAP_STRESS if self.stress.config else 0),
                stress_config=self.stress.config,
                calibration=self._calibration_session_snapshot(),
                calibration_record=(
                    self._latest_calibration_record.raw
                    if self._calibration_gate_state == "valid"
                    and self._latest_calibration_record is not None
                    else None
                ),
            )
            self._log(
                "[ONLINE_STATE] "
                f"start session_id={event.start.session_id} user_id={event.start.user_id} "
                f"training_id={event.start.training_id} ack_timeout_ms={event.start.ack_timeout_ms} "
                f"summary_batch={self._online_summary_ack_batch} store={session_dir}"
            )
            self.stress.on_start(event.start.session_id)
            self._emit_online_status("streaming", start=event.start.to_dict())
            return

        if event.kind == "stress" and event.stress is not None:
            try:
                await self._online_store_call("note_stress_status", event.stress)
            except Exception as exc:
                self._begin_online_fault("stress_status_save", exc)
                return
            elapsed = max(0.001, time.perf_counter() - self._online_started_perf)
            self.stress.emit("sample", sample=event.stress,
                receive_Bps=int(self._online_stats.get("stress_payload_bytes", 0) / elapsed))
            return

        if event.kind == "fragment" and event.fragment is not None:
            self._emit_online_status(
                "receiving",
                last_record_type=online_record_type_name(event.fragment.record_type),
                last_record_id=event.fragment.record_id,
            )
            return

        if event.kind == "record" and event.record is not None:
            await self._handle_online_completed_record(event.record)
            return

        if event.kind == "end" and event.end is not None:
            if event.end.session_id != self._online_session_id:
                self._begin_online_fault("terminal_session", ValueError("ONLINE_END session mismatch"))
                return
            stored_end = self._online_store_state.metadata.get("end")
            duplicate_end = (
                self._online_store_state.metadata.get("status") in {"ending", "ended"}
                and isinstance(stored_end, dict)
                and int(stored_end.get("session_id") or 0) == event.end.session_id
            )
            if duplicate_end:
                time_analysis = dict(
                    self._online_store_state.metadata.get("time_analysis")
                    or self._online_store_state.metadata.get("rtc_analysis")
                    or {}
                )
                payload_matches = stored_end == event.end.to_dict()
                self._log(
                    "[ONLINE_STATE] "
                    f"duplicate_end session_id={event.end.session_id} "
                    f"payload_matches={1 if payload_matches else 0}; ack_again=1"
                )
            else:
                try:
                    time_analysis = await self._online_store_call("stage_end", event.end)
                except Exception as exc:
                    self._begin_online_fault("terminal_save", exc)
                    return
            self._online_summary_pending.clear()
            self._cancel_online_summary_ack_tasks()
            self._online_stats["last_record_type"] = "END"
            self._online_stats["last_record_id"] = 0
            self._online_stats["packet_count"] = int(
                self._online_store_state.metadata.get("packet_count") or 0
            )
            self._online_stats["mag_sample_count"] = int(
                self._online_store_state.metadata.get("mag_sample_count") or 0
            )
            self._online_stats["mag_read_error_count"] = int(
                self._online_store_state.metadata.get("mag_read_error_count") or 0
            )
            self._online_stats["mag_missed_deadline_count"] = int(
                self._online_store_state.metadata.get("mag_missed_deadline_count") or 0
            )
            self._log(
                "[ONLINE_STATE] "
                f"end session_id={event.end.session_id} stop_reason={event.end.stop_reason} "
                f"produced={event.end.produced_raw}/{event.end.produced_summary}/{event.end.produced_event} "
                f"acked={event.end.acked_raw}/{event.end.acked_summary}/{event.end.acked_event} "
                f"packets={self._online_stats['packet_count']} "
                f"fixed40_time_valid={1 if time_analysis.get('valid') else 0}"
            )
            self._emit_online_status(
                "ending",
                end=event.end.to_dict(),
                time_analysis=time_analysis,
                uuid=uuid,
                duplicate=duplicate_end,
            )
            # The device may return to WAIT_START as soon as it receives END ACK,
            # while our storage worker is still committing metadata/report files.
            closing = (self._online_io_epoch, self._online_session_id, self._online_store)
            self._online_end_finalizing = closing
            try:
                ack_ok = await self._send_online_record_ack(
                    record_type=ONLINE_RECORD_END,
                    record_id=0,
                    ack_count=1,
                )
                if not ack_ok:
                    self._emit_online_status(
                        "end_ack_failed",
                        end=event.end.to_dict(),
                        time_analysis=time_analysis,
                        uuid=uuid,
                        duplicate=duplicate_end,
                        last_error="ONLINE_END ACK write failed",
                    )
                    return
                try:
                    await self._online_store_call("commit_end")
                except Exception as exc:
                    self._begin_online_fault("terminal_commit", exc)
                    return
            finally:
                if self._online_end_finalizing is closing:
                    self._online_end_finalizing = None
            self._online_speed_skip_until = time.monotonic() + 3.0
            self._log_online_diag_final("end")
            self._stop_online_diag()
            self._online_active = False
            await self._restore_export_high_speed("online_end_idle")
            if self._online_fault_reason:
                self._handle_online_error(self._online_fault_reason, terminal_source="host_online_fault")
                await self._drain_online_storage()
                self._online_fault_terminal.set()
                return
            if event.end.stress is not None:
                self.stress.terminal("ended", terminal=event.end.stress,
                    verdict=self._online_store_state.metadata.get("stress_verdict"),
                    report=self._online_store_state.metadata.get("stress_report"))
            self._emit_online_status(
                "end",
                end=event.end.to_dict(),
                time_analysis=time_analysis,
                uuid=uuid,
                duplicate=duplicate_end,
            )
            return

        if event.kind in {"failed", "abort"}:
            if event.stress is not None:
                try:
                    await self._online_store_call("note_stress_status", event.stress, terminal=True)
                except Exception as exc:
                    self._log(f"[STRESS] terminal status save failed: {exc}")
            if event.kind == "failed":
                self._begin_online_fault("receiver", ValueError(event.reason or event.kind))
                return
            self._handle_online_error(
                event.reason or event.kind,
                terminal_source=(
                    "online_abort" if event.kind == "abort" else "online_receiver"
                ),
            )
            return

        self._log(f"[ONLINE_STATE] ignored event={event.kind} frame_type=0x{frame_type:02X}")

    async def _handle_online_completed_record(self, record: OnlineCompletedRecord) -> None:
        save_started_ns = time.perf_counter_ns()
        try:
            await self._online_store_call("save_record", record)
        except Exception as exc:
            self._online_diag.note_save(
                (time.perf_counter_ns() - save_started_ns) / 1_000_000.0,
                profile=self._online_store_state.profile,
            )
            self._begin_online_fault("record_save", exc, record.record_id)
            return
        self._online_diag.note_save(
            (time.perf_counter_ns() - save_started_ns) / 1_000_000.0,
            profile=self._online_store_state.profile,
        )

        prefix = self._online_stat_prefix(record.record_type)
        count_key = f"{prefix}_count"
        bytes_key = f"{prefix}_bytes"
        self._online_stats[count_key] = int(self._online_stats.get(count_key) or 0) + 1
        self._online_stats[bytes_key] = int(self._online_stats.get(bytes_key) or 0) + record.record_bytes
        if record.stress is not None:
            self._online_stats["stress_payload_bytes"] = int(self._online_stats.get("stress_payload_bytes") or 0) + record.stress.data_bytes
        self._online_stats["bytes_total"] = int(self._online_stats.get("bytes_total") or 0) + record.record_bytes
        self._online_stats["last_record_type"] = online_record_type_name(record.record_type)
        self._online_stats["last_record_id"] = record.record_id
        self._online_stats["packet_count"] = int(
            self._online_store_state.metadata.get("packet_count") or 0
        )
        self._online_stats["mag_sample_count"] = int(
            self._online_store_state.metadata.get("mag_sample_count") or 0
        )
        self._online_stats["mag_read_error_count"] = int(
            self._online_store_state.metadata.get("mag_read_error_count") or 0
        )
        self._online_stats["mag_missed_deadline_count"] = int(
            self._online_store_state.metadata.get("mag_missed_deadline_count") or 0
        )
        if record.record_type in (ONLINE_RECORD_RAW, ONLINE_RECORD_EVENT, RECORD_STRESS):
            await self._send_online_record_ack(record_type=record.record_type, record_id=record.record_id, ack_count=1)
            return

        self._emit_online_status("record", record=record.to_dict())

        if record.record_type == ONLINE_RECORD_SUMMARY:
            if max(1, self._online_summary_ack_batch) <= 1:
                await self._send_online_record_ack(
                    record_type=ONLINE_RECORD_SUMMARY,
                    record_id=record.record_id,
                    ack_count=1,
                )
                return
            self._online_summary_pending.append(record)
            if len(self._online_summary_pending) >= max(1, self._online_summary_ack_batch):
                self._schedule_online_summary_full_ack()
            return

        self._handle_online_error(f"unsupported online record type: {record.record_type}")

    def _schedule_online_summary_full_ack(self) -> None:
        if self._online_summary_ack_task is not None and not self._online_summary_ack_task.done():
            return
        self._online_summary_ack_task = self._loop.create_task(self._online_summary_full_ack_after_delay())

    async def _online_summary_full_ack_after_delay(self) -> None:
        need_full = False
        try:
            await asyncio.sleep(ONLINE_SUMMARY_FULL_ACK_DELAY_S)
            batch_size = max(1, self._online_summary_ack_batch)
            records = list(self._online_summary_pending[:batch_size])
            if not records:
                return
            ok = await self._send_online_record_ack(
                record_type=ONLINE_RECORD_SUMMARY,
                record_id=records[-1].record_id,
                ack_count=len(records),
            )
            if ok:
                del self._online_summary_pending[:len(records)]
                if len(self._online_summary_pending) >= batch_size:
                    need_full = True
        except asyncio.CancelledError:
            return
        finally:
            self._online_summary_ack_task = None
        if need_full:
            self._schedule_online_summary_full_ack()

    def _schedule_online_summary_partial_ack(self, *, force: bool = False) -> None:
        if not self._online_summary_pending:
            return
        if not force:
            return
        if len(self._online_summary_pending) >= max(1, self._online_summary_ack_batch):
            self._schedule_online_summary_full_ack()
            return
        delay = ONLINE_SUMMARY_FULL_ACK_DELAY_S if force else ONLINE_SUMMARY_PARTIAL_RETRY_S
        if self._online_summary_partial_task is not None and not self._online_summary_partial_task.done():
            if not force:
                return
            self._loop.create_task(self._online_summary_partial_ack_once(delay))
            return
        self._online_summary_partial_task = self._loop.create_task(self._online_summary_partial_ack_loop(delay))

    async def _online_summary_partial_ack_once(self, delay: float) -> None:
        try:
            await asyncio.sleep(delay)
            if self._online_summary_pending and len(self._online_summary_pending) < max(1, self._online_summary_ack_batch):
                records = list(self._online_summary_pending)
                await self._send_online_record_ack(
                    record_type=ONLINE_RECORD_SUMMARY,
                    record_id=records[-1].record_id,
                    ack_count=len(records),
                )
        except asyncio.CancelledError:
            return

    async def _online_summary_partial_ack_loop(self, delay: float) -> None:
        try:
            await asyncio.sleep(delay)
            while self._online_summary_pending and len(self._online_summary_pending) < max(1, self._online_summary_ack_batch):
                records = list(self._online_summary_pending)
                await self._send_online_record_ack(
                    record_type=ONLINE_RECORD_SUMMARY,
                    record_id=records[-1].record_id,
                    ack_count=len(records),
                )
                await asyncio.sleep(ONLINE_SUMMARY_PARTIAL_RETRY_S)
        except asyncio.CancelledError:
            return
        finally:
            self._online_summary_partial_task = None

    def _cancel_online_summary_ack_tasks(self) -> None:
        if self._online_summary_ack_task is not None and not self._online_summary_ack_task.done():
            self._online_summary_ack_task.cancel()
        if self._online_summary_partial_task is not None and not self._online_summary_partial_task.done():
            self._online_summary_partial_task.cancel()
        self._online_summary_ack_task = None
        self._online_summary_partial_task = None

    def _handle_online_error(
        self,
        reason: str,
        raw: bytes | None = None,
        *,
        terminal_source: str = "host_error",
        device_state: int | None = None,
    ) -> None:
        self._log_online_diag_final(reason)
        self._stop_online_diag()
        if self.stress.running or self.stress.config is not None:
            self.stress.terminal("failed", message=reason)
        self._online_speed_skip_until = time.monotonic() + 3.0
        self._online_active = False
        self._online_summary_pending.clear()
        self._cancel_online_summary_ack_tasks()
        self._online_stats["last_error"] = reason
        self._online_io_epoch += 1
        self._queue_online_abort(reason, terminal_source=terminal_source, device_state=device_state)
        self._ci_state.transition(
            HostCiState.ACTIVE_IDLE,
            reason=f"online_error:{reason}",
            snapshot=self._export_speed.read_link_snapshot(self._client),
        )
        raw_part = f" frame_bytes={len(raw)}" if raw else ""
        self._log(f"[ONLINE_STATE][ERR] reason={reason}{raw_part}")
        self._emit_online_status("error", last_error=reason)

    async def _process_export_event(self, event: ExportReceiverEvent, uuid: str) -> None:
        if event.kind == "start" and event.start is not None:
            self._cancel_failed_export_timeout()
            if (
                self._pending_confirm_session_key
                and self._pending_confirm_ack_ok
                and not self._pending_confirm_final_ack_ok
            ):
                await self._mark_reclaim_ok_for_session(
                    self._pending_confirm_session_key,
                    self._pending_confirm_export_id or 0,
                    via="next_export_start",
                    device_state=DEVICE_STATE_BLE_EXPORTING,
                    device_ready=False,
                )
            self._reset_pending_confirm_context()
            self._export_commands_blocked = True
            self._last_transfer_progress_emit_ms = 0
            self._log(
                "[BLE_EXPORT] "
                f"start export_id={event.start.export_id} session_count={event.start.session_count} "
                f"estimated_total_bytes={event.start.estimated_total_bytes} estimate_only=1"
            )
            self._emit_transfer_status(
                "receiving",
                uuid=uuid,
                export_id=event.start.export_id,
                session_count=event.start.session_count,
                estimated_total_bytes=event.start.estimated_total_bytes,
                bytes_received=0,
                chunk_count=0,
            )
            return

        if event.kind == "data":
            start = self._export_receiver.start
            now_ms = int(time.time() * 1000)
            if now_ms - self._last_transfer_progress_emit_ms < 200:
                return
            self._last_transfer_progress_emit_ms = now_ms
            self._emit_transfer_status(
                "receiving",
                uuid=uuid,
                export_id=start.export_id if start else 0,
                estimated_total_bytes=start.estimated_total_bytes if start else 0,
                bytes_received=event.bytes_received,
                chunk_count=event.chunk_count,
            )
            return

        if event.kind in {"failed", "abort"}:
            self._export_commands_blocked = True
            self._emit_transfer_status(
                "transfer_failed",
                uuid=uuid,
                reason=event.reason,
                bytes_received=event.bytes_received,
                chunk_count=event.chunk_count,
            )
            self._ensure_failed_export_timeout()
            await self._restore_export_high_speed(event.kind)
            self._log_export_perf_summary("fail")
            return

        if event.kind == "end" and event.end is not None:
            self._cancel_failed_export_timeout()
            self._log(
                "[BLE_EXPORT] "
                f"end export_id={event.end.export_id} total_bytes={event.end.total_bytes} "
                f"stream_crc32=0x{event.end.stream_crc32:08X} chunks={event.end.chunk_count} "
                f"received_bytes={event.bytes_received} received_chunks={event.chunk_count}"
            )
            end_ok = bool(event.ok)
            end_reason = event.reason
            if self._export_pipeline.stats.queue_overflow_count > 0:
                end_ok = False
                self._export_pipeline.stats.crc_ok = False
                end_reason = (
                    "host notify queue overflow "
                    f"count={self._export_pipeline.stats.queue_overflow_count}"
                )
            elif self._export_pipeline.stats.parse_error_count > 0:
                end_ok = False
                self._export_pipeline.stats.crc_ok = False
                end_reason = f"host parse error count={self._export_pipeline.stats.parse_error_count}"

            if not end_ok:
                self._log(f"[BLE_EXPORT_ERR] end validation failed: {end_reason}")
                self._emit_transfer_status(
                    "transfer_failed",
                    uuid=uuid,
                    export_id=event.end.export_id,
                    reason=end_reason,
                    bytes_received=event.bytes_received,
                    chunk_count=event.chunk_count,
                )
                await self._send_export_confirm(event.end.export_id, result=1, session_key=None)
                self._export_receiver.reset()
                return

            self._emit_transfer_status(
                "verifying",
                uuid=uuid,
                export_id=event.end.export_id,
                total_bytes=event.end.total_bytes,
                stream_crc32=event.end.stream_crc32,
                chunk_count=event.end.chunk_count,
            )
            stream = self._export_receiver.get_stream()
            try:
                parsed = parse_feuf_stream(stream)
            except ValueError as exc:
                self._export_pipeline.stats.note_parser_error()
                self._export_pipeline.stats.crc_ok = False
                self._log(f"[BLE_EXPORT_ERR] FEUF parse failed: {exc}")
                self._emit_transfer_status(
                    "transfer_failed",
                    uuid=uuid,
                    export_id=event.end.export_id,
                    reason=f"FEUF parse failed: {exc}",
                )
                await self._send_export_confirm(event.end.export_id, result=1, session_key=None)
                self._export_receiver.reset()
                return

            self._export_pipeline.stats.note_feuf_stats(
                payload_max=parsed.manifest.payload_max if parsed.manifest else None,
                data_payload_lengths=[
                    len(frame.payload)
                    for frame in parsed.frames
                    if frame.frame_type == FEUF_FRAME_DATA
                ],
            )
            self._emit_transfer_status("saving_local", uuid=uuid, export_id=event.end.export_id)
            start = self._export_receiver.start
            file_write_started_ns = time.perf_counter_ns()
            try:
                save_result = await asyncio.to_thread(
                    self._session_store.save_or_reuse_completed_export,
                    feuf_bytes=stream,
                    export_id=event.end.export_id,
                    total_bytes=event.end.total_bytes,
                    stream_crc32=event.end.stream_crc32,
                    chunk_count=event.end.chunk_count,
                    parsed=parsed,
                    device_name=self._current_device_name,
                    device_address=self._connected_address or "",
                    session_count=start.session_count if start else 1,
                    estimated_total_bytes=start.estimated_total_bytes if start else 0,
                )
                self._export_pipeline.stats.add_file_write_ms(
                    (time.perf_counter_ns() - file_write_started_ns) / 1_000_000.0
                )
            except Exception as exc:
                self._export_pipeline.stats.add_file_write_ms(
                    (time.perf_counter_ns() - file_write_started_ns) / 1_000_000.0
                )
                self._log(f"[BLE_EXPORT_ERR] local save failed: {exc}")
                self._emit_transfer_status(
                    "transfer_failed",
                    uuid=uuid,
                    export_id=event.end.export_id,
                    reason=f"local save failed: {exc}",
                )
                await self._send_export_confirm(event.end.export_id, result=1, session_key=None)
                self._export_receiver.reset()
                return

            self._emit({"type": "sessions_changed"})
            self._emit_transfer_status(
                "saved_local",
                uuid=uuid,
                export_id=event.end.export_id,
                session_key=save_result.session_key,
                duplicate=save_result.duplicate,
            )
            await self._send_export_confirm(event.end.export_id, result=0, session_key=save_result.session_key)
            self._export_receiver.reset()

    def _log_export_perf_summary(self, result: str) -> None:
        client = self._client
        if client is not None:
            self._export_pipeline.stats.update_ble_snapshot(self._export_speed.read_link_snapshot(client))
        else:
            self._export_pipeline.stats.update_ble_snapshot(self._export_speed.snapshot)
        if self._host_ble_capability:
            self._export_pipeline.stats.capability_level = str(
                self._host_ble_capability.get("capability_level", "unknown")
            )
        for line in self._export_pipeline.stats.summary_lines(result=result):
            self._log(line)

    async def _send_export_confirm(self, export_id: int, *, result: int, session_key: str | None) -> None:
        client = self._client
        if client is None or not client.is_connected or self._command_uuid is None:
            if session_key:
                await self._mark_confirm_write_failed_for_session(
                    session_key,
                    export_id,
                    reason="confirm_write_no_connection",
                )
                self._log_export_perf_summary("fail")
                await self._restore_export_high_speed("confirm_write_no_connection")
                return
            self._emit_transfer_status("transfer_failed", export_id=export_id, reason="cannot send EXPORT_CONFIRM")
            self._log_export_perf_summary("fail")
            await self._restore_export_high_speed("confirm_write_no_connection")
            return

        seq = self._reserve_control_seq()
        payload = build_export_confirm_command(seq=seq, export_id=export_id, result=result)
        used_response = True
        self._pending_command_active = True
        try:
            try:
                used_response = await self._write_central_control_frame(
                    payload, label="EXPORT_CONFIRM"
                )
            except Exception as exc:
                self._log(f"Write EXPORT_CONFIRM failed: {exc}")
                if session_key:
                    await self._mark_confirm_write_failed_for_session(
                        session_key,
                        export_id,
                        reason=f"confirm_write_failed: {exc}",
                    )
                    self._log_export_perf_summary("fail")
                    await self._restore_export_high_speed("confirm_write_failed")
                    return
                self._emit_transfer_status("transfer_failed", export_id=export_id, reason=f"confirm write failed: {exc}")
                self._log_export_perf_summary("fail")
                await self._restore_export_high_speed("confirm_write_failed")
                return
        finally:
            self._pending_command_active = False

        self._export_pipeline.stats.note_confirm_send()
        self._log(
            "[BLE_TX] "
            f"EXPORT_CONFIRM seq={seq} export_id={export_id} result={result} "
            f"response={1 if used_response else 0} payload={hex_bytes(payload)}"
        )
        self._emit(
            {
                "type": "command_write_result",
                "ok": True,
                "cmd": CMD_EXPORT_CONFIRM,
                "cmd_name": command_name(CMD_EXPORT_CONFIRM),
                "seq": seq,
                "user_id": export_id,
                "training_id": result,
                "response": used_response,
                "payload_hex": hex_bytes(payload),
            }
        )
        self._log_export_perf_summary("ok" if result == 0 else "fail")
        wait_for_final_reclaim_ack = bool(result == 0 and session_key)
        if not wait_for_final_reclaim_ack:
            await self._restore_export_high_speed("export_confirm_sent")

        if wait_for_final_reclaim_ack:
            self._pending_confirm_session_key = session_key
            self._pending_confirm_export_id = export_id
            self._pending_confirm_seq = seq
            self._pending_confirm_ack_ok = False
            self._pending_confirm_final_ack_ok = False
            self._export_commands_blocked = True
            confirm_started_at = now_iso()
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "confirm_sent": True,
                    "confirm_ack_status": "pending",
                    "reclaim_status": "",
                    "reclaim_verified": False,
                    "last_confirm_at": confirm_started_at,
                },
            )
            self._log(f"[BLE_CONFIRM] pending export_id={export_id} session={session_key} confirm_seq={seq}")
            self._emit({"type": "sessions_changed"})
            self._emit_transfer_status("confirm_sent", export_id=export_id, session_key=session_key)

    def _ensure_failed_export_timeout(self) -> None:
        if self._failed_export_timeout_task is None or self._failed_export_timeout_task.done():
            self._failed_export_timeout_task = self._loop.create_task(self._disconnect_failed_export_after_timeout())

    def _cancel_failed_export_timeout(self) -> None:
        self._export_failure_epoch += 1
        try:
            current_task = asyncio.current_task()
        except RuntimeError:
            current_task = None
        if (
            self._failed_export_timeout_task is not None
            and not self._failed_export_timeout_task.done()
            and self._failed_export_timeout_task is not current_task
        ):
            self._failed_export_timeout_task.cancel()
        self._failed_export_timeout_task = None

    async def _disconnect_failed_export_after_timeout(self) -> None:
        epoch = self._export_failure_epoch
        client = self._client
        transfer = self._export_receiver.start
        try:
            await asyncio.sleep(5.0)
            if (epoch == self._export_failure_epoch and client is self._client
                    and transfer is self._export_receiver.start and not self._online_active
                    and not self._online_fault_reason
                    and self._export_receiver.failed and not self._export_receiver.end_received):
                self._log("[BLE_EXPORT_ERR] no EXPORT_END after failed transfer; disconnecting for retry")
                self._emit_transfer_status(
                    "transfer_failed",
                    reason="no EXPORT_END after transfer failure; reconnect to retry",
                )
                await self._safe_disconnect()
        except asyncio.CancelledError:
            return

    async def _update_device_clear_metadata(
        self,
        session_key: str,
        clear_status: str,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        updates: dict[str, Any] = {
            "device_clear_status": clear_status,
            "device_clear_verified": clear_status == CLEAR_STATUS_OK,
        }
        if metadata:
            updates.update(metadata)
        try:
            await asyncio.to_thread(self._session_store.update_metadata, session_key, updates)
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update device clear metadata failed: {exc}")

    def _emit_transfer_status(self, status: str, **payload: Any) -> None:
        event = {"type": "transfer_status", "status": status}
        event.update(payload)
        self._emit(event)

    def _handle_export_confirm_ack(self, ack: ZY100Ack) -> None:
        if ack.cmd_echo == CMD_EXPORT_CONFIRM:
            if self._pending_confirm_seq is not None and ack.seq_echo != self._pending_confirm_seq:
                return
            if self._pending_confirm_session_key is None:
                return
            if ack.status != STATUS_OK:
                self._loop.create_task(
                    self._mark_export_confirm_failed(
                        ack,
                        f"EXPORT_CONFIRM ACK status=0x{ack.status:02X}",
                    )
                )
                return
            if ack.exec_mode == EXEC_MODE_ASYNC_DONE:
                self._loop.create_task(self._record_export_confirm_final_ack(ack))
                return
            self._loop.create_task(self._record_export_confirm_immediate_ack(ack))
            return

        if (
            ack.cmd_echo == CMD_PING
            and self._pending_confirm_session_key is not None
            and self._pending_confirm_final_ack_ok
            and ack.status == STATUS_OK
            and ack.device_state == DEVICE_STATE_WAIT_START
        ):
            self._loop.create_task(self._finish_confirm_wait_after_device_ready(ack, via="ping"))

    async def _record_export_confirm_immediate_ack(self, ack: ZY100Ack) -> None:
        session_key = self._pending_confirm_session_key
        if not session_key:
            return
        export_id = self._pending_confirm_export_id or 0
        self._pending_confirm_ack_ok = True
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "confirm_ack_status": "OK",
                    "reclaim_status": RECLAIM_STATUS_RECLAIMING,
                    "reclaim_verified": False,
                    "reclaim_started_at": now_iso(),
                    "confirm_ack_at": now_iso(),
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update confirm ACK metadata failed: {exc}")

        self._log(
            f"[BLE_CONFIRM] ack_ok export_id={export_id} session={session_key} "
            f"device_state=0x{ack.device_state:02X}"
        )
        self._emit_transfer_status(
            RECLAIM_STATUS_RECLAIMING,
            session_key=session_key,
            export_id=export_id,
            device_state=ack.device_state,
        )

    async def _record_export_confirm_final_ack(self, ack: ZY100Ack) -> None:
        session_key = self._pending_confirm_session_key
        if not session_key:
            return
        export_id = self._pending_confirm_export_id or 0
        if ack.detail and ack.detail != (export_id & 0xFFFFFFFF):
            await self._mark_export_confirm_failed(
                ack,
                f"EXPORT_CONFIRM final ACK detail mismatch: detail={ack.detail} export_id={export_id}",
            )
            return

        self._pending_confirm_ack_ok = True
        self._pending_confirm_final_ack_ok = True
        device_ready = ack.device_state == DEVICE_STATE_WAIT_START
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "confirm_ack_status": "OK",
                    "reclaim_status": RECLAIM_STATUS_OK,
                    "reclaim_verified": True,
                    "reclaim_completed_at": now_iso(),
                    "reclaim_session_uid": ack.detail,
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update final reclaim metadata failed: {exc}")

        self._log(
            f"[BLE_CONFIRM] final_ack export_id={export_id} session={session_key} "
            f"device_state=0x{ack.device_state:02X} detail={ack.detail}"
        )
        self._emit_transfer_status(
            RECLAIM_STATUS_OK,
            session_key=session_key,
            export_id=export_id,
            device_state=ack.device_state,
            session_uid=ack.detail,
            device_ready=device_ready,
        )
        if device_ready:
            await self._finish_confirm_wait_after_device_ready(ack, via="export_confirm_final_ack")

    async def _finish_confirm_wait_after_device_ready(self, ack: ZY100Ack, *, via: str) -> None:
        session_key = self._pending_confirm_session_key
        if not session_key:
            return
        export_id = self._pending_confirm_export_id or 0
        self._reset_pending_confirm_context()
        self._export_commands_blocked = False
        self._log(f"[BLE_CONFIRM] device_ready export_id={export_id} via={via}")
        self._emit_transfer_status(
            RECLAIM_STATUS_OK,
            session_key=session_key,
            export_id=export_id,
            device_state=ack.device_state,
            device_ready=True,
        )
        await self._restore_export_high_speed(f"{via}_device_ready")

    async def _mark_export_confirm_failed(self, ack: ZY100Ack, reason: str) -> None:
        session_key = self._pending_confirm_session_key
        if not session_key:
            return
        export_id = self._pending_confirm_export_id or 0
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "confirm_ack_status": f"ERROR_0x{ack.status:02X}",
                    "reclaim_status": RECLAIM_STATUS_FAILED,
                    "reclaim_verified": False,
                    "reclaim_failed_at": now_iso(),
                    "reclaim_failed_reason": reason,
                    "confirm_failed_at": now_iso(),
                    "confirm_failed_reason": reason,
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update confirm failure metadata failed: {exc}")
        self._reset_pending_confirm_context()
        self._export_commands_blocked = True
        self._log(f"[BLE_CONFIRM] failed export_id={export_id} reason={reason}")
        self._emit_transfer_status(
            "transfer_failed",
            session_key=session_key,
            export_id=export_id,
            reason=reason,
        )
        await self._restore_export_high_speed("export_confirm_failed")

    async def _mark_confirm_write_failed_for_session(self, session_key: str, export_id: int, *, reason: str) -> None:
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "confirm_sent": False,
                    "confirm_ack_status": reason,
                    "reclaim_status": RECLAIM_STATUS_FAILED,
                    "reclaim_verified": False,
                    "reclaim_failed_at": now_iso(),
                    "reclaim_failed_reason": reason,
                    "confirm_failed_at": now_iso(),
                    "confirm_failed_reason": reason,
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update confirm write failure metadata failed: {exc}")
        self._export_commands_blocked = True
        self._emit_transfer_status(
            "transfer_failed",
            export_id=export_id,
            session_key=session_key,
            reason=reason,
        )

    async def _mark_reclaim_unknown_for_session(self, session_key: str, export_id: int, *, reason: str) -> None:
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "reclaim_status": RECLAIM_STATUS_UNKNOWN,
                    "reclaim_verified": False,
                    "reclaim_unknown_at": now_iso(),
                    "reclaim_unknown_reason": reason,
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update reclaim unknown metadata failed: {exc}")
        self._emit_transfer_status(
            RECLAIM_STATUS_UNKNOWN,
            export_id=export_id,
            session_key=session_key,
            reason=reason,
        )

    async def _mark_reclaim_ok_for_session(
        self,
        session_key: str,
        export_id: int,
        *,
        via: str,
        device_state: int,
        device_ready: bool,
    ) -> None:
        try:
            await asyncio.to_thread(
                self._session_store.update_metadata,
                session_key,
                {
                    "reclaim_status": RECLAIM_STATUS_OK,
                    "reclaim_verified": True,
                    "reclaim_completed_at": now_iso(),
                    "reclaim_verified_via": via,
                },
            )
            self._emit({"type": "sessions_changed"})
        except Exception as exc:
            self._log(f"Update reclaim ok metadata failed: {exc}")
        self._emit_transfer_status(
            RECLAIM_STATUS_OK,
            export_id=export_id,
            session_key=session_key,
            device_state=device_state,
            device_ready=device_ready,
            via=via,
        )

    async def _mark_pending_confirm_unknown_if_needed(self, *, reason: str) -> None:
        session_key = self._pending_confirm_session_key
        if not session_key:
            return
        if not self._pending_confirm_ack_ok or self._pending_confirm_final_ack_ok:
            return
        await self._mark_reclaim_unknown_for_session(
            session_key,
            self._pending_confirm_export_id or 0,
            reason=reason,
        )

    def _reset_pending_confirm_context(self) -> None:
        self._pending_confirm_session_key = None
        self._pending_confirm_export_id = None
        self._pending_confirm_seq = None
        self._pending_confirm_ack_ok = False
        self._pending_confirm_final_ack_ok = False

    def _clear_flash_is_current(self, transaction: _ClearFlashTransaction) -> bool:
        return (self._clear_flash_transaction is transaction
                and transaction.connection_token == self._connection_token
                and transaction.client is self._client)

    async def _clear_flash_send_failed(self, transaction: _ClearFlashTransaction, reason: str) -> None:
        if self._clear_flash_is_current(transaction):
            transaction.terminal_started = True
            await self._mark_clear_flash_failed(
                None, reason, transaction=transaction, resume_sync=True, keep_blocked=False)

    async def _track_clear_flash_request(
        self, seq: int, *, transaction: _ClearFlashTransaction | None = None,
    ) -> None:
        transaction = transaction or _ClearFlashTransaction(seq, self._connection_token, self._client)
        self._clear_flash_transaction = transaction
        self._clear_flash_seq = seq
        self._clear_flash_active = True
        self._clear_flash_device_accepted = False
        self._clear_flash_accept_ok = False
        self._export_commands_blocked = True
        await self._offline_v2_sync.pause_for_clear()
        if not self._clear_flash_is_current(transaction):
            return
        session_keys = await asyncio.to_thread(self._collect_current_device_session_keys_for_clear)
        if not self._clear_flash_is_current(transaction):
            return
        offline_keys = await asyncio.to_thread(self._collect_current_device_offline_v2_keys_for_clear)
        if not self._clear_flash_is_current(transaction):
            return
        self._clear_flash_session_keys = session_keys
        self._clear_flash_offline_v2_session_keys = offline_keys
        requested_at = now_iso()
        for session_key in sorted(self._clear_flash_session_keys):
            if not self._clear_flash_is_current(transaction):
                return
            await self._update_device_clear_metadata(
                session_key,
                CLEAR_STATUS_REQUESTED,
                {
                    "device_clear_requested_at": requested_at,
                    "device_clear_verified": False,
                },
            )
        if not self._clear_flash_is_current(transaction):
            return
        self._log(
            f"[BLE_CLEAR] requested seq={seq} legacy_sessions={len(self._clear_flash_session_keys)} "
            f"offline_v2_sessions={len(self._clear_flash_offline_v2_session_keys)} "
            f"address={self._connected_address or ''}"
        )
        self._emit_transfer_status(
            "clear_requested",
            session_count=(len(self._clear_flash_session_keys) +
                           len(self._clear_flash_offline_v2_session_keys)),
        )

    def _collect_current_device_session_keys_for_clear(self) -> set[str]:
        try:
            sessions = self._session_store.list_sessions()
        except Exception as exc:
            self._log(f"[BLE_CLEAR] list_sessions failed before CLEAR_FLASH: {exc}")
            return set()

        current_address = self._connected_address or ""
        keys: set[str] = set()
        for session in sessions:
            if session.get("transfer_status") not in {"complete", "duplicate"}:
                continue
            if current_address and session.get("device_address") != current_address:
                continue
            if self._session_device_clear_status(session) == CLEAR_STATUS_OK:
                continue
            session_key = str(session.get("session_key") or "")
            if session_key:
                keys.add(session_key)
        return keys

    def _collect_current_device_offline_v2_keys_for_clear(self) -> set[str]:
        current_address = self._connected_address or ""
        keys: set[str] = set()
        for session in self._offline_v2_store.list_sessions():
            if session.get("transfer_status") != "complete":
                continue
            if current_address and session.get("device_address") != current_address:
                continue
            if session.get("device_clear_verified"):
                continue
            session_key = str(session.get("session_key") or "")
            if session_key:
                keys.add(session_key)
        return keys

    def _handle_clear_flash_ack(self, ack: ZY100Ack) -> None:
        transaction = self._clear_flash_transaction
        if (ack.cmd_echo != CMD_CLEAR_FLASH or transaction is None
                or not self._clear_flash_is_current(transaction)
                or ack.seq_echo != transaction.seq or transaction.terminal_started):
            return
        transaction.ack_received = True
        if ack.status != STATUS_OK:
            transaction.terminal_started = True
            self._loop.create_task(self._mark_clear_flash_failed(
                ack, f"CLEAR_FLASH ACK status=0x{ack.status:02X}",
                transaction=transaction,
                resume_sync=not self._clear_flash_device_accepted,
                keep_blocked=self._clear_flash_device_accepted,
            ))
            return
        if ack.exec_mode in {EXEC_MODE_ACCEPTED_ASYNC, EXEC_MODE_REAL_ACTION, EXEC_MODE_ASYNC_DONE}:
            if not self._clear_flash_device_accepted:
                self._clear_flash_device_accepted = True
                self._clear_flash_accept_task = self._loop.create_task(
                    self._accept_clear_flash(transaction))
            self._log(
                f"[BLE_CLEAR] accepted seq={ack.seq_echo} device_state=0x{ack.device_state:02X} "
                f"exec_mode=0x{ack.exec_mode:02X}")
        if ack.exec_mode == EXEC_MODE_ASYNC_DONE:
            transaction.terminal_started = True
            self._loop.create_task(self._verify_and_mark_clear_flash_ok(ack, transaction))

    async def _accept_clear_flash(self, transaction: _ClearFlashTransaction) -> None:
        if not self._clear_flash_is_current(transaction):
            return
        try:
            discarded = await self._offline_v2_sync.accept_clear()
            if not self._clear_flash_is_current(transaction):
                return
            self._clear_flash_accept_ok = True
            self._log(f"[BLE_CLEAR] accepted_partial_discarded={discarded or '<none>'}")
            self._emit_transfer_status("clear_accepted", discarded_path=discarded or "")
        except Exception as exc:
            if not self._clear_flash_is_current(transaction):
                return
            self._export_commands_blocked = True
            self._log(f"[BLE_CLEAR][ERR] partial_discard_failed error={exc}")
            self._emit_transfer_status("clear_failed", reason=f"partial quarantine failed: {exc}")

    async def _verify_and_mark_clear_flash_ok(
        self, ack: ZY100Ack, transaction: _ClearFlashTransaction,
    ) -> None:
        if not self._clear_flash_is_current(transaction):
            return
        if self._clear_flash_accept_task is not None:
            await asyncio.shield(self._clear_flash_accept_task)
        if not self._clear_flash_is_current(transaction):
            return
        if not self._clear_flash_accept_ok:
            await self._mark_clear_flash_failed(
                ack, "CLEAR_FLASH partial quarantine did not complete", transaction=transaction)
            return
        try:
            empty = await self._offline_v2_sync.verify_empty_after_clear()
        except Exception as exc:
            if self._clear_flash_is_current(transaction):
                await self._mark_clear_flash_failed(
                    ack, f"CLEAR_FLASH empty LIST verification error: {exc}", transaction=transaction)
            return
        if not self._clear_flash_is_current(transaction):
            return
        if not empty:
            await self._mark_clear_flash_failed(
                ack, "CLEAR_FLASH completed but empty LIST verification failed", transaction=transaction)
            return
        await self._mark_clear_flash_ok(ack, transaction)

    async def _mark_clear_flash_ok(self, ack: ZY100Ack, transaction: _ClearFlashTransaction) -> None:
        if not self._clear_flash_is_current(transaction):
            return
        cleared_at = now_iso()
        for session_key in sorted(self._clear_flash_session_keys):
            if not self._clear_flash_is_current(transaction):
                return
            await self._update_device_clear_metadata(
                session_key,
                CLEAR_STATUS_OK,
                {
                    "device_clear_completed_at": cleared_at,
                    "device_clear_verified": True,
                },
            )
        for session_key in sorted(self._clear_flash_offline_v2_session_keys):
            if not self._clear_flash_is_current(transaction):
                return
            await asyncio.to_thread(
                self._offline_v2_store.update_device_clear_status,
                session_key,
                status=CLEAR_STATUS_OK,
                verified=True,
                reason="empty LIST verified",
            )
        if not self._clear_flash_is_current(transaction):
            return
        session_count = (len(self._clear_flash_session_keys) +
                         len(self._clear_flash_offline_v2_session_keys))
        self._offline_v2_sync.finish_clear_success()
        self._reset_clear_flash_context()
        self._export_commands_blocked = False
        self._log(f"[BLE_CLEAR] clear_ok sessions={session_count} device_state=0x{ack.device_state:02X}")
        self._emit_transfer_status(
            "clear_ok",
            session_count=session_count,
            device_state=ack.device_state,
        )
        result = self._clear_flash_result_future
        if result is not None and not result.done():
            result.set_result(True)

    async def _mark_clear_flash_failed(
        self,
        ack: ZY100Ack | None,
        reason: str,
        *,
        transaction: _ClearFlashTransaction,
        resume_sync: bool = False,
        keep_blocked: bool = True,
    ) -> None:
        if not self._clear_flash_is_current(transaction):
            return
        failed_at = now_iso()
        for session_key in sorted(self._clear_flash_session_keys):
            if not self._clear_flash_is_current(transaction):
                return
            await self._update_device_clear_metadata(
                session_key,
                CLEAR_STATUS_FAILED,
                {
                    "device_clear_failed_at": failed_at,
                    "device_clear_failed_reason": reason,
                    "device_clear_verified": False,
                },
            )
        for session_key in sorted(self._clear_flash_offline_v2_session_keys):
            if not self._clear_flash_is_current(transaction):
                return
            await asyncio.to_thread(
                self._offline_v2_store.update_device_clear_status,
                session_key,
                status=CLEAR_STATUS_FAILED,
                verified=False,
                reason=reason,
            )
        if not self._clear_flash_is_current(transaction):
            return
        session_count = (len(self._clear_flash_session_keys) +
                         len(self._clear_flash_offline_v2_session_keys))
        self._export_commands_blocked = keep_blocked
        if resume_sync:
            try:
                await self._offline_v2_sync.resume_after_clear_rejected()
            except Exception as exc:
                if not self._clear_flash_is_current(transaction):
                    return
                self._export_commands_blocked = True
                await self._offline_v2_sync.pause_for_clear()
                reason = f"{reason}; directory recovery failed: {exc}"
        if not self._clear_flash_is_current(transaction):
            return
        self._reset_clear_flash_context()
        self._log(f"[BLE_CLEAR] clear_failed sessions={session_count} reason={reason}")
        self._emit_transfer_status(
            "clear_failed",
            session_count=session_count,
            reason=reason,
        )
        result = self._clear_flash_result_future
        if result is not None and not result.done():
            result.set_result(False)

    def _reset_clear_flash_context(self) -> None:
        self._clear_flash_transaction = None
        self._clear_flash_session_keys = set()
        self._clear_flash_offline_v2_session_keys = set()
        self._clear_flash_seq = None
        self._clear_flash_active = False
        self._clear_flash_device_accepted = False
        self._clear_flash_accept_task = None
        self._clear_flash_accept_ok = False

    def _complete_ota_dfu_entry_ack(self, uuid: str, payload: bytes) -> bool:
        if payload not in {OTA_DFU_ENTRY_ACK, OTA_DFU_ENTRY_REJECT}:
            return False
        raw = hex_bytes(payload)
        accepted = payload == OTA_DFU_ENTRY_ACK
        self._log(f"[BLE_OTA_ENTRY] ack_received uuid={uuid} raw={raw}")
        if self._ota_dfu_entry_ack_future is not None and not self._ota_dfu_entry_ack_future.done():
            self._ota_dfu_entry_ack_future.set_result(
                {
                    "ok": accepted,
                    "uuid": uuid,
                    "raw_hex": raw,
                    "message": "" if accepted else "Device rejected the OTA entry request",
                }
            )
        self._emit(
            {
                "type": "ota_dfu_entry_ack",
                "ok": accepted,
                "uuid": uuid,
                "raw_hex": raw,
            }
        )
        return True

    def _complete_ota_dfu_entry_reject_from_zy100_ack(self, uuid: str, ack: ZY100Ack) -> bool:
        if self._ota_dfu_entry_ack_future is None or self._ota_dfu_entry_ack_future.done():
            return False
        if ack.cmd_echo != OTA_DFU_ENTRY_REQUEST[2] or ack.seq_echo != OTA_DFU_ENTRY_REQUEST[3]:
            return False
        if ack.status == STATUS_OK:
            return False

        raw = hex_bytes(ack.raw)
        message = (
            "进入 DFU 指令被普通 9ECA 命令入口拒绝："
            f"cmd_echo=0x{ack.cmd_echo:02X} seq_echo=0x{ack.seq_echo:02X} "
            f"status=0x{ack.status:02X}。这表示 7 字节裸指令写入到了错误 characteristic。"
        )
        self._log(f"[BLE_OTA_ENTRY][ERR] rejected_by_9eca uuid={uuid} raw={raw}")
        self._ota_dfu_entry_ack_future.set_result(
            {
                "ok": False,
                "uuid": uuid,
                "raw_hex": raw,
                "message": message,
            }
        )
        return True

    def _handle_ota_dfu_entry_notification(self, sender: Any, data: bytearray) -> None:
        payload = bytes(data)
        uuid = self._resolve_sender_uuid(sender, self._ota_dfu_entry_notify_uuid)
        if self._complete_ota_dfu_entry_ack(uuid, payload):
            return
        self._log(
            "[BLE_OTA_ENTRY] "
            f"notify_ignored uuid={uuid} len={len(payload)} raw={hex_bytes(payload)}"
        )

    def _receive_timeout_action(self, ack: ZY100Ack) -> None:
        try:
            notice = parse_timeout_action(ack)
        except ValueError as exc:
            self._log(f"[TIMEOUT_ACTION] ignored: {exc}")
            return
        client = self._client
        if client is None or not client.is_connected:
            return
        if self._bootstrap_generation and notice["generation"] != self._bootstrap_generation:
            return
        notice["connection_token"] = self._connection_token
        previous = getattr(self, "_timeout_action", None)
        if previous and previous != notice:
            # A delayed duplicate must not replace a newer transaction.
            delta = (notice["token"] - previous["token"]) & 0xFFFFFFFF
            if previous["connection_token"] == self._connection_token and delta >= 0x80000000:
                return
        self._timeout_action = notice
        self._timeout_action_client = client
        self._last_device_state = ack.device_state
        self._emit({"type": "timeout_action", **notice})

    def timeout_action_is_current(self, notice: dict[str, Any]) -> bool:
        current = getattr(self, "_timeout_action", None)
        return bool(current and self._client is self._timeout_action_client
                    and self._client is not None and self._client.is_connected
                    and current["connection_token"] == self._connection_token
                    and all(notice.get(key) == value for key, value in current.items()))

    def confirm_timeout_action(self, notice: dict[str, Any]) -> None:
        # Called by the UI only after its reason label has been updated.
        self._submit(self._ack_timeout_action(dict(notice)))

    async def _ack_timeout_action(self, notice: dict[str, Any]) -> None:
        current = getattr(self, "_timeout_action", None)
        if not current or any(notice.get(key) != value for key, value in current.items()):
            return
        client = self._timeout_action_client
        token = current["connection_token"]
        try:
            await self._write_central_control_frame(
                build_timeout_action_ack(current["generation"], current["token"], current["reason"]),
                label="TIMEOUT_ACTION_ACK", expected_client=client, expected_token=token,
                expected_guard=lambda: self._timeout_action == current)
        except Exception as exc:
            # Device's bounded fallback remains authoritative if the link fails.
            self._log(f"[TIMEOUT_ACTION] ACK write failed: {exc}")

    def _observe_timeout_action_state(self, state: int) -> None:
        notice = getattr(self, "_timeout_action", None)
        if not notice:
            return
        previous = getattr(self, "_last_device_state", None)
        if state == DEVICE_STATE_ERROR or (state in {DEVICE_STATE_OFFLINE_CAPTURING, DEVICE_STATE_CAPTURING}
                and previous not in {DEVICE_STATE_OFFLINE_CAPTURING, DEVICE_STATE_CAPTURING}):
            self._timeout_action = None
            self._timeout_action_client = None
            self._emit({"type": "timeout_action_clear"})
        elif notice["reason"] == TIMEOUT_ACTION_OFFLINE_IDLE and state != DEVICE_STATE_OFFLINE_CAPTURING:
            message = ("已自动停止，正在收尾" if state in {DEVICE_STATE_OFFLINE_FINALIZING, DEVICE_STATE_STOPPING}
                       else "已自动停止：连续30分钟没有保存到事件")
            self._emit({"type": "timeout_action_state", "message": message, **notice})

    def _handle_ack_notification(self, sender: Any, data: bytearray) -> None:
        payload = bytes(data)
        uuid = self._resolve_sender_uuid(sender, self._ack_uuid)
        if self._complete_ota_dfu_entry_ack(uuid, payload):
            return
        try:
            ack = parse_zy100_ack(payload)
        except ValueError as exc:
            raw = hex_bytes(payload)
            self._log(f"[BLE_ACK_ERR] reason={exc} raw={raw}")
            self._emit(
                {
                    "type": "zy100_ack_error",
                    "uuid": uuid,
                    "error": str(exc),
                    "raw_hex": raw,
                }
            )
            return
        if self.stress.handle_ack(ack):
            return
        if ack.cmd_echo == CMD_TIMEOUT_ACTION_NOTIFY:
            self._receive_timeout_action(ack)
            return
        if ack.cmd_echo == CMD_TIMEOUT_ACTION_ACK:
            return
        if (ack.cmd_echo == CMD_STATE_NOTIFY and
                (ack.detail & 0xFF) == STATE_DETAIL_LOW_BATTERY_REASON):
            self._observe_timeout_action_state(DEVICE_STATE_ERROR)
        if ack.cmd_echo == CMD_PING:
            self._link_trace.note_ping_ack(
                ack.seq_echo,
                status=ack.status,
                device_state=ack.device_state,
                gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
            )
        if self._complete_ota_dfu_entry_reject_from_zy100_ack(uuid, ack):
            return
        if ack.cmd_echo == CMD_ENTER_SHIPPING:
            if ack.status == 0x00 and ack.exec_mode == 0x03:
                self._shipping_operation_active = True
                self._shipping_accepted = True
                self._shipping_seq = ack.seq_echo
                self._emit(
                    {
                        "type": "shipping_status",
                        "status": "accepted",
                        "seq": ack.seq_echo,
                        "message": "设备已接受船运模式，等待设备断开",
                    }
                )
            elif ack.status != 0x00:
                self._emit(
                    {
                        "type": "shipping_status",
                        "status": "failed",
                        "seq": ack.seq_echo,
                        "message": f"进入船运模式失败：{status_text(ack.status)}",
                    }
                )
                self._shipping_operation_active = False
                self._shipping_seq = None
                self._shipping_accepted = False
        if (
            ack.cmd_echo == CMD_OTA_LINK_INTENT
            and self._ota_link_intent_ack_future is not None
            and not self._ota_link_intent_ack_future.done()
            and ack.seq_echo == self._ota_link_intent_seq
        ):
            self._ota_link_intent_ack_future.set_result(ack)
        if (
            ack.cmd_echo == CMD_OTA_PREPARE
            and self._ota_prepare_ack_future is not None
            and not self._ota_prepare_ack_future.done()
            and ack.seq_echo == self._ota_prepare_seq
        ):
            self._ota_prepare_ack_future.set_result(ack)
        if (
            ack.cmd_echo == CMD_OTA_COMMIT
            and self._ota_commit_ack_future is not None
            and not self._ota_commit_ack_future.done()
            and ack.seq_echo == self._ota_commit_seq
            and self._ota_commit_connection == (self._client, self._connection_token)
            and self._connection_coordinator.is_current(self._connection_token)
        ):
            if ack.status == STATUS_OK:
                self._ota_commit_accepted_connection = self._ota_commit_connection
            self._ota_commit_ack_future.set_result(ack)
        if (
            ack.cmd_echo == CMD_HOST_CI_MODE_ENABLE
            and self._central_enable_future is not None
            and not self._central_enable_future.done()
            and ack.seq_echo == self._central_enable_seq
        ):
            self._central_enable_future.set_result(ack)
        if (
            ack.cmd_echo == CMD_GET_CONNECTION_STATE
            and self._bootstrap_query_ack_future is not None
            and not self._bootstrap_query_ack_future.done()
            and ack.seq_echo == self._bootstrap_query_seq
        ):
            self._bootstrap_query_ack_future.set_result(ack)
        if ack.cmd_echo == CMD_CONNECTION_STATE_NOTIFY:
            try:
                snapshot = parse_connection_state_snapshot(ack)
            except ValueError as exc:
                self._log(f"[BLE_BOOTSTRAP][ERR] invalid_snapshot reason={exc}")
                return
            self._log(
                "[BLE_BOOTSTRAP] snapshot_rx "
                f"generation={snapshot.generation} revision={snapshot.revision} "
                f"status={snapshot.overall_status} ready_bits=0x{snapshot.ready_bits:08X} "
                f"stage={snapshot.stage} reason={snapshot.reason} retry_ms={snapshot.retry_ms}"
            )
            if (
                self._bootstrap_generation
                and snapshot.generation != self._bootstrap_generation
            ):
                self._log(
                    "[BLE_BOOTSTRAP] stale_generation "
                    f"got={snapshot.generation} expected={self._bootstrap_generation}"
                )
                return
            if (
                self._bootstrap_revision is not None
                and ((snapshot.revision - self._bootstrap_revision) & 0xFF) >= 0x80
            ):
                self._log(
                    "[BLE_BOOTSTRAP] stale_revision "
                    f"got={snapshot.revision} last={self._bootstrap_revision}"
                )
                return
            self._bootstrap_generation = snapshot.generation
            self._bootstrap_revision = snapshot.revision
            self._bootstrap_last_status = snapshot.overall_status
            self._bootstrap_ready_bits = snapshot.ready_bits
            self._bootstrap_device_state = snapshot.device_state
            self._bootstrap_suggested_action = snapshot.suggested_action
            self._bootstrap_stage = snapshot.stage
            self._bootstrap_reason = snapshot.reason
            self._observe_timeout_action_state(snapshot.device_state)
            self._last_device_state = snapshot.device_state
            self._observe_runtime_offline_state(
                snapshot.device_state,
                source="bootstrap",
            )
            self._bootstrap_snapshot_event.set()
            if (
                self._bootstrap_snapshot_future is not None
                and not self._bootstrap_snapshot_future.done()
            ):
                self._bootstrap_snapshot_future.set_result(snapshot)
            self._emit(
                {
                    "type": "connection_state",
                    "status": snapshot.overall_status,
                    "device_state": snapshot.device_state,
                    "suggested_action": snapshot.suggested_action,
                    "bootstrap_version": snapshot.bootstrap_version,
                    "generation": snapshot.generation,
                    "ready_bits": snapshot.ready_bits,
                    "stage": snapshot.stage,
                    "reason": snapshot.reason,
                    "retry_ms": snapshot.retry_ms,
                }
            )
            return
        if ack.cmd_echo == CMD_LINK_STATE_NOTIFY:
            self._handle_central_link_notify(ack)
            payload_dict = ack.to_dict()
            payload_dict["uuid"] = uuid
            self._emit({"type": "zy100_link_state", "ack": payload_dict})
            return
        self._observe_timeout_action_state(ack.device_state)
        self._last_device_state = ack.device_state
        # Configuration retry is owned by its original bootstrap transaction.
        # Its polling sees this state; unrelated ACKs must not create a retry
        # from an older snapshot while a manual submission is pending.
        self._observe_runtime_offline_state(
            ack.device_state,
            source=(
                "state_notify"
                if ack.cmd_echo == CMD_STATE_NOTIFY
                else "command_ack"
            ),
        )

        if ack.cmd_echo == CMD_STATE_NOTIFY:
            self._log(
                "[BLE_STATE] "
                f"notify seq={ack.seq_echo} status=0x{ack.status:02X} "
                f"device_state=0x{ack.device_state:02X} exec_mode=0x{ack.exec_mode:02X}"
            )
            self._handle_online_state_notify(ack)
            payload_dict = ack.to_dict()
            payload_dict["uuid"] = uuid
            self._emit({"type": "zy100_state_notify", "ack": payload_dict})
            self._maybe_enter_export_high_speed_from_ack(ack)
            return

        if ack.cmd_echo == CMD_TIME_SYNC:
            self._log(
                "[BLE_RX] "
                f"ACK TIME_SYNC seq={ack.seq_echo} status=0x{ack.status:02X} "
                f"device_state=0x{ack.device_state:02X} exec_mode=0x{ack.exec_mode:02X} "
                f"user_id={ack.user_id} training_id={ack.training_id} detail={ack.detail}"
            )
            if (
                self._time_sync_ack_future is not None
                and not self._time_sync_ack_future.done()
                and self._time_sync_seq == ack.seq_echo
            ):
                self._time_sync_ack_future.set_result(ack)

        if (
            ack.cmd_echo == CMD_CONNECTION_USER_SYNC
            and self._connection_user_sync_ack_future is not None
            and not self._connection_user_sync_ack_future.done()
            and self._connection_user_sync_seq == ack.seq_echo
        ):
            self._connection_user_sync_ack_future.set_result(ack)

        if (
            ack.cmd_echo == CMD_FEATURE_CONFIG_SYNC
            and self._feature_config_seq == ack.seq_echo
        ):
            self._feature_config_ack_queue.put_nowait(ack)

        if ack.cmd_echo != CMD_ONLINE_RECORD_ACK:
            self._log(
                "[BLE_ACK] "
                f"cmd={command_name(ack.cmd_echo)} seq={ack.seq_echo} status=0x{ack.status:02X} "
                f"exec_mode=0x{ack.exec_mode:02X} user_id={ack.user_id} training_id={ack.training_id}"
            )
        payload_dict = ack.to_dict()
        payload_dict["uuid"] = uuid
        self._emit({"type": "zy100_ack", "ack": payload_dict})

        if ack.cmd_echo in {
            CMD_OFFLINE_SESSION_LIST,
            CMD_OFFLINE_SESSION_BEGIN,
            CMD_OFFLINE_CHUNK_ACK,
            CMD_OFFLINE_SESSION_RESUME,
            CMD_OFFLINE_FINAL_CONFIRM,
            CMD_OFFLINE_RECLAIM_STATUS,
        }:
            self._loop.create_task(self._offline_v2_sync.on_ack(
                ack, epoch=self._offline_v2_sync.epoch))

        if ack.cmd_echo == CMD_OFFLINE_FOREIGN_PURGE:
            self._foreign_purge_ack_queue.put_nowait(ack)

        self._handle_export_confirm_ack(ack)
        self._handle_clear_flash_ack(ack)
        self._handle_offline_control_ack(ack)
        self._handle_online_command_ack(ack)
        self._maybe_enter_export_high_speed_from_ack(ack)

        if (
            ack.cmd_echo == CMD_START_CAPTURE
            and ack.status != 0x00
            and ack.status == 0x05
        ):
            self._log("[BLE_SYNC][ERR] START rejected, resend TIME_SYNC")

    def _central_profile_key_is_current(
        self,
        key: tuple[int, int, int, int],
    ) -> bool:
        return (
            self._central_mode_enabled
            and self._central_profile_key == key
            and key[0] == self._central_session_id
            and key[1] == (self._central_generation & 0xFFFF)
            and key[2] == self._central_transition
        )

    def _central_handoff_key_is_current(
        self, key: tuple[int, int, int, int]
    ) -> bool:
        return (
            self._central_mode_enabled
            and self._central_hybrid_enabled
            and key[0] == self._central_session_id
            and key[1] == (self._central_generation & 0xFFFF)
            and key[2] == self._central_transition
        )

    async def _release_for_exact_standby(
        self,
        *,
        key: tuple[int, int, int, int],
        target_state: int,
    ) -> None:
        session_id, generation, transition, _ = key
        if not self._central_handoff_key_is_current(key):
            return
        self._central_session_state = "RELEASING"
        self._stop_central_heartbeat()
        active_key = self._central_profile_key
        profile_task = self._central_profile_task
        if profile_task is not None and not profile_task.done():
            profile_task.cancel()
        released = await self._central_profiles.release_for_standby(
            operation_key=key,
            active_key=active_key,
        )
        if not released or not self._central_handoff_key_is_current(key):
            self._central_session_state = "HOST_ACTIVE"
            self._start_central_heartbeat()
            self._log(
                "[HOST_CI_HYBRID] release_failed=1 action=keep_windows_central"
            )
            self._emit(
                {
                    "type": "ci_handoff",
                    "status": "release_failed_keep_central",
                    "transition": transition,
                }
            )
            return
        seq = self._reserve_control_seq()
        payload = build_host_ci_handoff_command(
            seq,
            session_id,
            generation,
            transition,
            HOST_CI_HANDOFF_RELEASE_FOR_EXACT_STANDBY,
            target_state,
        )
        await self._write_central_control_frame(
            payload, label="HOST_CI_HANDOFF_RELEASE"
        )
        self._log(
            "[HOST_CI_HYBRID] release_sent=1 "
            f"generation={generation} transition={transition}"
        )

    async def _acquire_windows_central(
        self,
        *,
        key: tuple[int, int, int, int],
        target_state: int,
    ) -> None:
        session_id, generation, transition, _ = key
        if not self._central_handoff_key_is_current(key):
            return
        self._central_session_state = "ACQUIRING"
        seq = self._reserve_control_seq()
        payload = build_host_ci_handoff_command(
            seq,
            session_id,
            generation,
            transition,
            HOST_CI_HANDOFF_ACQUIRE_WINDOWS_CENTRAL,
            target_state,
        )
        await self._write_central_control_frame(
            payload, label="HOST_CI_HANDOFF_ACQUIRE"
        )
        self._central_session_state = "HOST_ACTIVE"
        self._start_central_heartbeat()
        self._log(
            "[HOST_CI_HYBRID] acquire_sent=1 "
            f"generation={generation} transition={transition} target={target_state}"
        )

    def _handle_central_link_notify(self, ack: ZY100Ack) -> None:
        generation = ack.training_id & 0xFFFF
        transition = (ack.training_id >> 16) & 0xFFFF
        actual_ci = ack.detail & 0xFFFF
        actual_latency = (ack.detail >> 16) & 0xFFFF
        profile_value = ack.exec_mode
        self._log(
            "[HOST_CI_CENTRAL][LINK] "
            f"session={ack.user_id} generation={generation} transition={transition} "
            f"business_state={ack.device_state} requested_profile={profile_value} "
            f"phase={ack.status} actual_ci={actual_ci} latency={actual_latency} "
            f"initiator={ack.reserved}"
        )
        if (
            not self._central_mode_enabled
            and ack.user_id == self._central_session_id
            and ack.status in {LINK_STATE_INTENT, LINK_STATE_WAIT_HOST}
        ):
            self._central_early_link_ack = ack
            self._log("[HOST_CI_CENTRAL] early_link_notify_buffered=1")
            return
        if (
            not self._central_mode_enabled
            or ack.user_id != self._central_session_id
            or generation != (self._central_generation & 0xFFFF)
        ):
            self._log("[HOST_CI_CENTRAL] link_notify_stale=1")
            return
        if ack.status == LINK_STATE_HANDOFF_RELEASE_REQUEST:
            if not self._central_hybrid_enabled:
                self._log("[HOST_CI_HYBRID] release_ignored=v1_session")
                return
            key = (ack.user_id, generation, transition, 0)
            if self._central_transition > transition:
                self._log(
                    "[HOST_CI_HYBRID] release_suppressed=stale "
                    f"transition={transition} current={self._central_transition}"
                )
                return
            self._central_transition = transition
            task = self._central_handoff_task
            if task is not None and not task.done():
                task.cancel()
            self._central_handoff_task = self._loop.create_task(
                self._release_for_exact_standby(
                    key=key, target_state=ack.device_state
                )
            )
            return
        if ack.status == LINK_STATE_HOST_ACQUIRE_REQUEST:
            if not self._central_hybrid_enabled:
                self._log("[HOST_CI_HYBRID] acquire_ignored=v1_session")
                return
            key = (ack.user_id, generation, transition, 0)
            if self._central_transition > transition:
                self._log(
                    "[HOST_CI_HYBRID] acquire_suppressed=stale "
                    f"transition={transition} current={self._central_transition}"
                )
                return
            self._central_transition = transition
            task = self._central_handoff_task
            if task is not None and not task.done():
                task.cancel()
            self._central_handoff_task = self._loop.create_task(
                self._acquire_windows_central(
                    key=key, target_state=ack.device_state
                )
            )
            return
        if ack.status == LINK_STATE_PERIPHERAL_EXACT_PENDING:
            self._central_session_state = "PERIPHERAL_STANDBY"
            self._stop_central_heartbeat()
            self._central_profile_key = None
            self._log(
                "[HOST_CI_HYBRID] peripheral_exact_pending=1 "
                f"transition={transition} actual_ci={actual_ci} latency={actual_latency}"
            )
            return
        if ack.status == LINK_STATE_HANDOFF_ABORTED:
            self._central_session_state = "HOST_ACTIVE"
            self._start_central_heartbeat()
            self._log(
                "[HOST_CI_HYBRID] handoff_aborted=1 action=keep_windows_central"
            )
            return
        if ack.status in {LINK_STATE_INTENT, LINK_STATE_WAIT_HOST}:
            self._central_link_intent_event.set()
            try:
                profile = WindowsCentralProfile(profile_value)
            except ValueError:
                self._log(
                    f"[HOST_CI_CENTRAL] unsupported_profile={profile_value}"
                )
                return
            if profile == WindowsCentralProfile.THROUGHPUT_OPTIMIZED:
                self._central_balanced_applied_event.clear()
            key = (ack.user_id, generation, transition, int(profile))
            current_key = self._central_profile_key
            if current_key == key:
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"profile_request_dedup=1 transition={transition} phase={ack.status}"
                )
                return
            if (
                current_key is not None
                and current_key[0] == ack.user_id
                and current_key[1] == generation
                and transition < current_key[2]
            ):
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"profile_notify_stale=1 transition={transition} current={current_key[2]}"
                )
                return
            task = self._central_profile_task
            if task is not None and not task.done():
                task.cancel()
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"profile_task_cancelled=1 old_transition={current_key[2] if current_key else 0} "
                    f"new_transition={transition}"
                )
            self._central_profile_key = key
            self._central_transition = transition
            self._central_profile_task = self._loop.create_task(
                self._apply_central_profile(
                    key=key,
                    profile=profile,
                )
            )
        elif ack.status == LINK_STATE_APPLIED:
            if (
                profile_value == HOST_PROFILE_BALANCED
                and 24 <= actual_ci <= 48
                and actual_latency == 0
            ):
                self._central_balanced_applied_event.set()
            if self._central_hybrid_enabled and ack.reserved == 2 and ack.device_state == 4:
                self._central_session_state = "PERIPHERAL_STANDBY"
                self._stop_central_heartbeat()
            self._emit(
                {
                    "type": "ci_link_applied",
                    "business_state": ack.device_state,
                    "profile": profile_value,
                    "actual_ci": actual_ci,
                    "actual_latency": actual_latency,
                    "generation": generation,
                    "transition": transition,
                }
            )
            if (
                self._central_session_state == "ENABLING"
                and profile_value == HOST_PROFILE_BALANCED
                and 24 <= actual_ci <= 48
                and actual_latency == 0
                and self._central_applied_future is not None
                and not self._central_applied_future.done()
            ):
                self._central_transition = transition
                self._central_applied_future.set_result(True)
                self._log(
                    "[HOST_CI_CENTRAL] initial_balanced_applied=1 "
                    f"generation={generation} transition={transition} "
                    f"ci={actual_ci} latency={actual_latency}"
                )
        elif ack.status in {LINK_STATE_FAILED, LINK_STATE_TIMEOUT}:
            if (
                self._central_session_state == "ENABLING"
                and self._central_applied_future is not None
                and not self._central_applied_future.done()
            ):
                self._central_applied_future.set_result(False)
            current_key = self._central_profile_key
            if (
                current_key is None
                or current_key[0] != ack.user_id
                or current_key[1] != generation
                or current_key[2] != transition
                or current_key[3] != profile_value
            ):
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"failure_suppressed=stale transition={transition} current="
                    f"{current_key[2] if current_key else 0}"
                )
                return
            task = self._central_profile_task
            if task is not None and not task.done():
                task.cancel()
            self._central_profile_task = None
            self._central_profile_key = None
            self._central_profiles.force_release(reason="device_link_failure")

    async def _apply_central_profile(
        self,
        *,
        key: tuple[int, int, int, int],
        profile: WindowsCentralProfile,
    ) -> None:
        session_id, generation, transition, _profile_value = key
        client = self._client
        if (
            client is None
            or not client.is_connected
            or not self._central_profile_key_is_current(key)
        ):
            return
        try:
            result = await self._central_profiles.activate(
                client,
                profile,
                operation_key=key,
                on_accepted=lambda request_status: self._send_host_profile_result(
                    key=key,
                    profile=profile,
                    request_status=request_status,
                    result=HOST_RESULT_REQUEST_ACCEPTED,
                ),
            )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if not self._central_profile_key_is_current(key):
                self._log(
                    "[HOST_CI_CENTRAL] "
                    f"result_suppressed=superseded transition={transition} "
                    f"reason=profile_apply_exception error={exc}"
                )
                return
            self._central_profiles.force_release(reason="profile_apply_exception")
            self._log(f"[HOST_CI_CENTRAL][ERR] profile_apply_exception={exc}")
            self._loop.create_task(self._recover_central_profile_failure())
            return
        if not self._central_profile_key_is_current(key):
            self._log(
                "[HOST_CI_CENTRAL] "
                f"result_suppressed=superseded transition={transition} session={session_id}"
            )
            return
        if result.actual_stable:
            if (
                isinstance(self._connection_coordinator.strategy, BootstrapV1Strategy)
                and self._bootstrap_last_status != 2
            ):
                self._pending_actual_stable = (
                    key, profile, result.request_status
                )
                self._log(
                    "[HOST_CI_CENTRAL] actual_stable_deferred=1 "
                    f"generation={generation} transition={transition}"
                )
            else:
                await self._send_host_profile_result(
                    key=key,
                    profile=profile,
                    request_status=result.request_status,
                    result=HOST_RESULT_ACTUAL_STABLE,
                )
            self._log(
                "[HOST_CI_CENTRAL] "
                f"actual_stable generation={generation} transition={transition} "
                f"profile={profile.name} ci={result.actual_ci} latency={result.actual_latency}"
            )
            self._central_recovery_used = False
            return
        await self._send_host_profile_result(
            key=key,
            profile=profile,
            request_status=result.request_status,
            result=HOST_RESULT_REQUEST_FAILED,
        )
        self._log(
            "[HOST_CI_CENTRAL][ERR] "
            f"profile_failed generation={generation} transition={transition} "
            f"profile={profile.name} reason={result.reason}"
        )
        self._loop.create_task(self._recover_central_profile_failure())

    async def _flush_pending_actual_stable(self) -> None:
        pending = self._pending_actual_stable
        self._pending_actual_stable = None
        if pending is None:
            return
        key, profile, request_status = pending
        if not self._central_profile_key_is_current(key):
            self._log("[HOST_CI_CENTRAL] actual_stable_deferred_stale=1")
            return
        await self._send_host_profile_result(
            key=key,
            profile=profile,
            request_status=request_status,
            result=HOST_RESULT_ACTUAL_STABLE,
        )
        self._log("[HOST_CI_CENTRAL] actual_stable_deferred_flushed=1")

    async def _send_host_profile_result(
        self,
        *,
        key: tuple[int, int, int, int],
        profile: WindowsCentralProfile,
        request_status: int,
        result: int,
    ) -> None:
        session_id, generation, transition, _profile_value = key
        if not self._central_profile_key_is_current(key):
            self._log(
                "[HOST_CI_CENTRAL] "
                f"result_suppressed=superseded transition={transition} result={result}"
            )
            return
        seq = self._reserve_control_seq()
        payload = build_host_profile_result_command(
            seq,
            session_id,
            generation,
            transition,
            int(profile),
            request_status,
            result,
        )
        await self._write_central_control_frame(payload, label="HOST_PROFILE_RESULT")

    async def _recover_central_profile_failure(self) -> None:
        if self._central_recovery_used:
            self._log("[HOST_CI_CENTRAL][ERR] recovery_exhausted=1")
            return
        self._central_recovery_used = True
        address = self._connected_address
        user_id = self._active_user_id or 1
        training_id = self._active_training_id or 1
        if not address:
            return
        await self._safe_disconnect(emit=False)
        await asyncio.sleep(1.0)
        await self._connect(address, user_id=user_id, training_id=training_id)

    def _handle_offline_control_ack(self, ack: ZY100Ack) -> None:
        if ack.cmd_echo != CMD_OFFLINE_CAPTURE_START:
            return
        if self._offline_start_seq is None or ack.seq_echo != self._offline_start_seq:
            self._log(
                f"[OFFLINE_CONTROL] start_ack_stale seq={ack.seq_echo} "
                f"pending={self._offline_start_seq} exec=0x{ack.exec_mode:02X}"
            )
            return
        if ack.status == STATUS_OK and ack.exec_mode == EXEC_MODE_ACCEPTED_ASYNC:
            self._log(
                f"[OFFLINE_CONTROL] start_accepted seq={ack.seq_echo} state=0x{ack.device_state:02X}"
            )
            return

        self._cancel_offline_start_final_timeout()
        self._offline_start_seq = None
        if (
            ack.status == STATUS_OK
            and ack.exec_mode == EXEC_MODE_REAL_ACTION
            and ack.device_state == DEVICE_STATE_OFFLINE_CAPTURING
        ):
            self._log(
                f"[OFFLINE_CONTROL] start_final_ok seq={ack.seq_echo} session={ack.detail}"
            )
            return
        if (
            ack.status == STATUS_OK
            and ack.exec_mode == EXEC_MODE_ASYNC_DONE
            and ack.device_state == DEVICE_STATE_WAIT_START
            and ack.detail == ZY100_BLE_OFFLINE_DETAIL_START_CANCELLED
        ):
            self._log(
                f"[OFFLINE_CONTROL] start_cancelled seq={ack.seq_echo}"
            )
            return
        self._log(
            f"[OFFLINE_CONTROL][ERR] start_final_failed seq={ack.seq_echo} "
            f"status=0x{ack.status:02X} detail=0x{ack.detail:08X}"
        )

    def _handle_online_command_ack(self, ack: ZY100Ack) -> None:
        if ack.cmd_echo == CMD_ONLINE_STREAM_READY:
            if (
                self._online_ready_seq is None
                or ack.seq_echo != self._online_ready_seq
            ):
                self._log(
                    "[ONLINE_STATE] "
                    f"ready_ack_stale status=0x{ack.status:02X} seq={ack.seq_echo} "
                    f"current_seq={self._online_ready_seq}"
                )
                return
            if self._online_ready_ack_future is not None and not self._online_ready_ack_future.done():
                self._online_ready_ack_future.set_result(ack)
            self._online_ready_ack_ok = ack.status == STATUS_OK
            if ack.status != STATUS_OK:
                self._online_ready_sent = False
                self._schedule_online_ready_retry("ack_failed")
            else:
                self._cancel_online_ready_retry()
            status = "ready_ack_ok" if self._online_ready_ack_ok else "ready_ack_failed"
            self._log(
                "[ONLINE_STATE] "
                f"ready_ack status=0x{ack.status:02X} seq={ack.seq_echo} detail=0x{ack.detail:08X}"
            )
            self._emit_online_status(status, ready_ack=ack.to_dict())
            return

        if ack.cmd_echo == CMD_START_CAPTURE:
            if self._online_start_seq is None or ack.seq_echo != self._online_start_seq:
                self._log(
                    f"[ONLINE_STATE] start_ack_stale seq={ack.seq_echo} "
                    f"pending={self._online_start_seq} exec=0x{ack.exec_mode:02X}"
                )
                return
            if ack.status == STATUS_OK and ack.exec_mode == EXEC_MODE_ACCEPTED_ASYNC:
                self._emit_online_status("start_preparing", start_ack=ack.to_dict())
                return
            if (
                ack.status == STATUS_OK
                and ack.exec_mode == EXEC_MODE_REAL_ACTION
                and ack.device_state == DEVICE_STATE_CAPTURING
                and ack.detail == 0x1000
            ):
                if self._client is not None:
                    self._ci_state.observe(
                        self._export_speed.read_link_snapshot(self._client),
                        source="start_final_ack",
                        force=True,
                    )
                self._emit_online_status("start_online_ack", start_ack=ack.to_dict())
                self.stress.on_start_ack()
                self._finish_online_start_operation(reason="final_ack_success")
                return
            detail_low = ack.detail & 0xFFFF
            reason = {
                0x1001: "not_ready",
                0x1002: "start_preparation_failed",
                0x1003: "notify_not_enabled",
                START_DETAIL_ONLINE_CI9_NOT_READY: "ci9_not_ready",
                0x1005: "mag_not_ready",
                START_DETAIL_ONLINE_CI_LLCP_STUCK: "ci_llcp_stuck",
            }.get(detail_low, "invalid_final_ack")
            self._handle_online_start_failure(ack, reason)
            return

        if ack.cmd_echo == CMD_ONLINE_RECORD_ACK:
            if ack.status != STATUS_OK:
                self._log(
                    "[ONLINE_ACK][ERR] "
                    f"ack_cmd_rejected seq={ack.seq_echo} status=0x{ack.status:02X} detail=0x{ack.detail:08X}"
                )
                self._emit_online_status("ack_rejected", record_ack=ack.to_dict())
            return

    def _handle_online_state_notify(self, ack: ZY100Ack) -> None:
        if (
            self._offline_v2_host_ci_deferred
            and not self._offline_observer_active
            and not self._offline_v2_capture_in_progress()
            and (
                self._offline_v2_host_ci_resume_task is None
                or self._offline_v2_host_ci_resume_task.done()
            )
        ):
            self._offline_v2_host_ci_resume_task = self._loop.create_task(
                self._resume_host_ci_after_offline_capture()
            )
        progress_status = ONLINE_NORMAL_DEVICE_STATE_STATUS.get(ack.device_state)
        if progress_status is not None:
            if (
                ack.device_state == DEVICE_STATE_WAIT_START
                and self._online_active
            ):
                closing = self._online_end_finalizing
                if (closing is not None
                        and closing[0] == self._online_io_epoch
                        and closing[1] == self._online_session_id
                        and closing[2] is self._online_store):
                    # Do not declare success here. ACK/storage errors still flow
                    # through their existing handlers, including cancellation.
                    self._log("[ONLINE_STATE] wait_start_during_end_finalize")
                    return
                self._handle_online_error(
                    "terminal_missing_wait_start",
                    terminal_source="device_state_wait_start",
                    device_state=ack.device_state,
                )
                return
            if self._online_active or self._online_session_id is not None:
                self._emit_online_status(
                    progress_status,
                    device_state=ack.to_dict(),
                )
            return
        if not self._online_active:
            return
        self._handle_online_error(
            f"device_state_0x{ack.device_state:02X}",
            terminal_source="device_state_error",
            device_state=ack.device_state,
        )

    async def _resume_host_ci_after_offline_capture(self) -> None:
        if self._offline_observer_active:
            self._bootstrap_snapshot_event.set()
            return
        self._log("[HOST_CI_CENTRAL] retry_after_offline_capture")
        if not await self._enable_windows_central_mode():
            if not self._offline_v2_host_ci_deferred:
                await self._fail_required_central_mode("offline_capture_completed")
            return
        user_id = self._connection_user_id or self._active_user_id or 1
        if not await self._ensure_connection_user_context_ready(user_id):
            self._enter_recovery_only("deferred_connection_user_sync_failed")
            return
        if self._last_device_state == DEVICE_STATE_OFFLINE_SESSION_READY:
            await self._enter_offline_capture_observer(
                address=self._connected_address or "<unknown>",
                name=self._current_device_name or "ZY100",
                user_id=user_id,
                training_id=self._active_training_id or 1,
                central_ready=True,
            )
            return
        if not await self._initialize_calibration():
            return
        self._offline_v2_commands_ready = True
        if self._connection_user_context_supported:
            business_ready = await self._run_user_isolated_offline_then_online(
                "offline_capture_completed",
                user_id=user_id,
                training_id=self._active_training_id or 1,
            )
        else:
            await self._sync_rtc_after_notify(
                user_id=self._active_user_id or 1,
                training_id=self._active_training_id or 1,
            )
            business_ready = await self._run_offline_priority_then_online(
                "offline_capture_completed"
            )
        if not business_ready:
            self._enter_recovery_only("deferred_offline_sync_failed")
            return
        if isinstance(self._connection_coordinator.strategy, BootstrapV1Strategy):
            if not await self._wait_for_business_ready_snapshot(0.5):
                if not await self._query_bootstrap_connection_state():
                    self._enter_recovery_only("deferred_business_query_failed")
                    return
            if self._bootstrap_last_status != 2:
                self._enter_recovery_only("deferred_business_gate_not_confirmed")
                return
        self._connection_business_ready = True
        self._set_offline_priority_blocked(
            False,
            reason="deferred_business_ready",
            phase="ready",
        )

    def _maybe_enter_export_high_speed_from_ack(self, ack: ZY100Ack) -> None:
        if self._central_mode_enabled:
            return
        if ack.status != STATUS_OK:
            return
        if ack.cmd_echo == CMD_ONLINE_RECORD_ACK:
            return
        online_recent = time.monotonic() < self._online_speed_skip_until
        online_context = self._online_active or online_recent
        if ack.cmd_echo == CMD_PAUSE_CAPTURE:
            if online_context:
                return
            self._loop.create_task(self._enter_export_high_speed("pause_capture_ack"))
            return
        if ack.device_state in EXPORT_SPEED_TRIGGER_STATES:
            if online_context:
                return
            self._loop.create_task(self._enter_export_high_speed(f"ack_device_state_0x{ack.device_state:02X}"))

    @staticmethod
    def _session_device_clear_status(session: dict[str, Any]) -> str:
        clear_status = str(session.get("device_clear_status") or "")
        if clear_status:
            return clear_status
        if session.get("device_clear_verified"):
            return CLEAR_STATUS_OK
        confirm_ack_status = str(session.get("confirm_ack_status") or "")
        if confirm_ack_status == "clear_timeout":
            return "clear_unknown"
        if session.get("confirm_sent") and confirm_ack_status == "OK":
            return clear_status
        if session.get("confirm_sent") and confirm_ack_status == "pending":
            return "clear_pending"
        return clear_status

    def _find_characteristic(
        self, client: BleakClient, uuid: str
    ) -> BleakGATTCharacteristic | None:
        target = self._normalize_uuid(uuid)
        for char in self._iter_characteristics(client):
            if self._normalize_uuid(char.uuid) == target:
                return char
        return None

    def _find_characteristic_in_service(
        self,
        client: BleakClient,
        service_uuid: str,
        char_uuid: str,
    ) -> BleakGATTCharacteristic | None:
        target_service = self._normalize_uuid(service_uuid)
        target_char = self._normalize_uuid(char_uuid)
        for service in self._iter_services(client):
            if self._normalize_uuid(service.uuid) != target_service:
                continue
            for char in service.characteristics:
                if self._normalize_uuid(char.uuid) == target_char:
                    return char
        return None

    def _has_required_gatt(self) -> bool:
        return (
            self._zy100_control_found
            and self._command_found
            and self._ack_found
            and self._export_found
            and self._command_uuid is not None
            and self._ack_uuid is not None
            and self._export_uuid is not None
        )

    def _is_gatt_ready(self) -> bool:
        client = self._client
        return (
            client is not None
            and client.is_connected
            and self._has_required_gatt()
            and self._ack_subscribed
            and self._export_subscribed
        )

    def _required_gatt_missing(self, *, include_subscriptions: bool = False) -> list[str]:
        missing: list[str] = []
        if not self._zy100_control_found:
            missing.append(f"Control Service {ZY100_CONTROL_SERVICE_UUID}")
        if not self._command_found:
            missing.append(f"Command {ZY100_COMMAND_UUID}")
        if not self._ack_found:
            missing.append(f"ACK {ZY100_ACK_UUID}")
        if not self._export_found:
            missing.append(f"Export Data {ZY100_EXPORT_DATA_UUID}")
        if include_subscriptions:
            if self._ack_found and not self._ack_subscribed:
                missing.append("ACK Notify subscription")
            if self._export_found and not self._export_subscribed:
                missing.append("Export Data Notify subscription")
        return missing

    def _emit_gatt_mismatch(
        self,
        control_status: dict[str, Any] | None = None,
        *,
        action: str = "GATT discovery failed",
        include_subscriptions: bool = False,
    ) -> None:
        missing = self._required_gatt_missing(include_subscriptions=include_subscriptions)
        missing_text = ", ".join(missing) if missing else "unknown"
        legacy_profile_detected = self._has_legacy_simple_profile()
        hint = "必需 GATT 或通知订阅不完整；无法仅凭服务列表判定旧固件或缓存损坏。"
        detail = f"GATT_MISMATCH: {action}; missing={missing_text}. {hint}"
        self._log(f"[BLE_GATT][MISMATCH] missing={missing_text}")
        if legacy_profile_detected:
            self._log(f"[BLE_GATT][MISMATCH] legacy_profile={LEGACY_SIMPLE_SERVICE_UUID}")
        self._emit(
            {
                "type": "gatt_mismatch",
                "status": "GATT_MISMATCH",
                "message": hint,
                "detail": detail,
                "missing": missing,
                "address": self._connected_address or "",
                "device_name": self._current_device_name,
                "legacy_profile_detected": legacy_profile_detected,
                "service_found": self._zy100_control_found,
                "command_found": self._command_found,
                "ack_found": self._ack_found,
                "export_found": self._export_found,
                "ack_subscribed": self._ack_subscribed,
                "export_subscribed": self._export_subscribed,
                "control_status": control_status or {},
            }
        )

    def _iter_services(self, client: BleakClient) -> Iterable[Any]:
        try:
            services = getattr(client, "services", None)
            if services is not None:
                return list(services)
        except Exception:
            pass
        return list(self._last_service_objects)

    def _iter_characteristics(self, client: BleakClient) -> Iterable[BleakGATTCharacteristic]:
        for service in self._iter_services(client):
            for char in service.characteristics:
                yield char

    def _has_legacy_simple_profile(self) -> bool:
        service_uuids, char_uuids = self._collect_discovered_uuid_sets()
        legacy_chars = set(LEGACY_SIMPLE_CHARACTERISTIC_UUIDS.values())
        return LEGACY_SIMPLE_SERVICE_UUID in service_uuids and bool(legacy_chars & char_uuids)

    def _collect_discovered_uuid_sets(self) -> tuple[set[str], set[str]]:
        service_uuids: set[str] = set()
        char_uuids: set[str] = set()

        client = self._client
        if client is not None:
            try:
                for service in self._iter_services(client):
                    service_uuids.add(self._normalize_uuid(str(service.uuid)))
                    for char in service.characteristics:
                        char_uuids.add(self._normalize_uuid(str(char.uuid)))
            except Exception as exc:  # pragma: no cover - host stack dependent
                self._log(f"Collect discovered UUIDs from client failed: {exc}")

        for service in self._last_services_payload:
            service_uuids.add(self._normalize_uuid(str(service.get("uuid", ""))))
            for char in service.get("characteristics", []) or []:
                char_uuids.add(self._normalize_uuid(str(char.get("uuid", ""))))

        service_uuids.discard("")
        char_uuids.discard("")
        return service_uuids, char_uuids

    def _resolve_sender_uuid(self, sender: Any, fallback_uuid: str | None = None) -> str:
        if isinstance(sender, BleakGATTCharacteristic):
            return sender.uuid.lower()

        if isinstance(sender, int) and self._client is not None:
            for char in self._iter_characteristics(self._client):
                if getattr(char, "handle", None) == sender:
                    return char.uuid.lower()

        return fallback_uuid or str(sender)

    def disconnect(self) -> None:
        self._submit(self._cancel_connection_and_disconnect())

    async def _cancel_connection_and_disconnect(self) -> None:
        online_tasks = [task for task in (self._online_fault_task, self._online_metadata_retry_task)
                        if task is not None and task is not asyncio.current_task()]
        await self._connection_recovery.cancel()
        await self._safe_disconnect()
        if online_tasks:
            await asyncio.gather(*online_tasks, return_exceptions=True)
        await self._drain_online_storage()

    def get_last_services_snapshot(self) -> list[dict[str, Any]]:
        return deepcopy(self._last_services_payload)

    async def _safe_disconnect(
        self,
        *,
        emit: bool = True,
        preserve_start_operation: bool = False,
    ) -> None:
        closing_client = self._client
        self._online_io_epoch += 1  # Revoke in-flight completions before any disconnect awaits.
        self._cancel_export_parse_task()
        try:
            self._stop_online_diag()
            await self._offline_v2_sync.on_disconnect()
            await self._mark_pending_confirm_unknown_if_needed(reason="disconnect_before_final_reclaim_ack")
            if self._client is not None and self._client.is_connected:
                address = self._connected_address or "<unknown>"
                self._log(f"Disconnecting: {address}")
                await self._restore_export_high_speed("disconnect")
                if self._ack_subscribed and self._ack_uuid:
                    try:
                        await self._connection_coordinator.stop_notify(
                            self._connection_token, self._client, self._ack_uuid,
                            name="stop_ack_disconnect",
                        )
                    except Exception as exc:  # pragma: no cover - host stack dependent
                        self._log(f"Stop ACK notify failed: {exc}")
                if self._export_subscribed and self._export_uuid:
                    try:
                        await self._connection_coordinator.stop_notify(
                            self._connection_token, self._client, self._export_uuid,
                            name="stop_export_disconnect",
                        )
                    except Exception as exc:  # pragma: no cover - host stack dependent
                        self._log(f"Stop Export Data notify failed: {exc}")
                if self._battery_subscribed and self._battery_level_char is not None:
                    try:
                        await self._connection_coordinator.stop_notify(
                            self._connection_token, self._client, self._battery_level_char,
                            name="stop_battery_disconnect",
                        )
                    except Exception as exc:  # pragma: no cover - host stack dependent
                        self._log(f"Stop Battery Level notify failed: {exc}")
                if self._calibration_tx_subscribed and self._calibration_tx_uuid:
                    try:
                        await self._connection_coordinator.stop_notify(
                            self._connection_token, self._client, self._calibration_tx_uuid,
                            name="stop_calibration_tx_disconnect",
                        )
                    except Exception as exc:  # pragma: no cover - host stack dependent
                        self._log(f"Stop Calibration TX notify failed: {exc}")
                if self._calibration_status_subscribed and self._calibration_status_uuid:
                    try:
                        await self._connection_coordinator.stop_notify(
                            self._connection_token, self._client, self._calibration_status_uuid,
                            name="stop_calibration_status_disconnect",
                        )
                    except Exception as exc:  # pragma: no cover - host stack dependent
                        self._log(f"Stop Calibration Status notify failed: {exc}")
                await self._stop_ota_dfu_entry_notify(self._client)
                self._expected_disconnect_client_ids.add(id(self._client))
                from .client_lifecycle import release_client
                await release_client(self._client, self._log)
                self._force_close_client_backend(self._client, reason="safe_disconnect")

            self._client = None
            self._connected_address = None
            self._ci_state.disconnected(reason="safe_disconnect")
            self._last_services_payload = []
            self._last_service_objects = []
            self._clear_control_state(
                preserve_start_operation=preserve_start_operation,
            )
            self._connection_coordinator.invalidate()
            await asyncio.sleep(0)
            self._log("Disconnected")
        except Exception as exc:  # pragma: no cover - host stack dependent
            self._log(f"Disconnect failed: {exc}")
            self._emit({"type": "error", "message": f"Disconnect failed: {exc}"})
        finally:
            clean = True
            if closing_client is not None:
                clean = self._force_close_client_backend(closing_client, reason="safe_disconnect_finally")
            if self._client is closing_client:
                self._client = None
                self._connected_address = None
                self._last_services_payload = []
                self._last_service_objects = []
                self._clear_control_state(preserve_start_operation=preserve_start_operation)
                self._connection_coordinator.invalidate()
            if clean is False:
                from .connection_recovery import ConnectionFailure
                raise ConnectionFailure("cleanup", "本应用 BLE 资源释放失败；请退出并重新启动上位机")

    def _clear_control_state(
        self,
        *,
        preserve_start_operation: bool | None = None,
    ) -> None:
        self._timeout_action = None
        self._timeout_action_client = None
        if preserve_start_operation is None:
            preserve_start_operation = self._online_start_recovery_preserve
        self._cancel_recovery_idle_timer()
        self._cancel_online_fault_tasks()
        self._recovery_only = False
        self._offline_observer_active = False
        self._offline_runtime_last_state = None
        self._connection_business_ready = False
        self.stress.reset()
        online_store_was_active = self._online_store_state.metadata.get("status") in {
            "active",
            "ending",
        }
        online_was_active = bool(self._online_active or online_store_was_active)
        self._online_io_epoch += 1
        if online_was_active:
            self._queue_online_abort("disconnect_before_online_terminal", terminal_source="host_disconnect")
        if online_was_active:
            # Publish a terminal local outcome before the connection-level
            # reset so the host never keeps waiting for a device ACK that can
            # no longer arrive.  The session is intentionally partial/aborted.
            self._emit_online_status(
                "aborted_partial",
                active=False,
                disconnect=True,
                last_error="device disconnected; current session discarded",
            )
        self._cancel_ci_observer()
        try:
            current_task = asyncio.current_task()
        except RuntimeError:
            current_task = None
        for task in (
            self._offline_observer_task,
            self._central_profile_task,
            self._central_heartbeat_task,
            self._central_handoff_task,
        ):
            if task is not None and not task.done() and task is not current_task:
                task.cancel()
        self._offline_observer_task = None
        self._central_profile_task = None
        self._central_heartbeat_task = None
        self._central_handoff_task = None
        if (
            self._central_enable_future is not None
            and not self._central_enable_future.done()
        ):
            self._central_enable_future.cancel()
        self._central_enable_future = None
        self._central_enable_seq = None
        if (
            self._bootstrap_query_ack_future is not None
            and not self._bootstrap_query_ack_future.done()
        ):
            self._bootstrap_query_ack_future.cancel()
        if (
            self._bootstrap_snapshot_future is not None
            and not self._bootstrap_snapshot_future.done()
        ):
            self._bootstrap_snapshot_future.cancel()
        self._bootstrap_query_ack_future = None
        self._bootstrap_snapshot_future = None
        self._bootstrap_query_seq = None
        self._bootstrap_generation = 0
        self._bootstrap_revision = None
        self._bootstrap_last_status = None
        self._bootstrap_ready_bits = 0
        self._bootstrap_device_state = None
        self._bootstrap_suggested_action = None
        self._bootstrap_stage = None
        self._bootstrap_reason = None
        self._bootstrap_snapshot_event.clear()
        self._device_info_text = ""
        if (
            self._connection_user_sync_ack_future is not None
            and not self._connection_user_sync_ack_future.done()
        ):
            self._connection_user_sync_ack_future.cancel()
        self._connection_user_sync_ack_future = None
        self._connection_user_sync_seq = None
        self._connection_user_context_supported = False
        self._connection_user_synced = False
        self._user_sync_result = UserSyncResult.FAILED
        self._capture_identity_attempted = False
        self._connection_requested_user_id = None
        self._connection_user_id = None
        self._connection_user_session_count = 0
        self._foreign_session_count = 0
        self._foreign_prompt_shown = False
        self._foreign_purge_device_active = False
        if (
            self._feature_config_retry_task is not None
            and not self._feature_config_retry_task.done()
            and self._feature_config_retry_task is not current_task
        ):
            self._feature_config_retry_task.cancel()
        self._feature_config_retry_task = None
        self._feature_config_supported = False
        self._feature_config_synced = False
        self._feature_config_pending = False
        self._feature_config_generation = 0
        self._feature_config_seq = None
        self._feature_config_transaction_id += 1
        self._feature_config_ack_queue = asyncio.Queue()
        if self._foreign_prompt_future is not None and not self._foreign_prompt_future.done():
            self._foreign_prompt_future.cancel()
        self._foreign_prompt_future = None
        if (
            self._central_applied_future is not None
            and not self._central_applied_future.done()
        ):
            self._central_applied_future.cancel()
        self._central_applied_future = None
        self._central_link_intent_event.clear()
        self._central_early_link_ack = None
        self._pending_actual_stable = None
        self._central_profiles.force_release(reason="clear_control_state")
        self._ota_high_speed_key = None
        # The entry coroutine owns this flag until its terminal event/finally.
        # Disconnect cleanup may run while its COMMIT write is still unwinding.
        self._central_mode_enabled = False
        self._central_balanced_applied_event.clear()
        self._offline_v2_commands_ready = False
        self._offline_priority_blocked = False
        self._offline_priority_phase = "idle"
        self._offline_v2_host_ci_deferred = False
        self._offline_v2_host_ci_defer_detail = 0
        if (
            self._offline_v2_host_ci_resume_task is not None
            and not self._offline_v2_host_ci_resume_task.done()
            and self._offline_v2_host_ci_resume_task is not current_task
        ):
            self._offline_v2_host_ci_resume_task.cancel()
        self._offline_v2_host_ci_resume_task = None
        self._central_hybrid_enabled = False
        self._central_session_state = "TERMINAL"
        self._central_session_id = 0
        self._central_generation = 0
        self._central_transition = 0
        self._central_profile_key = None
        if self._time_sync_ack_future is not None and not self._time_sync_ack_future.done():
            self._time_sync_ack_future.cancel()
        if self._online_ready_ack_future is not None and not self._online_ready_ack_future.done():
            self._online_ready_ack_future.cancel()
        self._cancel_online_ready_retry()
        self._cancel_online_start_final_timeout()
        self._cancel_offline_start_final_timeout()
        self._offline_start_seq = None
        if not preserve_start_operation:
            self._cancel_online_start_ci48_restore_wait()
        if self._ota_dfu_entry_ack_future is not None and not self._ota_dfu_entry_ack_future.done():
            self._ota_dfu_entry_ack_future.cancel()
        self._ota_dfu_entry_ack_future = None
        if (
            self._ota_link_intent_ack_future is not None
            and not self._ota_link_intent_ack_future.done()
        ):
            self._ota_link_intent_ack_future.cancel()
        self._ota_link_intent_ack_future = None
        self._ota_link_intent_seq = None
        if self._ota_prepare_ack_future is not None and not self._ota_prepare_ack_future.done():
            self._ota_prepare_ack_future.cancel()
        self._ota_prepare_ack_future = None
        self._ota_prepare_seq = None
        if self._ota_commit_ack_future is not None and not self._ota_commit_ack_future.done():
            self._ota_commit_ack_future.cancel()
        self._ota_commit_ack_future = None
        self._ota_commit_seq = None
        self._ota_commit_connection = None
        self._ota_commit_accepted_connection = None
        self._zy100_control_found = False
        self._command_found = False
        self._ack_found = False
        self._export_found = False
        self._command_uuid = None
        self._ack_uuid = None
        self._export_uuid = None
        self._ack_subscribed = False
        self._export_subscribed = False
        self._calibration_service_found = False
        self._calibration_info_uuid = None
        self._calibration_tx_uuid = None
        self._calibration_rx_uuid = None
        self._calibration_status_uuid = None
        self._calibration_tx_subscribed = False
        self._calibration_status_subscribed = False
        self._calibration_command_pending = False
        self._calibration_active = False
        self._calibration_receiver.reset()
        self._calibration_diag_request_transaction_id = None
        self._calibration_diag_request_pending = False
        self._calibration_diag_auto_pending = False
        self._calibration_diag_retry_used = False
        self._calibration_gate_state = "unknown"
        self._calibration_summary = {}
        self._latest_calibration_record = None
        if (
            self._calibration_cache_confirm_future is not None
            and not self._calibration_cache_confirm_future.done()
        ):
            self._calibration_cache_confirm_future.cancel()
        self._calibration_cache_confirm_future = None
        self._calibration_cache_confirm_transaction_id = None
        if (
            self._calibration_info_confirm_future is not None
            and not self._calibration_info_confirm_future.done()
        ):
            self._calibration_info_confirm_future.cancel()
        self._calibration_info_confirm_future = None
        self._calibration_info_confirm_transaction_id = None
        if (
            self._calibration_record_sync_future is not None
            and not self._calibration_record_sync_future.done()
        ):
            self._calibration_record_sync_future.cancel()
        self._calibration_record_sync_future = None
        self._calibration_record_ack_transaction_id = None
        self._calibration_record_request_active = False
        self._calibration_record_expected_summary = {}
        self._battery_level_char = None
        self._battery_level_uuid = None
        self._battery_subscribed = False
        self._battery_percent = None
        if (
            self._post_ready_info_task is not None
            and not self._post_ready_info_task.done()
            and self._post_ready_info_task is not current_task
        ):
            self._post_ready_info_task.cancel()
        self._post_ready_info_task = None
        self._ota_dfu_entry_notify_uuid = None
        self._ota_dfu_entry_notify_subscribed = False
        self._notify_ready = False
        self._rtc_sync_ok = False
        self._last_device_state = None
        self._active_user_id = None
        self._active_training_id = None
        self._last_rtc_sync_ms = None
        self._cancel_online_summary_ack_tasks()
        self._online_receiver.reset()
        old_store = self._online_store
        self._online_store = OnlineSessionStore(old_store.root, calibration_store=old_store.calibration_store)
        self._online_ready_sent = False
        self._online_ready_ack_ok = False
        self._online_ready_seq = None
        self._online_ready_ack_future = None
        self._online_start_seq = None
        self._cancel_offline_start_final_timeout()
        self._offline_start_seq = None
        if not preserve_start_operation:
            recovery_task = self._online_start_ci_recovery_task
            if (
                recovery_task is not None
                and not recovery_task.done()
                and recovery_task is not asyncio.current_task()
            ):
                recovery_task.cancel()
            self._online_start_ci_recovery_task = None
            if self._online_start_operation_state != START_OP_IDLE:
                self._set_online_start_operation_state(
                    START_OP_TERMINAL,
                    reason="control_state_cleared",
                )
                self._set_online_start_operation_state(
                    START_OP_IDLE,
                    reason="control_state_clear_complete",
                )
            self._online_start_ci_recovery_used = False
            self._online_start_attempt = 0
        self._online_active = False
        self._online_status = "idle"
        self._online_session_id = None
        self._online_summary_ack_batch = ONLINE_SUMMARY_ACK_BATCH_DEFAULT
        self._online_summary_pending = []
        self._online_started_perf = 0.0
        self._online_stats = self._new_online_stats()
        self._stop_online_diag()
        self._cancel_export_parse_task()
        self._export_pipeline.reset()
        self._export_speed.force_release(reason="clear_control_state")
        self._export_receiver.reset()
        self._cancel_failed_export_timeout()
        self._reset_pending_confirm_context()
        self._reset_clear_flash_context()
        if self._clear_flash_result_future is not None and not self._clear_flash_result_future.done():
            self._clear_flash_result_future.cancel()
        self._clear_flash_result_future = None
        self._export_commands_blocked = False
        self._pending_command_active = False
        self._shipping_operation_active = False
        self._shipping_seq = None
        self._shipping_accepted = False
        self._clear_pending_time_sync()
        self._device_channel_supported = False
        self._device_channel_received = False
        self._device_channel_value = None

    def _handle_disconnected(self, _: BleakClient) -> None:
        if _ is not None and _ is not self._client and id(_) not in self._expected_disconnect_client_ids:
            self._log("[BLE_DISCONNECT] ignored stale client")
            return
        notice = getattr(self, "_timeout_action", None)
        standby_timeout = bool(notice and notice["reason"] == TIMEOUT_ACTION_STANDBY
                               and notice["connection_token"] == self._connection_token
                               and getattr(self, "_timeout_action_client", None) is _)
        ota_disconnect = (
            self._ota_commit_accepted_connection == (_, self._connection_token)
            and _ is self._client
        )
        disconnect_reason = ("ota_commit_disconnect" if ota_disconnect else
                             "standby_timeout" if standby_timeout else "unexpected_disconnect")
        shipping_disconnect = self._shipping_operation_active
        if shipping_disconnect:
            self._expected_disconnect_client_ids.discard(id(_))
            self._emit(
                {
                    "type": "shipping_status",
                    "status": "disconnected" if self._shipping_accepted else "failed",
                    "message": (
                        "设备已断开，船运模式已执行；请插入 VIN 唤醒设备"
                        if self._shipping_accepted
                        else "进入船运模式过程中设备断开，结果待确认"
                    ),
                }
            )
        if not shipping_disconnect and id(_) in self._expected_disconnect_client_ids:
            self._expected_disconnect_client_ids.discard(id(_))
            self._link_trace.record(
                "expected_disconnect",
                gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
            )
            self._log("[BLE_DISCONNECT] expected")
            return

        self._link_trace.record(
            disconnect_reason,
            client_is_connected=getattr(_, "is_connected", None),
            gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
        )
        if not standby_timeout and not ota_disconnect:
            self._loop.create_task(
                self._link_trace.dump(
                    "unexpected_disconnect",
                    gatt=self._connection_coordinator.gatt_diagnostic_snapshot(),
                )
            )

        if (
            self._pending_confirm_session_key
            and self._pending_confirm_ack_ok
            and not self._pending_confirm_final_ack_ok
        ):
            self._loop.create_task(
                self._mark_reclaim_unknown_for_session(
                    self._pending_confirm_session_key,
                    self._pending_confirm_export_id or 0,
                    reason="disconnect_before_final_reclaim_ack",
                )
            )
        self._log_online_diag_final(disconnect_reason)
        self._stop_online_diag()
        self._loop.create_task(self._offline_v2_sync.on_disconnect())
        self._client = None
        self._connected_address = None
        self._ci_state.disconnected(reason=disconnect_reason)
        self._last_services_payload = []
        self._last_service_objects = []
        self._clear_control_state()
        self._connection_coordinator.invalidate()
        if ota_disconnect:
            self._log("[BLE_DISCONNECT] ota_commit_disconnect expected=1")
        elif standby_timeout:
            self._emit({"type": "timeout_action_state", **notice,
                        "message": "设备因待机超时主动断开连接"})
            self._log("Device disconnected: standby timeout")
        else:
            self._log("Device disconnected unexpectedly")

    def shutdown(self) -> bool:
        """Leave the loop alive on timeout: active OS writes cannot be cancelled."""
        if self._loop.is_closed():
            return True
        try:
            future = self._submit(self._cancel_connection_and_disconnect())
            future.result(timeout=30)
        except Exception as exc:
            self._log(f"[ONLINE_SHUTDOWN] incomplete=1 reason={exc}")
            return False

        self._online_storage.close()
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            return False
        if not self._loop.is_closed():
            self._loop.close()
        return True
