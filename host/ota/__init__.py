"""OTA package for the RTL8762D desktop BLE tool."""

from .dfu_protocol import DfuProtocolConfig
from .image_info import OtaImageInfo, load_ota_image_info
from .ota_manager import OtaManager, OtaPreflightResult, OtaServiceDiscoveryResult, OtaStartResult
from .ota_state import OtaState
from .realtek_dfu import RealtekDfuOtaClient, load_realtek_app_image

__all__ = [
    "DfuProtocolConfig",
    "OtaImageInfo",
    "OtaManager",
    "OtaPreflightResult",
    "OtaServiceDiscoveryResult",
    "OtaStartResult",
    "OtaState",
    "RealtekDfuOtaClient",
    "load_ota_image_info",
    "load_realtek_app_image",
]
