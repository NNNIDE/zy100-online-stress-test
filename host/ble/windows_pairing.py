"""Resolve pairing on BLE association endpoints, never service interfaces."""
from dataclasses import dataclass
import asyncio
import re
from typing import Any


def normalize_address(value: str) -> str:
    compact = value.replace(":", "").replace("-", "").upper()
    if not re.fullmatch(r"[0-9A-F]{12}", compact):
        return ""
    return ":".join(compact[i:i+2] for i in range(0, 12, 2))


@dataclass(frozen=True)
class PairingTarget:
    state: str
    address: str
    endpoint_id: str = ""
    info: Any = None
    error: str = ""


async def resolve_pairing_target(address: str) -> PairingTarget:
    target = normalize_address(address)
    if not target:
        return PairingTarget("unknown", address, error="invalid address")
    try:
        from winrt.windows.devices.bluetooth import BluetoothLEDevice
        from winrt.windows.devices.enumeration import DeviceInformation, DeviceInformationKind
        for paired in (True, False):
            selector = BluetoothLEDevice.get_device_selector_from_pairing_state(paired)
            infos = await asyncio.wait_for(
                DeviceInformation.find_all_async_with_kind_aqs_filter_and_additional_properties(
                    selector, ["System.Devices.Aep.DeviceAddress"], DeviceInformationKind.ASSOCIATION_ENDPOINT),
                timeout=5.0)
            matches = []
            for info in infos:
                # BLE AEP IDs end in the remote address, unlike BTHLEDevice interfaces.
                suffix = str(info.id).rsplit("-", 1)[-1]
                if normalize_address(suffix) == target:
                    matches.append(info)
            if len(matches) > 1:
                return PairingTarget("unknown", target, error=f"endpoint matches={len(matches)}")
            if matches:
                info = matches[0]
                return PairingTarget("paired" if info.pairing.is_paired else "unpaired", target, str(info.id), info)
        return PairingTarget("unknown", target, error="endpoint not found")
    except Exception as exc:
        return PairingTarget("unknown", target, error=f"{type(exc).__name__}: {exc}")
