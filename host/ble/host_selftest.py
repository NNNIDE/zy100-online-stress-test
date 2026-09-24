from __future__ import annotations

import asyncio
import importlib
import json
import os
import platform
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Awaitable, Callable, Iterable

from .export_pipeline import EXPORT_NOTIFY_QUEUE_MAX
from .export_speed import (
    BleSpeedSnapshot,
    FIXED_CONN_PARAM_CONTROL,
    FIXED_CONN_PARAM_REASON,
    WinRtExportSpeedAdapter,
    export_target_ci_ms,
    format_ci_ms,
    observe_connection_interval,
)
from .zy100_protocol import (
    DEVICE_NAME_PREFIX,
    PREFERRED_DEVICE_NAME,
    ZY100_ACK_UUID,
    ZY100_COMMAND_UUID,
    ZY100_CONTROL_SERVICE_UUID,
    ZY100_EXPORT_DATA_UUID,
)


LogCallback = Callable[[str], None]

HOST_BLE_SELFTEST_PREFIX = "[HOST_BLE_SELFTEST]"
ENABLE_HOST_BLE_SELF_CHECK = "ENABLE_HOST_BLE_SELF_CHECK"
ENABLE_HOST_BLE_SELF_CHECK_THROUGHPUT = "ENABLE_HOST_BLE_SELF_CHECK_THROUGHPUT"
HOST_BLE_SELF_CHECK_REDACT = "HOST_BLE_SELF_CHECK_REDACT"

CAPABILITY_LEVEL_RUNTIME_60MS = "runtime_60ms"
CAPABILITY_LEVEL_RUNTIME_60MS_NOT_REACHED = "runtime_60ms_not_reached"
CAPABILITY_LEVEL_APP_PIPELINE = "app_pipeline_bottleneck"
CAPABILITY_LEVEL_API_UNAVAILABLE = "api_unavailable"
CAPABILITY_LEVEL_UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class ReportWriteResult:
    json_path: str
    txt_path: str


def env_flag_enabled(name: str) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return False
    return raw.strip().lower() in {"1", "true", "on", "yes"}


def self_check_enabled() -> bool:
    return env_flag_enabled(ENABLE_HOST_BLE_SELF_CHECK)


def throughput_probe_enabled() -> bool:
    return env_flag_enabled(ENABLE_HOST_BLE_SELF_CHECK_THROUGHPUT)


def redact_enabled() -> bool:
    return env_flag_enabled(HOST_BLE_SELF_CHECK_REDACT)


def selftest_log(log: LogCallback, message: str) -> None:
    try:
        log(f"{HOST_BLE_SELFTEST_PREFIX} {message}")
    except Exception:
        pass


async def get_bleak_services(client: Any) -> list[Any]:
    """Return services without calling removed bleak APIs first."""
    services_error: Exception | None = None
    try:
        services = getattr(client, "services")
        if services is not None:
            return list(services)
    except Exception as exc:
        services_error = exc

    get_services = getattr(client, "get_services", None)
    if callable(get_services):
        result = get_services()
        if hasattr(result, "__await__"):
            result = await result
        if result is not None:
            return list(result)

    if services_error is not None:
        raise services_error
    return []


async def run_host_ble_self_check_offline(
    *,
    log: LogCallback,
    report_dir: str | Path = "logs",
    scan: bool = True,
    scan_timeout_s: float = 3.0,
    write_report: bool = True,
    command_timeout_s: float = 5.0,
) -> dict[str, Any]:
    report: dict[str, Any] = _base_report("offline")
    report["python"] = _python_info(redact=redact_enabled())
    report["os"] = _os_info()
    report["bleak"] = _bleak_info()
    report["winrt"] = inspect_winrt_api()
    report["windows_api"] = _windows_api_info(report["winrt"])

    adapter_task = asyncio.create_task(_adapter_info(command_timeout_s=command_timeout_s))
    service_task = asyncio.create_task(_bluetooth_service_info(command_timeout_s=command_timeout_s))
    scan_task = (
        asyncio.create_task(_scan_info(scan_timeout_s=scan_timeout_s))
        if scan
        else _ready_task({"enabled": False, "reason": "not_requested"})
    )
    report["adapter"], report["services"], report["scan"] = await asyncio.gather(
        adapter_task,
        service_task,
        scan_task,
    )

    report["capability"] = evaluate_capability(report)
    report["recommendation"] = _recommendations(report["capability"])
    _log_offline_summary(log, report)
    if write_report:
        report["report_files"] = asdict(await write_self_check_report(report, report_dir=report_dir))
        selftest_log(log, f"report json={report['report_files']['json_path']}")
        selftest_log(log, f"report txt={report['report_files']['txt_path']}")
    return report


async def run_host_ble_self_check_online(
    *,
    client: Any,
    state: dict[str, Any],
    log: LogCallback,
    report_dir: str | Path = "logs",
    write_report: bool = True,
    throughput_wait_s: float = 1.0,
) -> dict[str, Any]:
    report = _base_report("online")
    report["bleak"] = _bleak_info()
    report["winrt"] = inspect_winrt_api()
    report["state"] = dict(state)

    try:
        services = await get_bleak_services(client)
    except Exception as exc:
        services = []
        report["gatt_error"] = str(exc)

    report["gatt"] = _gatt_info(services, state)
    report["notify_pipeline"] = _notify_pipeline_info(state)
    before = read_connection_details(client)
    report["connection_parameters"] = {"before": before}
    report["phy"] = before.get("phy", {"tx": "unknown", "rx": "unknown"})

    report["throughput_request"] = await _maybe_probe_throughput(
        client=client,
        state=state,
        log=log,
        before=before,
        wait_s=throughput_wait_s,
    )
    if report["throughput_request"].get("after") is not None:
        report["connection_parameters"]["after_throughput"] = report["throughput_request"]["after"]
    if report["throughput_request"].get("restored") is not None:
        report["connection_parameters"]["after_release"] = report["throughput_request"]["restored"]

    report["capability"] = evaluate_capability(report)
    report["recommendation"] = _recommendations(report["capability"])
    _log_online_summary(log, report)
    if write_report:
        report["report_files"] = asdict(await write_self_check_report(report, report_dir=report_dir))
        selftest_log(log, f"report json={report['report_files']['json_path']}")
        selftest_log(log, f"report txt={report['report_files']['txt_path']}")
    return report


def read_connection_details(client: Any) -> dict[str, Any]:
    requester = _get_winrt_requester(client)
    result: dict[str, Any] = {
        "backend": _backend_name(client),
        "available": requester is not None,
        "ci_raw": "unknown",
        "ci_raw_type": "unknown",
        "ci_ms": "unknown",
        "latency": "unknown",
        "timeout_ms": "unknown",
        "phy": {"tx": "unknown", "rx": "unknown"},
    }
    if requester is None:
        result["reason"] = "not_winrt_backend"
        return result

    try:
        params = requester.get_connection_parameters()
        raw_ci = getattr(params, "connection_interval", None)
        observed = observe_connection_interval(raw_ci)
        result["ci_raw"] = observed.ci_raw
        result["ci_raw_type"] = observed.ci_raw_type
        result["ci_ms"] = "unknown" if observed.ci_ms is None else format_ci_ms(observed.ci_ms)
        result["latency"] = _safe_int_or_unknown(getattr(params, "connection_latency", None))
        timeout_units = _safe_float_or_none(getattr(params, "link_timeout", None))
        result["timeout_ms"] = "unknown" if timeout_units is None else format_ci_ms(timeout_units * 10.0)
    except Exception as exc:
        result["reason"] = f"read_conn_params_failed:{exc}"

    try:
        phy = requester.get_connection_phy()
        phy_name = _phy_name(phy)
        result["phy"] = {"tx": phy_name, "rx": phy_name}
    except Exception:
        result["phy"] = {"tx": "unknown", "rx": "unknown"}
    return result


def inspect_winrt_api() -> dict[str, Any]:
    modules: dict[str, str] = {}
    winrt_available = True
    for module_name in (
        "winrt",
        "winrt.windows.devices.bluetooth",
        "winrt.windows.devices.bluetooth.genericattributeprofile",
    ):
        try:
            importlib.import_module(module_name)
            modules[module_name] = "available"
        except Exception:
            modules[module_name] = "missing"
            winrt_available = False

    bluetooth_module = None
    try:
        bluetooth_module = importlib.import_module("winrt.windows.devices.bluetooth")
    except Exception:
        pass

    bluetooth_device = getattr(bluetooth_module, "BluetoothLEDevice", None) if bluetooth_module else None
    preferred = (
        getattr(bluetooth_module, "BluetoothLEPreferredConnectionParameters", None)
        if bluetooth_module
        else None
    )
    throughput = getattr(preferred, "throughput_optimized", None) if preferred is not None else None
    result = {
        "winrt_available": bool(winrt_available and bluetooth_module is not None),
        "pywinrt_modules": modules,
        "bluetooth_le_device": "available" if bluetooth_device is not None else "missing",
        "preferred_connection_parameters": "available" if preferred is not None else "missing",
        "preferred_conn_param_api": _api_status(bluetooth_device, "request_preferred_connection_parameters"),
        "throughput_optimized_api": "available" if throughput is not None else "missing",
        "read_conn_params_api": _api_status(bluetooth_device, "get_connection_parameters"),
        "connection_parameters_changed_api": _api_status(bluetooth_device, "connection_parameters_changed"),
        "read_phy_api": _api_status(bluetooth_device, "get_connection_phy"),
    }
    return result


def evaluate_capability(report: dict[str, Any]) -> dict[str, Any]:
    winrt = report.get("winrt", {})
    throughput_api_available = winrt.get("throughput_optimized_api") == "available"
    preferred_api_available = winrt.get("preferred_conn_param_api") == "available"

    conn = report.get("connection_parameters", {})
    before = conn.get("before", {})
    after = conn.get("after_throughput", {})
    ci_values = [
        _ci_ms_or_none(before),
        _ci_ms_or_none(after),
        _ci_ms_or_none(conn.get("after_release", {})),
    ]
    observed = [value for value in ci_values if value is not None]
    ci_min = min(observed) if observed else None

    throughput = report.get("throughput_request", {})
    request_result = str(throughput.get("result", "not_run"))
    request_accepted = _tri_state(throughput.get("request_accepted"))
    effective_changed = _effective_changed(_ci_ms_or_none(before), _ci_ms_or_none(after))
    target_ms = export_target_ci_ms()
    target_reached = _target_reached(_ci_ms_or_none(after) or _ci_ms_or_none(before), target_ms)

    pipeline = report.get("notify_pipeline", {})
    callback_avg_us = _safe_float_or_none(pipeline.get("callback_avg_us"))
    queue_overflow = _safe_int_or_zero(pipeline.get("queue_overflow"))

    capability_level = _capability_level(
        api_available=throughput_api_available or preferred_api_available,
        request_result=request_result,
        callback_avg_us=callback_avg_us,
        queue_overflow=queue_overflow,
        target_reached=target_reached,
    )

    return {
        "winrt_backend": str(before.get("backend", "unknown")).lower() == "winrt",
        "preferred_connection_parameters_api": preferred_api_available,
        "throughput_optimized_api": throughput_api_available,
        "api_available": throughput_api_available or preferred_api_available,
        "request_result": request_result,
        "request_accepted": request_accepted,
        "effective_changed": effective_changed,
        "target_reached": target_reached,
        "ci_min_observed_ms": "unknown" if ci_min is None else format_ci_ms(ci_min),
        "ci_exact_control": False,
        "ci_control_method": FIXED_CONN_PARAM_CONTROL,
        "mtu_effective": _mtu_from_report(report),
        "phy_effective": before.get("phy", {}).get("tx", "unknown"),
        "capability_level": capability_level,
    }


async def write_self_check_report(report: dict[str, Any], *, report_dir: str | Path = "logs") -> ReportWriteResult:
    report_for_disk = redact_report(report) if redact_enabled() else dict(report)
    report_for_disk["privacy_notice"] = (
        "redacted"
        if redact_enabled()
        else "contains local machine and device identifiers"
    )
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(report_dir)
    await asyncio.to_thread(root.mkdir, parents=True, exist_ok=True)
    json_path = root / f"ble_host_selftest_{timestamp}.json"
    txt_path = root / f"ble_host_selftest_{timestamp}.txt"
    await asyncio.to_thread(_write_json, json_path, report_for_disk)
    await asyncio.to_thread(_write_text, txt_path, _format_text_report(report_for_disk))
    return ReportWriteResult(json_path=str(json_path), txt_path=str(txt_path))


def redact_report(value: Any) -> Any:
    return _redact(value, key="")


def snapshot_to_connection_details(snapshot: BleSpeedSnapshot) -> dict[str, Any]:
    return {
        "backend": snapshot.backend,
        "ci_raw": snapshot.ci_raw,
        "ci_raw_type": snapshot.ci_raw_type,
        "ci_ms": snapshot.ci_effective_ms,
        "latency": "unknown",
        "timeout_ms": "unknown",
        "phy": {"tx": snapshot.phy_effective, "rx": snapshot.phy_effective},
    }


async def _maybe_probe_throughput(
    *,
    client: Any,
    state: dict[str, Any],
    log: LogCallback,
    before: dict[str, Any],
    wait_s: float,
) -> dict[str, Any]:
    if not throughput_probe_enabled():
        selftest_log(log, "throughput_request attempted=0 skipped reason=disabled")
        return {"attempted": 0, "result": "skipped", "reason": "disabled", "before": before}

    busy_reason = str(state.get("busy_reason") or "")
    if busy_reason:
        selftest_log(log, f"throughput_request attempted=0 skipped reason=busy_state detail={busy_reason}")
        return {
            "attempted": 0,
            "result": "skipped",
            "reason": "busy_state",
            "busy_detail": busy_reason,
            "before": before,
        }

    adapter = WinRtExportSpeedAdapter(lambda message: selftest_log(log, f"speed {message}"))
    result: dict[str, Any] = {"attempted": 1, "before": before}
    try:
        entered = await adapter.enter(client, reason="host_self_check")
        if entered.conn_param_control == FIXED_CONN_PARAM_CONTROL:
            after = read_connection_details(client)
            result.update(
                {
                    "attempted": 0,
                    "result": "skipped",
                    "reason": FIXED_CONN_PARAM_REASON,
                    "request_accepted": False,
                    "after": after,
                    "throughput_effective": _effective_changed(
                        _ci_ms_or_none(before),
                        _ci_ms_or_none(after),
                    ),
                }
            )
            selftest_log(log, f"throughput_request attempted=0 skipped reason={FIXED_CONN_PARAM_REASON}")
            return result
        result["request_accepted"] = entered.conn_param_control == "winrt_throughput_optimized"
        if entered.conn_param_control == "unsupported":
            result["result"] = "unsupported"
        elif entered.conn_param_control == "failed":
            result["result"] = "fail"
        else:
            result["result"] = "ok"
        if wait_s > 0:
            await asyncio.sleep(wait_s)
        after = read_connection_details(client)
        result["after"] = after
        await adapter.restore(client, reason="host_self_check")
        if wait_s > 0:
            await asyncio.sleep(wait_s)
        result["restored"] = read_connection_details(client)
        result["throughput_effective"] = _effective_changed(_ci_ms_or_none(before), _ci_ms_or_none(after))
        selftest_log(log, f"throughput_request attempted=1 result={result['result']}")
        selftest_log(log, f"conn before ci_ms={before.get('ci_ms', 'unknown')}")
        selftest_log(log, f"conn after ci_ms={after.get('ci_ms', 'unknown')}")
        selftest_log(log, f"conn restored ci_ms={result['restored'].get('ci_ms', 'unknown')}")
        selftest_log(log, f"throughput_effective={result['throughput_effective']}")
        return result
    except Exception as exc:
        try:
            await adapter.restore(client, reason="host_self_check_error")
        except Exception:
            pass
        result["result"] = "fail"
        result["error"] = str(exc)
        result["request_accepted"] = False
        result["throughput_effective"] = "unknown"
        selftest_log(log, f"throughput_request attempted=1 result=fail reason={exc}")
        return result


def _base_report(kind: str) -> dict[str, Any]:
    return {
        "kind": kind,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "privacy_notice": "redacted" if redact_enabled() else "contains local machine and device identifiers",
    }


def _python_info(*, redact: bool) -> dict[str, Any]:
    return {
        "version": sys.version.replace("\n", " "),
        "executable": "<redacted>" if redact else sys.executable,
        "architecture": platform.architecture()[0],
        "asyncio_event_loop_policy": type(asyncio.get_event_loop_policy()).__name__,
    }


def _os_info() -> dict[str, Any]:
    win32 = platform.win32_ver()
    return {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": platform.machine(),
        "windows_build": win32[1] or platform.version() or "unknown",
    }


def _bleak_info() -> dict[str, Any]:
    version = "unknown"
    try:
        from importlib import metadata

        version = metadata.version("bleak")
    except Exception:
        try:
            bleak = importlib.import_module("bleak")
            version = str(getattr(bleak, "__version__", "unknown"))
        except Exception:
            pass

    try:
        from bleak import BleakClient

        client_module = getattr(BleakClient, "__module__", "unknown")
        client_class = getattr(BleakClient, "__name__", "BleakClient")
        services_available = hasattr(BleakClient, "services")
        get_services_available = hasattr(BleakClient, "get_services")
    except Exception:
        client_module = "unknown"
        client_class = "unknown"
        services_available = False
        get_services_available = False

    backend = "winrt" if sys.platform == "win32" else "unknown"
    return {
        "version": version,
        "backend_name": backend,
        "client_class": client_class,
        "client_module": client_module,
        "winrt_backend": backend == "winrt",
        "bleak_get_services_api": "available" if get_services_available else "missing",
        "bleak_services_property": "available" if services_available else "missing",
        "gatt_discovery_api": "client.services",
    }


def _windows_api_info(winrt: dict[str, Any]) -> dict[str, Any]:
    contract = "unknown"
    try:
        metadata_module = importlib.import_module("winrt.windows.foundation.metadata")
        api_info = getattr(metadata_module, "ApiInformation", None)
        if api_info is not None:
            highest = 0
            for version in range(1, 25):
                try:
                    if api_info.is_api_contract_present("Windows.Foundation.UniversalApiContract", version):
                        highest = version
                except Exception:
                    break
            contract = str(highest) if highest else "unknown"
    except Exception:
        contract = "unknown"
    supported: int | str
    if winrt.get("preferred_conn_param_api") == "available":
        supported = 1
    elif winrt.get("winrt_available"):
        supported = 0
    else:
        supported = "unknown"
    return {
        "windows_build": _os_info().get("windows_build", "unknown"),
        "universal_api_contract": contract,
        "request_preferred_conn_params_supported": supported,
    }


async def _adapter_info(*, command_timeout_s: float) -> dict[str, Any]:
    script = r"""
$ErrorActionPreference='SilentlyContinue'
$devices = Get-PnpDevice -Class Bluetooth | Select-Object -First 8 FriendlyName,InstanceId,Manufacturer,Status,Class
$drivers = Get-CimInstance Win32_PnPSignedDriver | Where-Object { $_.DeviceClass -eq 'Bluetooth' -or $_.DeviceName -like '*Bluetooth*' } | Select-Object -First 8 DeviceName,DriverProviderName,DriverVersion,DriverDate,Manufacturer,DeviceID
[pscustomobject]@{devices=$devices;drivers=$drivers} | ConvertTo-Json -Depth 5
"""
    parsed = await _run_powershell_json(script, timeout_s=command_timeout_s)
    if not isinstance(parsed, dict):
        return _unknown_adapter()
    devices = _as_list(parsed.get("devices"))
    drivers = _as_list(parsed.get("drivers"))
    first_device = devices[0] if devices else {}
    first_driver = drivers[0] if drivers else {}
    hwid = str(first_device.get("InstanceId") or first_driver.get("DeviceID") or "unknown")
    return {
        "name": first_device.get("FriendlyName") or first_driver.get("DeviceName") or "unknown",
        "hwid": hwid,
        "manufacturer": first_device.get("Manufacturer") or first_driver.get("Manufacturer") or "unknown",
        "driver_provider": first_driver.get("DriverProviderName") or "unknown",
        "driver_version": first_driver.get("DriverVersion") or "unknown",
        "driver_date": first_driver.get("DriverDate") or "unknown",
        "bus": _bus_from_hwid(hwid),
        "raw_count": {"devices": len(devices), "drivers": len(drivers)},
    }


async def _bluetooth_service_info(*, command_timeout_s: float) -> dict[str, Any]:
    script = r"""
$ErrorActionPreference='SilentlyContinue'
$bth = Get-Service bthserv
$das = Get-Service DeviceAssociationService
[pscustomobject]@{bthserv=$bth.Status.ToString();deviceassociationservice=$das.Status.ToString()} | ConvertTo-Json -Depth 3
"""
    parsed = await _run_powershell_json(script, timeout_s=command_timeout_s)
    if not isinstance(parsed, dict):
        return {
            "bthserv": "unknown",
            "deviceassociationservice": "unknown",
            "bluetooth_radio_state": "unknown",
        }
    return {
        "bthserv": str(parsed.get("bthserv") or "unknown").lower(),
        "deviceassociationservice": str(parsed.get("deviceassociationservice") or "unknown").lower(),
        "bluetooth_radio_state": "unknown",
    }


async def _scan_info(*, scan_timeout_s: float) -> dict[str, Any]:
    started = datetime.now()
    try:
        from bleak import BleakScanner

        discovered = await BleakScanner.discover(timeout=scan_timeout_s, return_adv=True)
    except Exception as exc:
        return {
            "duration_ms": int(scan_timeout_s * 1000),
            "discovered": "unknown",
            "target_found_count": "unknown",
            "error": str(exc),
        }

    items: Iterable[Any]
    if isinstance(discovered, dict):
        items = discovered.values()
    else:
        items = [(device, None) for device in discovered]
    target_devices: list[dict[str, Any]] = []
    names: dict[str, int] = {}
    addresses: dict[str, int] = {}
    total = 0
    for item in items:
        total += 1
        if isinstance(item, tuple) and len(item) == 2:
            device, adv = item
        else:
            device, adv = item, None
        name = str(getattr(device, "name", "") or getattr(adv, "local_name", "") or "")
        address = str(getattr(device, "address", "") or "")
        if name:
            names[name] = names.get(name, 0) + 1
        if address:
            addresses[address] = addresses.get(address, 0) + 1
        if name.startswith(DEVICE_NAME_PREFIX) or name == PREFERRED_DEVICE_NAME:
            target_devices.append(
                {
                    "name": name,
                    "address": address,
                    "rssi": getattr(adv, "rssi", getattr(device, "rssi", "unknown")),
                    "address_type": getattr(device, "address_type", "unknown"),
                }
            )
    elapsed_ms = int((datetime.now() - started).total_seconds() * 1000)
    return {
        "duration_ms": elapsed_ms,
        "discovered": total,
        "target_found_count": len(target_devices),
        "targets": target_devices,
        "duplicate_names": sorted(name for name, count in names.items() if count > 1),
        "duplicate_addresses": sorted(address for address, count in addresses.items() if count > 1),
    }


def _gatt_info(services: list[Any], state: dict[str, Any]) -> dict[str, Any]:
    char_count = sum(len(list(getattr(service, "characteristics", []) or [])) for service in services)
    service_uuids = {str(getattr(service, "uuid", "")).lower() for service in services}
    char_uuids = {
        str(getattr(char, "uuid", "")).lower()
        for service in services
        for char in list(getattr(service, "characteristics", []) or [])
    }
    return {
        "service_count": len(services),
        "characteristic_count": char_count,
        "control_found": ZY100_CONTROL_SERVICE_UUID in service_uuids,
        "command_found": ZY100_COMMAND_UUID in char_uuids,
        "ack_found": ZY100_ACK_UUID in char_uuids,
        "export_found": ZY100_EXPORT_DATA_UUID in char_uuids,
        "ack_notify_subscribed": bool(state.get("ack_subscribed")),
        "export_notify_subscribed": bool(state.get("export_subscribed")),
        "mtu_effective": state.get("mtu_effective", "unknown"),
    }


def _notify_pipeline_info(state: dict[str, Any]) -> dict[str, Any]:
    return {
        "callback_lightweight": 1,
        "queue_capacity": state.get("queue_capacity", EXPORT_NOTIFY_QUEUE_MAX),
        "parser_worker": state.get("parser_worker", "unknown"),
        "ui_throttle_ms": state.get("ui_throttle_ms", 200),
        "per_packet_log": 0,
        "queue_overflow": state.get("queue_overflow", 0),
        "callback_avg_us": state.get("callback_avg_us", 0.0),
    }


async def _run_powershell_json(script: str, *, timeout_s: float) -> Any:
    return await asyncio.to_thread(_run_powershell_json_sync, script, timeout_s)


def _run_powershell_json_sync(script: str, timeout_s: float) -> Any:
    try:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        kwargs: dict[str, Any] = {
            "capture_output": True,
            "text": True,
            "timeout": timeout_s,
        }
        if creationflags:
            kwargs["creationflags"] = creationflags
        completed = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script],
            **kwargs,
        )
    except Exception:
        return None
    if completed.returncode != 0 or not completed.stdout.strip():
        return None
    try:
        return json.loads(completed.stdout)
    except Exception:
        return None


def _log_offline_summary(log: LogCallback, report: dict[str, Any]) -> None:
    py = report.get("python", {})
    os_info = report.get("os", {})
    bleak = report.get("bleak", {})
    winrt = report.get("winrt", {})
    adapter = report.get("adapter", {})
    scan = report.get("scan", {})
    capability = report.get("capability", {})
    selftest_log(log, f"python version={py.get('version', 'unknown')}")
    selftest_log(log, f"python exe={py.get('executable', 'unknown')}")
    selftest_log(log, f"python arch={py.get('architecture', 'unknown')}")
    selftest_log(
        log,
        f"os system={os_info.get('system', 'unknown')} release={os_info.get('release', 'unknown')} "
        f"version={os_info.get('version', 'unknown')}",
    )
    selftest_log(log, f"bleak version={bleak.get('version', 'unknown')} backend={bleak.get('backend_name', 'unknown')}")
    selftest_log(log, f"bleak_get_services_api={bleak.get('bleak_get_services_api', 'unknown')}")
    selftest_log(log, f"bleak_services_property={bleak.get('bleak_services_property', 'unknown')}")
    selftest_log(log, f"gatt_discovery_api={bleak.get('gatt_discovery_api', 'unknown')}")
    selftest_log(log, f"winrt_available={1 if winrt.get('winrt_available') else 0}")
    selftest_log(log, f"throughput_optimized_api={winrt.get('throughput_optimized_api', 'unknown')}")
    selftest_log(log, f"read_conn_params_api={winrt.get('read_conn_params_api', 'unknown')}")
    selftest_log(log, f"adapter name={adapter.get('name', 'unknown')}")
    selftest_log(log, f"adapter driver_version={adapter.get('driver_version', 'unknown')}")
    selftest_log(log, f"adapter bus={adapter.get('bus', 'unknown')}")
    selftest_log(log, f"scan duration_ms={scan.get('duration_ms', 'unknown')} discovered={scan.get('discovered', 'unknown')}")
    selftest_log(log, f"target found={scan.get('target_found_count', 'unknown')}")
    selftest_log(log, f"capability_level={capability.get('capability_level', 'unknown')}")


def _log_online_summary(log: LogCallback, report: dict[str, Any]) -> None:
    gatt = report.get("gatt", {})
    notify = report.get("notify_pipeline", {})
    conn = report.get("connection_parameters", {}).get("before", {})
    throughput = report.get("throughput_request", {})
    capability = report.get("capability", {})
    selftest_log(log, f"gatt service_count={gatt.get('service_count', 'unknown')}")
    selftest_log(
        log,
        "gatt "
        f"control_found={1 if gatt.get('control_found') else 0} "
        f"command_found={1 if gatt.get('command_found') else 0} "
        f"ack_found={1 if gatt.get('ack_found') else 0} "
        f"export_found={1 if gatt.get('export_found') else 0}",
    )
    selftest_log(log, f"mtu_effective={gatt.get('mtu_effective', 'unknown')}")
    selftest_log(log, f"notify_pipeline callback_lightweight={notify.get('callback_lightweight', 'unknown')}")
    selftest_log(log, f"notify_pipeline queue_capacity={notify.get('queue_capacity', 'unknown')}")
    selftest_log(log, f"notify_pipeline parser_worker={notify.get('parser_worker', 'unknown')}")
    selftest_log(log, f"notify_pipeline ui_throttle_ms={notify.get('ui_throttle_ms', 'unknown')}")
    selftest_log(log, f"notify_pipeline per_packet_log={notify.get('per_packet_log', 'unknown')}")
    selftest_log(
        log,
        f"conn before_request ci_raw={conn.get('ci_raw', 'unknown')} "
        f"ci_raw_type={conn.get('ci_raw_type', 'unknown')} ci_ms={conn.get('ci_ms', 'unknown')} "
        f"latency={conn.get('latency', 'unknown')} timeout_ms={conn.get('timeout_ms', 'unknown')}",
    )
    selftest_log(log, f"throughput_request result={throughput.get('result', 'not_run')}")
    selftest_log(log, f"request_accepted={capability.get('request_accepted', 'unknown')}")
    selftest_log(log, f"effective_changed={capability.get('effective_changed', 'unknown')}")
    selftest_log(log, f"target_reached={capability.get('target_reached', 'unknown')}")
    selftest_log(log, f"runtime_target_reached={capability.get('target_reached', 'unknown')}")
    selftest_log(log, f"ci_min_observed_ms={capability.get('ci_min_observed_ms', 'unknown')}")
    selftest_log(log, f"ci_exact_control={1 if capability.get('ci_exact_control') else 0}")
    selftest_log(log, f"ci_control_method={capability.get('ci_control_method', 'unknown')}")
    selftest_log(log, f"capability_level={capability.get('capability_level', 'unknown')}")
    for recommendation in report.get("recommendation", []):
        selftest_log(log, f"recommendation={recommendation}")


def _recommendations(capability: dict[str, Any]) -> list[str]:
    level = capability.get("capability_level")
    if level == CAPABILITY_LEVEL_APP_PIPELINE:
        return ["optimize_notification_pipeline"]
    if level == CAPABILITY_LEVEL_API_UNAVAILABLE:
        return ["upgrade_windows_or_use_supported_adapter"]
    if level == CAPABILITY_LEVEL_RUNTIME_60MS_NOT_REACHED:
        return [
            "verify_firmware_runtime_conn_param_update",
            "check_windows_central_connection_parameter_policy",
        ]
    if capability.get("mtu_effective") not in {"247", 247, "unknown"}:
        return ["check_device_or_windows_mtu"]
    return []


def _format_text_report(report: dict[str, Any]) -> str:
    lines: list[str] = []
    _append_text_lines(lines, "", report)
    return "\n".join(lines) + "\n"


def _append_text_lines(lines: list[str], prefix: str, value: Any) -> None:
    if isinstance(value, dict):
        for key in sorted(value):
            _append_text_lines(lines, f"{prefix}{key}.", value[key])
        return
    if isinstance(value, list):
        for idx, item in enumerate(value):
            _append_text_lines(lines, f"{prefix}{idx}.", item)
        return
    lines.append(f"{prefix[:-1]}={value}")


def _write_json(path: Path, report: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)


def _write_text(path: Path, text: str) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write(text)


def _ready_task(value: Any) -> asyncio.Task[Any]:
    async def _return() -> Any:
        return value

    return asyncio.create_task(_return())


def _unknown_adapter() -> dict[str, Any]:
    return {
        "name": "unknown",
        "hwid": "unknown",
        "manufacturer": "unknown",
        "driver_provider": "unknown",
        "driver_version": "unknown",
        "driver_date": "unknown",
        "bus": "unknown",
    }


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _bus_from_hwid(hwid: str) -> str:
    text = hwid.upper()
    if text.startswith("USB\\") or "\\VID_" in text:
        return "usb"
    if text.startswith(("PCI\\", "ACPI\\", "BTH\\")):
        return "internal"
    return "unknown"


def _get_winrt_requester(client: Any) -> Any | None:
    backend = getattr(client, "_backend", None)
    if backend is None:
        return None
    return getattr(backend, "_requester", None)


def _backend_name(client: Any) -> str:
    backend_id = getattr(client, "backend_id", "")
    if backend_id:
        return str(backend_id).lower()
    backend = getattr(client, "_backend", None)
    if backend is None:
        return "unknown"
    module = getattr(type(backend), "__module__", "")
    if "winrt" in module.lower():
        return "winrt"
    return module or type(backend).__name__


def _api_status(obj: Any, attr_name: str) -> str:
    return "available" if obj is not None and hasattr(obj, attr_name) else "missing"


def _phy_name(phy: Any) -> str:
    if getattr(phy, "is_uncoded2_m_phy", False):
        return "2M"
    if getattr(phy, "is_uncoded1_m_phy", False):
        return "1M"
    if getattr(phy, "is_coded_phy", False):
        return "coded"
    return "unknown"


def _ci_ms_or_none(value: Any) -> float | None:
    if not isinstance(value, dict):
        return None
    raw = value.get("ci_ms")
    if raw in {None, "unknown"}:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _effective_changed(before: float | None, after: float | None) -> str:
    if before is None or after is None:
        return "unknown"
    return "1" if abs(before - after) > 0.25 else "0"


def _target_reached(ci_ms: float | None, target_ms: float) -> str:
    if ci_ms is None:
        return "unknown"
    return "1" if ci_ms <= target_ms + 0.25 else "0"


def _tri_state(value: Any) -> str:
    if value is True:
        return "1"
    if value is False:
        return "0"
    return "unknown"


def _capability_level(
    *,
    api_available: bool,
    request_result: str,
    callback_avg_us: float | None,
    queue_overflow: int,
    target_reached: str,
) -> str:
    if queue_overflow > 0 or (callback_avg_us is not None and callback_avg_us > 1000.0):
        return CAPABILITY_LEVEL_APP_PIPELINE
    if target_reached == "1":
        return CAPABILITY_LEVEL_RUNTIME_60MS
    if target_reached == "0":
        return CAPABILITY_LEVEL_RUNTIME_60MS_NOT_REACHED
    if not api_available:
        return CAPABILITY_LEVEL_API_UNAVAILABLE
    if request_result in {"unsupported", "fail"}:
        return CAPABILITY_LEVEL_API_UNAVAILABLE
    return CAPABILITY_LEVEL_UNKNOWN


def _mtu_from_report(report: dict[str, Any]) -> Any:
    gatt = report.get("gatt", {})
    if "mtu_effective" in gatt:
        return gatt["mtu_effective"]
    return "unknown"


def _safe_int_or_unknown(value: Any) -> int | str:
    try:
        return int(value)
    except (TypeError, ValueError):
        return "unknown"


def _safe_int_or_zero(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _safe_float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _redact(value: Any, *, key: str) -> Any:
    key_lower = key.lower()
    if isinstance(value, dict):
        return {item_key: _redact(item_value, key=str(item_key)) for item_key, item_value in value.items()}
    if isinstance(value, list):
        return [_redact(item, key=key) for item in value]
    if any(token in key_lower for token in ("address", "addr", "hwid", "hardware", "instanceid", "deviceid")):
        return "<redacted>"
    if key_lower in {"executable", "path", "json_path", "txt_path"} or key_lower.endswith("_path"):
        return "<redacted>"
    return value
