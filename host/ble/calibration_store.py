from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path

from .calibration_protocol import (
    CalibrationRecord,
    MagCalibrationDiagnostics,
    parse_calibration_record,
    parse_mag_calibration_diagnostics,
)


@dataclass(frozen=True, slots=True)
class CalibrationSaveResult:
    device_dir: Path
    binary_path: Path
    metadata_path: Path
    metadata: dict[str, object]


@dataclass(frozen=True, slots=True)
class DiagnosticsSaveResult:
    device_dir: Path
    binary_path: Path
    metadata_path: Path
    metadata: dict[str, object]


class CalibrationStore:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or default_calibration_root()

    def save_verified_record(
        self,
        record: CalibrationRecord,
        *,
        device_name: str = "",
        device_address: str = "",
        device_serial: str = "",
    ) -> CalibrationSaveResult:
        parsed = parse_calibration_record(record.raw)
        if parsed.crc32 != record.crc32 or parsed.generation != record.generation:
            raise ValueError("Calibration record changed before persistence")

        device_key = _safe_name(
            device_serial or device_address or device_name or "unknown_device"
        )
        device_dir = self.root / device_key
        device_dir.mkdir(parents=True, exist_ok=True)
        stem = f"calibration_g{record.generation:010d}_{record.crc32:08X}"
        binary_path = device_dir / f"{stem}.bin"
        metadata_path = device_dir / f"{stem}.json"
        metadata = {
            **record.to_dict(),
            "device_name": device_name,
            "device_address": device_address,
            "device_serial": device_serial,
            "received_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            "binary_file": binary_path.name,
        }

        _atomic_write_bytes(binary_path, record.raw)
        parse_calibration_record(binary_path.read_bytes())
        _atomic_write_json(metadata_path, metadata)
        _atomic_write_bytes(device_dir / "latest.bin", record.raw)
        _atomic_write_json(device_dir / "latest.json", metadata)
        return CalibrationSaveResult(device_dir, binary_path, metadata_path, metadata)

    def find_verified_record(
        self,
        *,
        generation: int,
        crc32: int,
        record_bytes: int,
        valid_flags: int | None = None,
        device_name: str = "",
        device_address: str = "",
        device_serial: str = "",
    ) -> CalibrationRecord | None:
        keys = []
        for value in (device_serial, device_address, device_name, "unknown_device"):
            key = _safe_name(value) if value else ""
            if key and key not in keys:
                keys.append(key)
        filename = (
            f"calibration_g{int(generation):010d}_{int(crc32) & 0xFFFFFFFF:08X}.bin"
        )
        for key in keys:
            candidate = self.root / key / filename
            if not candidate.is_file():
                continue
            try:
                record = parse_calibration_record(candidate.read_bytes())
            except (OSError, ValueError):
                continue
            if (
                record.generation != int(generation)
                or record.crc32 != (int(crc32) & 0xFFFFFFFF)
                or record.record_bytes != int(record_bytes)
                or (valid_flags is not None and record.valid_flags != int(valid_flags))
            ):
                continue
            if device_serial and key != _safe_name(device_serial):
                self.save_verified_record(
                    record,
                    device_name=device_name,
                    device_address=device_address,
                    device_serial=device_serial,
                )
            return record
        return None

    def save_verified_diagnostics(
        self,
        diagnostics: MagCalibrationDiagnostics,
        *,
        request_transaction_id: int,
        device_name: str = "",
        device_address: str = "",
        generation: int = 0,
        calibration_crc32: int = 0,
    ) -> DiagnosticsSaveResult:
        parsed = parse_mag_calibration_diagnostics(diagnostics.raw)
        if parsed.crc32 != diagnostics.crc32:
            raise ValueError("Mag calibration diagnostics changed before persistence")

        device_key = _safe_name(device_address or device_name or "unknown_device")
        device_dir = self.root / device_key
        device_dir.mkdir(parents=True, exist_ok=True)
        stem = (
            f"mag_diagnostics_t{diagnostics.transaction_id:03d}_"
            f"{diagnostics.crc32:08X}"
        )
        binary_path = device_dir / f"{stem}.bin"
        metadata_path = device_dir / f"{stem}.json"
        metadata = {
            **diagnostics.to_dict(),
            "request_transaction_id": request_transaction_id & 0xFF,
            "generation": int(generation),
            "calibration_crc32": f"0x{calibration_crc32 & 0xFFFFFFFF:08X}",
            "device_name": device_name,
            "device_address": device_address,
            "received_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            "binary_file": binary_path.name,
        }

        _atomic_write_bytes(binary_path, diagnostics.raw)
        parse_mag_calibration_diagnostics(binary_path.read_bytes())
        _atomic_write_json(metadata_path, metadata)
        _atomic_write_bytes(device_dir / "latest_diagnostics.bin", diagnostics.raw)
        _atomic_write_json(device_dir / "latest_diagnostics.json", metadata)
        return DiagnosticsSaveResult(
            device_dir, binary_path, metadata_path, metadata
        )

def default_calibration_root() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        return Path(local_app_data) / "ZY100_BLE_Tool" / "calibration"
    return Path(__file__).resolve().parents[1] / ".zy100_calibration"


def _safe_name(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return text.strip("._") or "unknown_device"


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("wb") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    data = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True).encode("utf-8")
    _atomic_write_bytes(path, data)
