from __future__ import annotations

import csv
import json
import os
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .offline_v2_protocol import (
    OFFLINE_EVENT_RECORD_BYTES,
    OfflineEventRecord,
    OfflineSessionEnd,
    OfflineSessionListEntry,
    OfflineSessionManifest,
    crc32_ieee,
    parse_event_record,
)


OFFLINE_V2_DATA_NAME = "events_v2.bin"
OFFLINE_V2_PART_NAME = "events_v2.bin.part"
OFFLINE_V2_METADATA_NAME = "metadata.json"
OFFLINE_V2_RESUME_NAME = "resume.json"
OFFLINE_V2_CSV_NAME = "events_q12.csv"
OFFLINE_V2_DISCARDED_DIR = "discarded"
OFFLINE_V2_RESTART_NAME = "restart_after_capture.json"


@dataclass(frozen=True, slots=True)
class OfflineV2PrepareResult:
    session_key: str
    session_dir: Path
    durable_offset: int
    prefix_crc32: int
    completed: bool


@dataclass(frozen=True, slots=True)
class OfflineV2AppendResult:
    received_offset: int
    prefix_crc32: int
    duplicate: bool


@dataclass(frozen=True, slots=True)
class OfflineV2FinalizeResult:
    session_key: str
    session_dir: Path
    metadata: dict[str, Any]
    duplicate: bool


class OfflineV2SessionStore:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or default_offline_v2_root()
        self._session_key: str | None = None
        self._session_dir: Path | None = None
        self._manifest: OfflineSessionManifest | None = None
        self._list_entry: OfflineSessionListEntry | None = None
        self._device_name = ""
        self._device_address = ""
        self._received_offset = 0
        self._received_crc32 = 0
        self._durable_offset = 0
        self._durable_crc32 = 0

    @property
    def active_session_key(self) -> str | None:
        return self._session_key

    @property
    def received_offset(self) -> int:
        return self._received_offset

    @property
    def durable_offset(self) -> int:
        return self._durable_offset

    @property
    def durable_crc32(self) -> int:
        return self._durable_crc32

    def validate_active_checkpoint(self) -> tuple[int, int]:
        """Re-read the durable prefix before using it for a device resume."""
        manifest, session_dir = self._require_active()
        durable_offset = self._durable_offset
        if durable_offset < 0 or durable_offset > manifest.logical_bytes:
            raise ValueError("OFFLINE V2 durable offset is outside the manifest")
        part_path = session_dir / OFFLINE_V2_PART_NAME
        if not part_path.exists() or part_path.stat().st_size < durable_offset:
            raise ValueError("OFFLINE V2 partial file is shorter than the durable offset")
        actual_crc = _crc32_file_prefix(part_path, durable_offset)
        if actual_crc != self._durable_crc32:
            raise ValueError(
                "OFFLINE V2 durable prefix CRC mismatch: "
                f"0x{actual_crc:08X} != 0x{self._durable_crc32:08X}"
            )
        return durable_offset, actual_crc

    def mark_capture_preempted(
        self, entry: OfflineSessionListEntry, *, device_address: str
    ) -> None:
        """A separate commit survives finalize, disconnect and Host restart."""
        key = make_offline_v2_session_key(
            entry.session_id, entry.generation, entry.stream_crc32
        )
        directory = self.root / key
        directory.mkdir(parents=True, exist_ok=True)
        _atomic_write_json(directory / OFFLINE_V2_RESTART_NAME, {
            "session_id": entry.session_id, "generation": entry.generation,
            "stream_crc32": entry.stream_crc32, "device_address": device_address,
        })

    def prepare(
        self,
        manifest: OfflineSessionManifest,
        *,
        list_entry: OfflineSessionListEntry | None = None,
        device_name: str = "",
        device_address: str = "",
    ) -> OfflineV2PrepareResult:
        session_key = make_offline_v2_session_key(
            manifest.session_id, manifest.generation, manifest.stream_crc32
        )
        session_dir = self.root / session_key
        session_dir.mkdir(parents=True, exist_ok=True)
        metadata_path = session_dir / OFFLINE_V2_METADATA_NAME
        final_path = session_dir / OFFLINE_V2_DATA_NAME
        restart_path = session_dir / OFFLINE_V2_RESTART_NAME
        restart = _read_json(restart_path) if restart_path.exists() else None
        force_zero = bool(restart and
                          restart.get("device_address", "").casefold() == device_address.casefold())
        if not force_zero and metadata_path.exists() and final_path.exists():
            metadata = _read_json(metadata_path)
            if _metadata_matches_manifest(metadata, manifest):
                _validate_completed_data(final_path, manifest)
                self._clear_active()
                return OfflineV2PrepareResult(
                    session_key, session_dir, manifest.logical_bytes, manifest.stream_crc32, True
                )

        part_path = session_dir / OFFLINE_V2_PART_NAME
        resume_path = session_dir / OFFLINE_V2_RESUME_NAME
        if force_zero:
            # Retain an already committed record; replace it atomically only after
            # this fresh full transfer verifies. Never join S1 bytes to S2.
            _atomic_write_bytes(part_path, b"")
        if not force_zero and not part_path.exists() and final_path.exists():
            # metadata.json is the completion commit. A final data file without it
            # is an interrupted finalize and remains safe to resume from.
            os.replace(final_path, part_path)

        durable_offset = 0
        durable_crc = 0
        if not force_zero and resume_path.exists() and part_path.exists():
            resume = _read_json(resume_path)
            if _resume_matches_manifest(resume, manifest):
                durable_offset = int(resume.get("durable_offset") or 0)
                durable_crc = int(resume.get("prefix_crc32") or 0) & 0xFFFFFFFF
        if durable_offset < 0 or durable_offset > manifest.logical_bytes:
            durable_offset = 0
            durable_crc = 0

        if part_path.exists():
            size = part_path.stat().st_size
            if size < durable_offset:
                durable_offset = 0
                durable_crc = 0
            elif size > durable_offset:
                with part_path.open("r+b") as stream:
                    stream.truncate(durable_offset)
                    stream.flush()
                    os.fsync(stream.fileno())
        else:
            _atomic_write_bytes(part_path, b"")

        actual_crc = _crc32_file_prefix(part_path, durable_offset)
        if actual_crc != durable_crc:
            durable_offset = 0
            durable_crc = 0
            with part_path.open("r+b") as stream:
                stream.truncate(0)
                stream.flush()
                os.fsync(stream.fileno())

        self._session_key = session_key
        self._session_dir = session_dir
        self._manifest = manifest
        self._list_entry = list_entry
        self._device_name = device_name
        self._device_address = device_address
        self._received_offset = durable_offset
        self._received_crc32 = durable_crc
        self._durable_offset = durable_offset
        self._durable_crc32 = durable_crc
        self._write_resume(status="partial")
        if force_zero:
            restart_path.unlink()  # Zero checkpoint committed before consuming intent.
        return OfflineV2PrepareResult(
            session_key, session_dir, durable_offset, durable_crc, False
        )

    def append(self, *, offset: int, data: bytes, expected_prefix_crc32: int) -> OfflineV2AppendResult:
        manifest, session_dir = self._require_active()
        payload = bytes(data)
        next_offset = offset + len(payload)
        if not payload or next_offset > manifest.logical_bytes:
            raise ValueError("OFFLINE V2 append exceeds manifest")
        part_path = session_dir / OFFLINE_V2_PART_NAME

        if offset < self._received_offset:
            if next_offset > self._received_offset:
                raise ValueError("OFFLINE V2 overlapping retransmission is not supported")
            with part_path.open("rb") as stream:
                stream.seek(offset)
                existing = stream.read(len(payload))
            if existing != payload:
                raise ValueError("OFFLINE V2 retransmitted bytes differ from persisted data")
            prefix = _crc32_file_prefix(part_path, next_offset)
            if prefix != (expected_prefix_crc32 & 0xFFFFFFFF):
                raise ValueError("OFFLINE V2 retransmitted prefix CRC mismatch")
            return OfflineV2AppendResult(self._received_offset, self._received_crc32, True)

        if offset != self._received_offset:
            raise ValueError(
                f"OFFLINE V2 data gap: offset={offset} expected={self._received_offset}"
            )
        next_crc = crc32_ieee(payload, self._received_crc32)
        if next_crc != (expected_prefix_crc32 & 0xFFFFFFFF):
            raise ValueError(
                f"OFFLINE V2 prefix CRC mismatch: 0x{expected_prefix_crc32:08X} != 0x{next_crc:08X}"
            )
        with part_path.open("ab") as stream:
            stream.write(payload)
            stream.flush()
        self._received_offset = next_offset
        self._received_crc32 = next_crc
        return OfflineV2AppendResult(next_offset, next_crc, False)

    def checkpoint(self) -> tuple[int, int]:
        _, session_dir = self._require_active()
        part_path = session_dir / OFFLINE_V2_PART_NAME
        with part_path.open("r+b") as stream:
            stream.flush()
            os.fsync(stream.fileno())
        self._durable_offset = self._received_offset
        self._durable_crc32 = self._received_crc32
        self._write_resume(status="partial")
        return self._durable_offset, self._durable_crc32

    def finalize(self, end: OfflineSessionEnd) -> OfflineV2FinalizeResult:
        manifest, session_dir = self._require_active()
        if (
            end.session_id != manifest.session_id
            or end.transfer_id != manifest.transfer_id
            or end.generation != manifest.generation
            or end.logical_bytes != manifest.logical_bytes
            or end.event_count != manifest.event_count
            or end.stream_crc32 != manifest.stream_crc32
            or end.manifest_crc32 != manifest.manifest_crc32
        ):
            raise ValueError("OFFLINE V2 END does not match BEGIN manifest")
        self.checkpoint()
        if self._durable_offset != manifest.logical_bytes:
            raise ValueError(
                f"OFFLINE V2 finalize length mismatch: {self._durable_offset} != {manifest.logical_bytes}"
            )
        if self._durable_crc32 != manifest.stream_crc32:
            raise ValueError(
                f"OFFLINE V2 stream CRC mismatch: 0x{self._durable_crc32:08X} != 0x{manifest.stream_crc32:08X}"
            )

        part_path = session_dir / OFFLINE_V2_PART_NAME
        records = _validate_all_records(part_path, manifest)
        csv_tmp = session_dir / f"{OFFLINE_V2_CSV_NAME}.tmp"
        metadata_tmp = session_dir / f"{OFFLINE_V2_METADATA_NAME}.tmp"
        final_path = session_dir / OFFLINE_V2_DATA_NAME
        csv_path = session_dir / OFFLINE_V2_CSV_NAME
        metadata_path = session_dir / OFFLINE_V2_METADATA_NAME
        _write_csv(csv_tmp, records)
        metadata = self._build_completed_metadata(end)
        _atomic_write_json(metadata_tmp, metadata)

        os.replace(part_path, final_path)
        os.replace(csv_tmp, csv_path)
        os.replace(metadata_tmp, metadata_path)
        _validate_completed_data(final_path, manifest)
        _read_json(metadata_path)
        resume_path = session_dir / OFFLINE_V2_RESUME_NAME
        if resume_path.exists():
            resume_path.unlink()
        result = OfflineV2FinalizeResult(
            session_key=self._session_key or session_dir.name,
            session_dir=session_dir,
            metadata=metadata,
            duplicate=False,
        )
        self._clear_active()
        return result

    def list_sessions(self) -> list[dict[str, Any]]:
        if not self.root.exists():
            return []
        result: list[dict[str, Any]] = []
        for session_dir in self.root.iterdir():
            if not session_dir.is_dir():
                continue
            metadata_path = session_dir / OFFLINE_V2_METADATA_NAME
            if not metadata_path.exists():
                continue
            try:
                metadata = _read_json(metadata_path)
                metadata["can_download"] = self._can_download(session_dir, metadata)
            except (OSError, ValueError) as exc:
                metadata = {
                    "session_key": session_dir.name,
                    "transfer_status": "corrupt",
                    "corrupt_reason": str(exc),
                    "can_download": False,
                }
            metadata["session_dir"] = str(session_dir)
            result.append(metadata)
        result.sort(
            key=lambda item: (
                int(item.get("start_unix_ms") or 0),
                int(item.get("generation") or 0),
                int(item.get("session_id") or 0),
            ),
            reverse=True,
        )
        return result

    def update_reclaim_status(
        self,
        session_id: int,
        generation: int,
        *,
        state: str,
        erased_bytes: int = 0,
        extent_bytes: int = 0,
    ) -> dict[str, Any] | None:
        session_dir = self._find_completed_session(session_id, generation)
        if session_dir is None:
            return None
        metadata_path = session_dir / OFFLINE_V2_METADATA_NAME
        metadata = _read_json(metadata_path)
        metadata.update(
            {
                "reclaim_status": state,
                "reclaim_erased_bytes": erased_bytes,
                "reclaim_extent_bytes": extent_bytes,
                "updated_at": now_iso(),
            }
        )
        _atomic_write_json(metadata_path, metadata)
        return metadata

    def write_zip(self, session_key: str, destination: str | Path) -> Path:
        session_dir = self._session_dir_for_key(session_key)
        metadata = _read_json(session_dir / OFFLINE_V2_METADATA_NAME)
        if not self._can_download(session_dir, metadata):
            raise ValueError("OFFLINE V2 session is not complete enough to download")
        destination_path = Path(destination)
        with zipfile.ZipFile(destination_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for name in (OFFLINE_V2_DATA_NAME, OFFLINE_V2_METADATA_NAME, OFFLINE_V2_CSV_NAME):
                archive.write(session_dir / name, arcname=name)
        return destination_path

    def delete_session(self, session_key: str) -> None:
        session_dir = self._session_dir_for_key(session_key)
        if session_dir.parent.resolve() != self.root.resolve():
            raise ValueError("OFFLINE V2 session path escaped storage root")
        allowed_names = {
            OFFLINE_V2_DATA_NAME,
            OFFLINE_V2_PART_NAME,
            OFFLINE_V2_METADATA_NAME,
            OFFLINE_V2_RESUME_NAME,
            OFFLINE_V2_RESTART_NAME,
            OFFLINE_V2_CSV_NAME,
            f"{OFFLINE_V2_METADATA_NAME}.tmp",
            f"{OFFLINE_V2_CSV_NAME}.tmp",
        }
        children = list(session_dir.iterdir())
        for child in children:
            if child.name not in allowed_names or child.is_symlink() or not child.is_file():
                raise ValueError(f"OFFLINE V2 session contains an unexpected entry: {child.name}")
        for child in children:
            child.unlink()
        session_dir.rmdir()

    def close_active(self) -> None:
        self._clear_active()

    def discard_active_for_device_clear(self) -> Path | None:
        if self._session_dir is None or self._session_key is None:
            return None
        session_dir = self._session_dir
        if session_dir.parent.resolve() != self.root.resolve():
            raise ValueError("OFFLINE V2 active session escaped storage root")
        self._write_resume(status="discarded_by_device_clear")
        discarded_root = self.root / OFFLINE_V2_DISCARDED_DIR
        discarded_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        destination = discarded_root / f"{self._session_key}_clear_{stamp}"
        os.replace(session_dir, destination)
        self._clear_active()
        return destination

    def update_device_clear_status(
        self,
        session_key: str,
        *,
        status: str,
        verified: bool,
        reason: str = "",
    ) -> dict[str, Any]:
        session_dir = self._session_dir_for_key(session_key)
        metadata_path = session_dir / OFFLINE_V2_METADATA_NAME
        metadata = _read_json(metadata_path)
        metadata.update(
            {
                "device_clear_status": status,
                "device_clear_verified": bool(verified),
                "device_clear_reason": reason,
                "device_clear_updated_at": now_iso(),
                "updated_at": now_iso(),
            }
        )
        _atomic_write_json(metadata_path, metadata)
        return metadata

    def _write_resume(self, *, status: str) -> None:
        manifest, session_dir = self._require_active()
        resume = {
            "protocol": "offline_v2",
            "status": status,
            "session_key": self._session_key,
            "session_id": manifest.session_id,
            "generation": manifest.generation,
            "stream_crc32": manifest.stream_crc32,
            "logical_bytes": manifest.logical_bytes,
            "event_count": manifest.event_count,
            "owner_user_id": manifest.owner_user_id,
            "owner_user_id_known": manifest.owner_user_id is not None,
            "durable_offset": self._durable_offset,
            "prefix_crc32": self._durable_crc32,
            "updated_at": now_iso(),
        }
        _atomic_write_json(session_dir / OFFLINE_V2_RESUME_NAME, resume)

    def _build_completed_metadata(self, end: OfflineSessionEnd) -> dict[str, Any]:
        manifest, _ = self._require_active()
        entry = asdict(self._list_entry) if self._list_entry is not None else None
        return {
            "protocol": "offline_v2",
            "format_version": manifest.format_version,
            "session_key": self._session_key,
            "session_id": manifest.session_id,
            "generation": manifest.generation,
            "owner_user_id": manifest.owner_user_id,
            "owner_user_id_known": manifest.owner_user_id is not None,
            "start_unix_ms": manifest.start_unix_ms,
            "duration_ms": manifest.duration_ms,
            "event_count": manifest.event_count,
            "logical_bytes": manifest.logical_bytes,
            "total_bytes": manifest.logical_bytes,
            "stream_crc32": manifest.stream_crc32,
            "sample_rate_hz": manifest.sample_rate_hz,
            "feature_dimensions": manifest.feature_dimensions,
            "q_frac_bits": manifest.q_frac_bits,
            "event_record_bytes": manifest.event_record_bytes,
            "event_slot_bytes": manifest.event_slot_bytes,
            "manifest": asdict(manifest),
            "session_health": manifest.health_name,
            "session_attention_required": manifest.health != 0,
            "stop_reason": manifest.stop_reason,
            "clean": manifest.clean,
            "session_list_entry": entry,
            "session_end": asdict(end),
            "device_name": self._device_name,
            "device_address": self._device_address,
            "transfer_status": "complete",
            "reclaim_status": "confirm_pending",
            "created_at": now_iso(),
            "updated_at": now_iso(),
        }

    def _can_download(self, session_dir: Path, metadata: dict[str, Any]) -> bool:
        if metadata.get("protocol") != "offline_v2" or metadata.get("transfer_status") != "complete":
            return False
        try:
            manifest = _manifest_from_metadata(metadata)
            _validate_completed_data(session_dir / OFFLINE_V2_DATA_NAME, manifest)
        except (OSError, TypeError, ValueError):
            return False
        return (session_dir / OFFLINE_V2_CSV_NAME).is_file()

    def _find_completed_session(self, session_id: int, generation: int) -> Path | None:
        if not self.root.exists():
            return None
        prefix = f"offline_v2_{session_id & 0xFFFFFFFF:08X}_{generation & 0xFFFFFFFF:08X}_"
        for child in self.root.iterdir():
            if child.is_dir() and child.name.startswith(prefix) and (child / OFFLINE_V2_METADATA_NAME).exists():
                return child
        return None

    def _session_dir_for_key(self, session_key: str) -> Path:
        if not session_key or Path(session_key).name != session_key:
            raise ValueError("invalid OFFLINE V2 session key")
        session_dir = self.root / session_key
        if not session_dir.is_dir():
            raise FileNotFoundError(f"OFFLINE V2 session not found: {session_key}")
        return session_dir

    def _require_active(self) -> tuple[OfflineSessionManifest, Path]:
        if self._manifest is None or self._session_dir is None:
            raise RuntimeError("no active OFFLINE V2 session")
        return self._manifest, self._session_dir

    def _clear_active(self) -> None:
        self._session_key = None
        self._session_dir = None
        self._manifest = None
        self._list_entry = None
        self._device_name = ""
        self._device_address = ""
        self._received_offset = 0
        self._received_crc32 = 0
        self._durable_offset = 0
        self._durable_crc32 = 0


def default_offline_v2_root() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        return Path(local_app_data) / "ZY100_BLE_Tool" / "offline_v2_sessions"
    return Path(__file__).resolve().parents[1] / ".zy100_offline_v2_sessions"


def make_offline_v2_session_key(session_id: int, generation: int, stream_crc32: int) -> str:
    return (
        f"offline_v2_{session_id & 0xFFFFFFFF:08X}_"
        f"{generation & 0xFFFFFFFF:08X}_{stream_crc32 & 0xFFFFFFFF:08X}"
    )


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f"{path.name}.tmp")
    with temp_path.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def _atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path if path.name.endswith(".tmp") else path.with_name(f"{path.name}.tmp")
    with temp_path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    if temp_path != path:
        os.replace(temp_path, path)


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return data


def _crc32_file_prefix(path: Path, length: int) -> int:
    if length == 0:
        return 0
    remaining = length
    value = 0
    with path.open("rb") as stream:
        while remaining:
            block = stream.read(min(64 * 1024, remaining))
            if not block:
                raise ValueError("OFFLINE V2 partial file shorter than expected")
            value = crc32_ieee(block, value)
            remaining -= len(block)
    return value


def _validate_all_records(path: Path, manifest: OfflineSessionManifest) -> list[OfflineEventRecord]:
    records: list[OfflineEventRecord] = []
    if manifest.event_record_bytes != OFFLINE_EVENT_RECORD_BYTES:
        raise ValueError(
            "OFFLINE V2 stored event size is unsupported: "
            f"{manifest.event_record_bytes}"
        )
    with path.open("rb") as stream:
        for index in range(manifest.event_count):
            raw = stream.read(manifest.event_record_bytes)
            if len(raw) != manifest.event_record_bytes:
                raise ValueError(f"OFFLINE V2 event {index} is truncated")
            records.append(
                parse_event_record(
                    raw,
                    expected_session_id=manifest.session_id,
                    expected_event_index=index,
                    expected_config_crc32=manifest.config_crc32,
                )
            )
        if stream.read(1):
            raise ValueError("OFFLINE V2 data contains trailing bytes")
    return records


def _write_csv(path: Path, records: list[OfflineEventRecord]) -> None:
    headers = [
        "event_index",
        "center_sample_index",
        "center_time_seconds",
        "peak_mag2_raw",
        "flags",
        "shock_available",
        "valid_hit",
        "accel_clipped",
        "gyro_clipped",
        "q12_saturated",
        "shock_numerator",
        "shock_baseline",
        "shock_ratio_rounded_4dp",
        "detector_version",
        "shock_version",
        "feature_version",
        "codec_version",
        "config_crc32",
        "record_crc32",
    ] + [f"feature_q12_{index:03d}" for index in range(128)]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for record in records:
            writer.writerow(
                [
                    record.event_index,
                    record.center_sample_index,
                    f"{record.center_time_seconds:.6f}",
                    record.peak_mag2_raw,
                    record.flags,
                    int(record.shock_available),
                    int(record.valid_hit),
                    int(record.accel_clipped),
                    int(record.gyro_clipped),
                    int(record.q12_saturated),
                    f"{record.shock_numerator:.9g}",
                    f"{record.shock_baseline:.9g}",
                    f"{record.shock_ratio_rounded_4dp:.4f}",
                    record.detector_version,
                    record.shock_version,
                    record.feature_version,
                    record.codec_version,
                    f"0x{record.config_crc32:08X}",
                    f"0x{record.record_crc32:08X}",
                    *record.feature_q12,
                ]
            )
        stream.flush()
        os.fsync(stream.fileno())


def _metadata_matches_manifest(metadata: dict[str, Any], manifest: OfflineSessionManifest) -> bool:
    return (
        metadata.get("protocol") == "offline_v2"
        and int(metadata.get("session_id") or 0) == manifest.session_id
        and int(metadata.get("generation") or 0) == manifest.generation
        and int(metadata.get("logical_bytes") or 0) == manifest.logical_bytes
        and int(metadata.get("stream_crc32") or 0) == manifest.stream_crc32
        and metadata.get("owner_user_id") == manifest.owner_user_id
        and metadata.get("transfer_status") == "complete"
    )


def _resume_matches_manifest(resume: dict[str, Any], manifest: OfflineSessionManifest) -> bool:
    return (
        resume.get("protocol") == "offline_v2"
        and int(resume.get("session_id") or 0) == manifest.session_id
        and int(resume.get("generation") or 0) == manifest.generation
        and int(resume.get("logical_bytes") or 0) == manifest.logical_bytes
        and int(resume.get("stream_crc32") or 0) == manifest.stream_crc32
        and resume.get("owner_user_id") == manifest.owner_user_id
    )


def _manifest_from_metadata(metadata: dict[str, Any]) -> OfflineSessionManifest:
    raw = metadata.get("manifest")
    if not isinstance(raw, dict):
        raise ValueError("OFFLINE V2 metadata manifest missing")
    values = dict(raw)
    contract = values.get("contract_crc32")
    if not isinstance(contract, (list, tuple)) or len(contract) != 5:
        raise ValueError("OFFLINE V2 metadata contract CRC list invalid")
    values["contract_crc32"] = tuple(int(value) for value in contract)
    recovered = bool(int(values.get("flags") or 0) & 0x02)
    values.setdefault("stop_reason", 3 if recovered else 0)
    values.setdefault("clean", not recovered)
    values.setdefault("health", 3 if recovered else 0)
    values.setdefault("fifo_overflow_count", 0)
    values.setdefault("fifo_discard_count", 0)
    values.setdefault("time_gap_count", 0)
    values.setdefault("feature_drop_count", 0)
    values.setdefault("q12_clip_count", 0)
    values.setdefault("flash_error_count", 0)
    values.setdefault("owner_user_id", None)
    return OfflineSessionManifest(**values)


def _validate_completed_data(path: Path, manifest: OfflineSessionManifest) -> None:
    if not path.is_file() or path.stat().st_size != manifest.logical_bytes:
        raise ValueError("OFFLINE V2 completed data length mismatch")
    if _crc32_file_prefix(path, manifest.logical_bytes) != manifest.stream_crc32:
        raise ValueError("OFFLINE V2 completed data stream CRC mismatch")
    _validate_all_records(path, manifest)
