from __future__ import annotations

import binascii
import json
import os
import re
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .zy100_protocol import FEUFParseResult, parse_feuf_stream


SESSION_FEUF_NAME = "session.feuf"
SESSION_METADATA_NAME = "metadata.json"


@dataclass(frozen=True, slots=True)
class SessionSaveResult:
    session_key: str
    session_dir: Path
    metadata: dict[str, Any]
    duplicate: bool


class SessionStore:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or default_session_root()

    def list_sessions(self) -> list[dict[str, Any]]:
        if not self.root.exists():
            return []

        sessions: list[dict[str, Any]] = []
        for session_dir in sorted(self.root.iterdir(), reverse=True):
            if not session_dir.is_dir():
                continue
            metadata_path = session_dir / SESSION_METADATA_NAME
            if metadata_path.exists():
                try:
                    metadata = self._read_json(metadata_path)
                except (OSError, ValueError):
                    metadata = self._corrupt_metadata(session_dir.name, "metadata unreadable")
            else:
                metadata = self._corrupt_metadata(session_dir.name, "metadata missing")

            metadata["session_key"] = metadata.get("session_key") or session_dir.name
            metadata["session_dir"] = str(session_dir)
            metadata["can_download"] = self._can_download(session_dir, metadata)
            sessions.append(metadata)

        sessions.sort(key=_session_sort_key, reverse=True)
        return sessions

    def save_or_reuse_completed_export(
        self,
        *,
        feuf_bytes: bytes,
        export_id: int,
        total_bytes: int,
        stream_crc32: int,
        chunk_count: int,
        parsed: FEUFParseResult,
        device_name: str = "",
        device_address: str = "",
        session_count: int = 1,
        estimated_total_bytes: int = 0,
    ) -> SessionSaveResult:
        _validate_export_id_matches_manifest(export_id, parsed)
        session_key = make_session_key(export_id, total_bytes, stream_crc32)
        existing = self._find_session_dir(session_key)
        if existing is not None:
            metadata = self._validate_existing_or_mark_corrupt(existing, total_bytes, stream_crc32)
            if metadata is not None:
                updates = {
                    "transfer_status": "duplicate",
                    "duplicate_count": int(metadata.get("duplicate_count") or 0) + 1,
                    "last_duplicate_at": now_iso(),
                }
                metadata = self.update_metadata(session_key, updates)
                return SessionSaveResult(session_key=session_key, session_dir=existing, metadata=metadata, duplicate=True)

        self.root.mkdir(parents=True, exist_ok=True)
        session_dir = self.root / _safe_dir_name(session_key)
        session_dir.mkdir(parents=True, exist_ok=True)
        metadata = build_session_metadata(
            session_key=session_key,
            export_id=export_id,
            total_bytes=total_bytes,
            stream_crc32=stream_crc32,
            chunk_count=chunk_count,
            parsed=parsed,
            device_name=device_name,
            device_address=device_address,
            session_count=session_count,
            estimated_total_bytes=estimated_total_bytes,
            transfer_status="complete",
        )

        feuf_tmp = session_dir / f"{SESSION_FEUF_NAME}.tmp"
        metadata_tmp = session_dir / f"{SESSION_METADATA_NAME}.tmp"
        feuf_path = session_dir / SESSION_FEUF_NAME
        metadata_path = session_dir / SESSION_METADATA_NAME

        self._atomic_write_bytes_tmp(feuf_tmp, feuf_bytes)
        read_back = feuf_tmp.read_bytes()
        _validate_feuf_bytes(read_back, total_bytes, stream_crc32)
        self._atomic_write_json_tmp(metadata_tmp, metadata)
        self._read_json(metadata_tmp)

        os.replace(feuf_tmp, feuf_path)
        os.replace(metadata_tmp, metadata_path)

        _validate_feuf_bytes(feuf_path.read_bytes(), total_bytes, stream_crc32)
        metadata = self._read_json(metadata_path)
        return SessionSaveResult(session_key=session_key, session_dir=session_dir, metadata=metadata, duplicate=False)

    def update_metadata(self, session_key: str, updates: dict[str, Any]) -> dict[str, Any]:
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"session not found: {session_key}")
        metadata_path = session_dir / SESSION_METADATA_NAME
        metadata = self._read_json(metadata_path) if metadata_path.exists() else {"session_key": session_key}
        metadata.update(updates)
        metadata["updated_at"] = now_iso()
        tmp_path = session_dir / f"{SESSION_METADATA_NAME}.tmp"
        self._atomic_write_json_tmp(tmp_path, metadata)
        self._read_json(tmp_path)
        os.replace(tmp_path, metadata_path)
        return metadata

    def mark_corrupt(self, session_key: str, reason: str) -> dict[str, Any]:
        session_dir = self._find_session_dir(session_key) or (self.root / _safe_dir_name(session_key))
        session_dir.mkdir(parents=True, exist_ok=True)
        metadata_path = session_dir / SESSION_METADATA_NAME
        if metadata_path.exists():
            try:
                metadata = self._read_json(metadata_path)
            except (OSError, ValueError):
                metadata = {"session_key": session_key}
        else:
            metadata = {"session_key": session_key}
        metadata.update(
            {
                "transfer_status": "corrupt",
                "corrupt_reason": reason,
                "updated_at": now_iso(),
            }
        )
        tmp_path = session_dir / f"{SESSION_METADATA_NAME}.tmp"
        self._atomic_write_json_tmp(tmp_path, metadata)
        os.replace(tmp_path, metadata_path)
        return metadata

    def delete_session(self, session_key: str) -> None:
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"session not found: {session_key}")
        for child in session_dir.iterdir():
            if child.is_file():
                child.unlink()
        session_dir.rmdir()

    def write_zip(self, session_key: str, destination: str | Path) -> Path:
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"session not found: {session_key}")
        metadata = self._read_json(session_dir / SESSION_METADATA_NAME)
        if not self._can_download(session_dir, metadata):
            raise ValueError("session is not complete enough to download")
        total_bytes = int(metadata.get("total_bytes") or 0)
        stream_crc32 = int(metadata.get("stream_crc32") or 0)
        _validate_feuf_bytes((session_dir / SESSION_FEUF_NAME).read_bytes(), total_bytes, stream_crc32)
        destination_path = Path(destination)
        with zipfile.ZipFile(destination_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.write(session_dir / SESSION_FEUF_NAME, arcname=SESSION_FEUF_NAME)
            zf.write(session_dir / SESSION_METADATA_NAME, arcname=SESSION_METADATA_NAME)
        return destination_path

    def _find_session_dir(self, session_key: str) -> Path | None:
        candidate = self.root / _safe_dir_name(session_key)
        if candidate.exists() and candidate.is_dir():
            return candidate
        if not self.root.exists():
            return None
        prefix = _safe_dir_name(session_key)
        for child in self.root.iterdir():
            if child.is_dir() and child.name.startswith(prefix):
                return child
        return None

    def _validate_existing_or_mark_corrupt(
        self,
        session_dir: Path,
        total_bytes: int,
        stream_crc32: int,
    ) -> dict[str, Any] | None:
        session_key = session_dir.name
        try:
            feuf_path = session_dir / SESSION_FEUF_NAME
            metadata_path = session_dir / SESSION_METADATA_NAME
            if not feuf_path.exists() or not metadata_path.exists():
                raise ValueError("formal session files missing")
            _validate_feuf_bytes(feuf_path.read_bytes(), total_bytes, stream_crc32)
            metadata = self._read_json(metadata_path)
            return metadata
        except (OSError, ValueError) as exc:
            self.mark_corrupt(session_key, str(exc))
            return None

    def _can_download(self, session_dir: Path, metadata: dict[str, Any]) -> bool:
        if metadata.get("transfer_status") == "corrupt":
            return False
        return (session_dir / SESSION_FEUF_NAME).is_file() and (session_dir / SESSION_METADATA_NAME).is_file()

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as f:
            loaded = json.load(f)
        if not isinstance(loaded, dict):
            raise ValueError("metadata is not an object")
        return loaded

    @staticmethod
    def _atomic_write_bytes_tmp(path: Path, data: bytes) -> None:
        with path.open("wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())

    @staticmethod
    def _atomic_write_json_tmp(path: Path, data: dict[str, Any]) -> None:
        encoded = json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True).encode("utf-8")
        with path.open("wb") as f:
            f.write(encoded)
            f.flush()
            os.fsync(f.fileno())

    @staticmethod
    def _corrupt_metadata(session_key: str, reason: str) -> dict[str, Any]:
        return {
            "session_key": session_key,
            "transfer_status": "corrupt",
            "corrupt_reason": reason,
            "created_at": "",
        }


def build_session_metadata(
    *,
    session_key: str,
    export_id: int,
    total_bytes: int,
    stream_crc32: int,
    chunk_count: int,
    parsed: FEUFParseResult,
    device_name: str,
    device_address: str,
    session_count: int,
    estimated_total_bytes: int,
    transfer_status: str,
) -> dict[str, Any]:
    manifest = parsed.manifest
    begin = parsed.training_begin
    end = parsed.training_end
    feuf_export_end = parsed.export_end
    is_manifest_v3 = bool(manifest and manifest.protocol_version == 3)
    session_uid = manifest.session_uid if is_manifest_v3 and manifest else 0
    session_id = session_uid or (manifest.round if manifest else (begin.capture_round if begin else export_id))
    user_id = (
        (manifest.user_id if is_manifest_v3 and manifest else None)
        or (begin.user_id if begin else None)
        or (end.user_id if end else 0)
    )
    training_id = (
        (manifest.training_id if is_manifest_v3 and manifest else None)
        or (begin.training_id if begin else None)
        or (end.training_id if end else 0)
    )
    start_time_ms = (
        (manifest.start_time_ms if is_manifest_v3 and manifest else None)
        or (begin.start_time_ms if begin else None)
        or (end.start_time_ms if end else 0)
    )
    end_time_ms = (
        (manifest.end_time_ms if is_manifest_v3 and manifest else None)
        or (end.end_time_ms if end else 0)
    )
    duration_ms = end_time_ms - start_time_ms if start_time_ms and end_time_ms and end_time_ms >= start_time_ms else 0

    return {
        "schema_version": 1,
        "session_key": session_key,
        "transfer_status": transfer_status,
        "saved_local": transfer_status in {"complete", "duplicate"},
        "confirm_sent": False,
        "confirm_ack_status": "",
        "reclaim_status": "",
        "reclaim_verified": False,
        "reclaim_started_at": "",
        "reclaim_completed_at": "",
        "reclaim_unknown_at": "",
        "reclaim_failed_at": "",
        "reclaim_failed_reason": "",
        "device_clear_status": "",
        "device_clear_verified": False,
        "duplicate_of": "",
        "last_confirm_at": "",
        "clear_wait_started_at": "",
        "created_at": now_iso(),
        "updated_at": now_iso(),
        "device_name": device_name,
        "device_address": device_address,
        "export_id": export_id,
        "session_count": session_count,
        "estimated_total_bytes": estimated_total_bytes,
        "total_bytes": total_bytes,
        "stream_crc32": stream_crc32,
        "chunk_count": chunk_count,
        "session_id": session_id,
        "session_uid": session_uid,
        "export_index": manifest.export_index if manifest else 0,
        "export_total": manifest.export_total if manifest else session_count,
        "user_id": user_id,
        "training_id": training_id,
        "session_seq": (manifest.session_seq if is_manifest_v3 and manifest else (begin.session_seq if begin else 0)),
        "capture_round": (manifest.round if manifest else (begin.capture_round if begin else session_id)),
        "start_time_ms": start_time_ms,
        "end_time_ms": end_time_ms,
        "duration_ms": duration_ms,
        "source": begin.source if begin else 0,
        "stop_reason": end.stop_reason if end else (manifest.stop_reason if manifest else 0),
        "raw_count": manifest.raw_count if manifest else (end.raw_block_count if end else 0),
        "summary_count": manifest.summary_count if manifest else (end.summary_count if end else 0),
        "event_count": manifest.event_count if manifest else (end.event_count if end else 0),
        "raw_data_begin_addr": manifest.raw_data_begin_addr if manifest else 0,
        "raw_data_end_addr": manifest.raw_data_end_addr if manifest else 0,
        "raw_reclaim_end_addr": manifest.raw_reclaim_end_addr if manifest else 0,
        "summary_data_begin_addr": manifest.summary_data_begin_addr if manifest else 0,
        "summary_data_end_addr": manifest.summary_data_end_addr if manifest else 0,
        "summary_reclaim_end_addr": manifest.summary_reclaim_end_addr if manifest else 0,
        "event_data_begin_addr": manifest.event_data_begin_addr if manifest else 0,
        "event_data_end_addr": manifest.event_data_end_addr if manifest else 0,
        "event_reclaim_end_addr": manifest.event_reclaim_end_addr if manifest else 0,
        "raw_bytes": manifest.raw_bytes if manifest else 0,
        "summary_bytes": manifest.summary_bytes if manifest else 0,
        "event_bytes": manifest.event_bytes if manifest else 0,
        "manifest_stream_crc32": manifest.manifest_stream_crc32 if manifest else 0,
        "feuf_export_end_session_id": feuf_export_end.session_id if feuf_export_end else 0,
        "feuf_export_end_frames_sent": feuf_export_end.frames_sent if feuf_export_end else 0,
        "feuf_export_end_payload_bytes_sent": feuf_export_end.payload_bytes_sent if feuf_export_end else 0,
        "feuf_export_end_section_crc32_xor": feuf_export_end.section_crc32_xor if feuf_export_end else 0,
        "feuf_export_end_status": feuf_export_end.status if feuf_export_end else 0,
        "feuf": parsed.to_summary_dict(),
    }


def _validate_export_id_matches_manifest(export_id: int, parsed: FEUFParseResult) -> None:
    manifest = parsed.manifest
    if manifest is None or manifest.protocol_version != 3:
        return
    if manifest.session_uid != (export_id & 0xFFFFFFFF):
        raise ValueError(
            f"v3 manifest session_uid mismatch: manifest={manifest.session_uid} export_id={export_id & 0xFFFFFFFF}"
        )


def default_session_root() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        return Path(local_app_data) / "ZY100_BLE_Tool" / "sessions"
    return Path(__file__).resolve().parents[1] / ".zy100_sessions"


def make_session_key(export_id: int, total_bytes: int, stream_crc32: int) -> str:
    return f"{export_id & 0xFFFFFFFF:08X}_{total_bytes}_{stream_crc32 & 0xFFFFFFFF:08X}"


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _session_sort_key(session: dict[str, Any]) -> tuple[int, int, str]:
    def as_int(value: Any) -> int:
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return 0

    return (
        as_int(session.get("start_time_ms")),
        as_int(session.get("session_seq")),
        str(session.get("created_at", "")),
    )


def _safe_dir_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", value)


def _validate_feuf_bytes(data: bytes, total_bytes: int, stream_crc32: int) -> FEUFParseResult:
    if len(data) != total_bytes:
        raise ValueError(f"FEUF length mismatch: {len(data)} != {total_bytes}")
    actual_crc = binascii.crc32(data) & 0xFFFFFFFF
    if actual_crc != (stream_crc32 & 0xFFFFFFFF):
        raise ValueError(f"FEUF CRC mismatch: 0x{actual_crc:08X} != 0x{stream_crc32 & 0xFFFFFFFF:08X}")
    return parse_feuf_stream(data)
