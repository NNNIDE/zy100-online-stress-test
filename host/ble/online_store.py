from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import time
import tempfile
import zipfile
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .online_stress import RECORD_STRESS
from .calibration_protocol import CalibrationRecord, parse_calibration_record
from .calibration_store import CalibrationStore
from .zy100_protocol import (
    ONLINE_RECORD_EVENT,
    ONLINE_RECORD_RAW,
    ONLINE_RECORD_SUMMARY,
    OnlineCompletedRecord,
    OnlineEndPayload,
    OnlineStartPayload,
    online_record_type_name,
)


ONLINE_METADATA_NAME = "metadata.json"
ONLINE_CALIBRATION_INFO_NAME = "calibration_info.json"
ONLINE_CALIBRATION_RECORD_NAME = "calibration_record.bin"
ONLINE_RECORD_KINDS = ("raw", "summary", "event", "stress")
ONLINE_SAMPLE_DIR_NAME = "samples"
ONLINE_HEADER_DIR_NAME = "headers"


class OnlineStorageError(OSError):
    def __init__(self, stage: str, path: Path, cause: Exception) -> None:
        self.stage = stage
        self.path = path
        super().__init__(f"{stage}: {path}: {type(cause).__name__}: {cause}")


class OnlineSessionStore:
    def __init__(
        self,
        root: Path | None = None,
        *,
        calibration_store: CalibrationStore | None = None,
    ) -> None:
        self.root = root or default_online_root()
        self.calibration_store = calibration_store or CalibrationStore()
        self.session_dir: Path | None = None
        self.metadata: dict[str, Any] = {}
        self.metadata_dirty = False
        self.last_save_profile: dict[str, Any] | None = None
        self._save_stages: dict[str, float] | None = None
        self._stress_samples: list[dict] = []
        self._stress_terminal: dict | None = None

    def recover_incomplete_sessions(self) -> int:
        """Close sessions that cannot receive a terminal frame after restart."""
        if not self.root.exists():
            return 0
        recovered = 0
        for session_dir in self.root.iterdir():
            metadata_path = session_dir / ONLINE_METADATA_NAME
            if not session_dir.is_dir() or not metadata_path.is_file():
                continue
            try:
                metadata = self._read_json(metadata_path)
            except (OSError, ValueError):
                continue
            if metadata.get("status") not in {"active", "ending"}:
                continue
            metadata["status"] = "aborted"
            metadata["terminal_source"] = "host_restart"
            metadata["terminal_reason"] = "host_restart_before_online_terminal"
            metadata["last_error"] = "host_restart_before_online_terminal"
            metadata["host_ended_at"] = now_iso()
            metadata["updated_at"] = metadata["host_ended_at"]
            self._write_json_atomic(metadata_path, metadata)
            recovered += 1
        return recovered

    def list_sessions(self, *, include_stress: bool = False) -> list[dict[str, Any]]:
        if not self.root.exists():
            return []

        sessions: list[dict[str, Any]] = []
        for session_dir in sorted(self.root.iterdir(), reverse=True):
            if not session_dir.is_dir():
                continue
            metadata_path = session_dir / ONLINE_METADATA_NAME
            if metadata_path.exists():
                try:
                    metadata = self._read_json(metadata_path)
                except (OSError, ValueError):
                    metadata = self._corrupt_metadata(session_dir.name, "metadata unreadable")
            else:
                metadata = self._corrupt_metadata(session_dir.name, "metadata missing")

            if not include_stress and (metadata.get("session_kind") == "link_stress"
                    or "_STRESS" in session_dir.name):
                continue  # Diagnostic data must not enter the ordinary sensor export list.
            metadata["session_key"] = metadata.get("session_key") or session_dir.name
            metadata["session_dir"] = str(session_dir)
            sample_files = self._sample_files(session_dir)
            metadata["sample_file_count"] = len(sample_files)
            metadata["sample_bytes"] = sum(path.stat().st_size for path in sample_files)
            metadata["header_file_count"] = len(self._header_files(session_dir))
            metadata["can_download"] = metadata_path.is_file() and metadata.get("status") != "corrupt"
            sessions.append(metadata)

        sessions.sort(key=_online_session_sort_key, reverse=True)
        return sessions

    def start(
        self,
        start: OnlineStartPayload,
        *,
        device_name: str = "",
        device_address: str = "",
        capability_mask: int = 0,
        calibration: dict[str, Any] | None = None,
        calibration_record: bytes | None = None,
        stress_config: dict | None = None,
    ) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        key = time.strftime("%Y%m%d_%H%M%S") + f"_{start.session_id:08X}"
        if stress_config is not None:
            key += "_STRESS"
        self._stress_samples = []
        self._stress_terminal = None
        session_dir = self.root / key
        suffix = 1
        while session_dir.exists():
            suffix += 1
            session_dir = self.root / f"{key}_{suffix}"
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / ONLINE_SAMPLE_DIR_NAME).mkdir(exist_ok=True)
        (session_dir / ONLINE_HEADER_DIR_NAME).mkdir(exist_ok=True)

        self.session_dir = session_dir
        host_started_at = now_iso()
        calibration_snapshot = dict(calibration or {})
        record = self._validate_session_calibration(
            calibration_snapshot,
            calibration_record,
        )
        calibration_record_metadata = self._calibration_record_metadata(record)
        self.metadata = {
            "session_key": session_dir.name,
            "session_kind": "link_stress" if stress_config is not None else "sensor_capture",
            "stress_config": dict(stress_config) if stress_config is not None else None,
            "stress_next_block": 0,
            "stress_saved_bytes": 0,
            "stress_record_count": 0,
            "session_id": start.session_id,
            "user_id": start.user_id,
            "training_id": start.training_id,
            "ack_timeout_ms": start.ack_timeout_ms,
            "summary_ack_batch": start.summary_ack_batch,
            "spool_region_bytes": start.spool_region_bytes,
            "contiguous_erased_bytes": start.contiguous_erased_bytes,
            "device_name": device_name,
            "device_address": device_address,
            "requested_capability_mask": int(capability_mask) & 0xFFFFFFFF,
            "calibration": calibration_snapshot,
            "calibration_record": calibration_record_metadata,
            "host_started_at": host_started_at,
            "started_at": "",
            "status": "active",
            "sample_rate_hz": 800,
            "record_count": 0,
            "sample_count": 0,
            "packet_count": 0,
            "continuous_raw_record_count": 0,
            "first_packet_sequence": None,
            "last_packet_sequence": None,
            "next_packet_sequence": None,
            "first_imu_timestamp_raw": None,
            "last_imu_timestamp_raw": None,
            "packet_sequence_continuous": True,
            "raw_source_versions": [],
            "mag_sample_rate_hz": 100,
            "mag_sample_count": 0,
            "mag_raw_record_count": 0,
            "mag_read_error_count": 0,
            "mag_missed_deadline_count": 0,
            "first_mag_elapsed_us": None,
            "last_mag_elapsed_us": None,
            "last_mag_elapsed_raw_u32": None,
            "mag_elapsed_monotonic": True,
            "mag_fresh_ready_gated": False,
            "sample_bytes": 0,
            "ack_counts": {"raw": 0, "summary": 0, "event": 0, "end": 0},
            "segment_metadata": {
                "records_with_segment_metadata": 0,
                "records_without_segment_metadata": 0,
                "min_capture_segment_id": None,
                "max_capture_segment_id": None,
                "last_capture_segment_id": None,
                "last_segment_start_offset_ms": None,
                "last_source_id": None,
            },
            "last_error": "",
        }
        self._write_json_atomic(
            session_dir / ONLINE_CALIBRATION_INFO_NAME,
            calibration_snapshot,
        )
        if record is not None:
            self._write_bytes_atomic(
                session_dir / ONLINE_CALIBRATION_RECORD_NAME,
                record.raw,
            )
        self._write_metadata()
        return session_dir

    @contextmanager
    def _save_stage(self, name: str):
        """Measure non-overlapping steps only while saving one record."""
        stages = self._save_stages
        if stages is None:
            yield
            return
        started_ns = time.perf_counter_ns()
        try:
            yield
        finally:
            elapsed_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
            stages[name] = stages.get(name, 0.0) + elapsed_ms

    def save_record(self, record: OnlineCompletedRecord) -> Path:
        self.last_save_profile = None
        stages: dict[str, float] = {}
        self._save_stages = stages
        started_ns = time.perf_counter_ns()
        cpu_started_ns = time.thread_time_ns()
        succeeded = False
        try:
            result = self._save_record(record)
            succeeded = True
            return result
        finally:
            cpu_ms = (time.thread_time_ns() - cpu_started_ns) / 1_000_000.0
            wall_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
            self._save_stages = None
            # Thread CPU is the Windows host thread, not MCU CPU loading.
            self.last_save_profile = {
                "record_id": record.record_id, "ok": succeeded,
                "wall_ms": wall_ms, "cpu_ms": cpu_ms, "stages": stages,
            }

    def _save_record(self, record: OnlineCompletedRecord) -> Path:
        if self.session_dir is None:
            raise RuntimeError("online session is not active")
        with self._save_stage("validate"):
            self._validate_continuous_sequence(record)
            self._validate_mag_sequence(record)
            if self.metadata.get("session_id") is not None and record.session_id != self.metadata["session_id"]:
                raise ValueError("record belongs to another session")
            if record.stress is not None:
                config = self.metadata.get("stress_config")
                if (not config or record.stress.rate_Bps != config["rate_Bps"]
                        or record.stress.first_block != self.metadata["stress_next_block"]
                        or record.stress.byte_offset != self.metadata["stress_saved_bytes"]):
                    raise ValueError("STRESS missing/duplicate block, wrong rate or non-stress session")
        target_dir = self.session_dir / ONLINE_SAMPLE_DIR_NAME
        with self._save_stage("record_dir"):
            target_dir.mkdir(exist_ok=True)
        target = target_dir / f"{record.record_type:02X}_{record.record_id:08d}.bin"
        if record.stress is not None and target.exists():
            raise ValueError("duplicate STRESS record file")
        tmp = target.with_suffix(".bin.tmp")
        stage = "record_write"
        try:
            with self._save_stage("record_open"):
                handle = tmp.open("w+b")
            # Keep ownership through verification. Reopening a just-closed file
            # adds an unnecessary filesystem/filter-driver round trip per record.
            with handle:
                with self._save_stage(stage):
                    handle.write(record.data)
                stage = "record_flush"
                with self._save_stage(stage):
                    handle.flush()
                stage = "record_verify"
                with self._save_stage(stage):
                    handle.seek(0)
                    if handle.read() != record.data:
                        raise ValueError("online record write verify failed")
            stage = "record_replace"
            with self._save_stage(stage):
                os.replace(tmp, target)
        except (OSError, ValueError) as exc:
            raise OnlineStorageError(stage, target, exc) from exc
        self._write_record_header(record, target.name)

        with self._save_stage("metadata_update"):
            self.metadata["record_count"] = int(self.metadata.get("record_count") or 0) + 1
            # Kept for read-only compatibility with sessions created before packet
            # accounting was added. sample_count continues to mean Record count.
            self.metadata["sample_count"] = int(self.metadata.get("sample_count") or 0) + 1
            self.metadata["sample_bytes"] = int(self.metadata.get("sample_bytes") or 0) + record.record_bytes
            self._note_continuous_raw(record)
            self._note_mag_raw(record)
            self._note_raw_source_version(record)
            if record.stress is not None:
                self.metadata["stress_next_block"] += record.stress.block_count
                self.metadata["stress_saved_bytes"] += record.stress.data_bytes
                self.metadata["stress_record_count"] += 1
            self._note_segment_metadata(record)
            self.metadata["last_record"] = record.to_dict()
            self.metadata["updated_at"] = now_iso()
        self._write_metadata()
        return target

    def note_ack(self, record_type: int, ack_count: int) -> None:
        if self.session_dir is None:
            return
        kind = "end" if record_type == 0xFF else _record_dir_name(record_type)
        ack_counts = self.metadata.setdefault("ack_counts", {})
        ack_counts[kind] = int(ack_counts.get(kind) or 0) + max(1, int(ack_count))
        self.metadata["updated_at"] = now_iso()
        self._write_metadata()

    def stage_end(self, end: OnlineEndPayload) -> dict[str, Any]:
        if self.session_dir is None:
            return analyze_online_timebase(end, {})
        self.metadata["status"] = "ending"
        self.metadata["terminal_source"] = "online_end"
        self.metadata["terminal_reason"] = "end_ack_pending"
        self.metadata["host_end_received_at"] = now_iso()
        self.metadata["ended_at"] = ""
        self.metadata["end"] = end.to_dict()
        if self.metadata.get("stress_config") is not None:
            self._stress_terminal = end.stress
        analysis = analyze_online_timebase(end, self.metadata)
        self.metadata["time_analysis"] = analysis
        # Read-only compatibility for UI/builds that predate the V3 field name.
        self.metadata["rtc_analysis"] = analysis
        if analysis.get("valid"):
            self.metadata["unix_start_us"] = analysis["first_unix_time_us"]
            self.metadata["unix_end_us"] = analysis["last_unix_time_us"]
            self.metadata["unix_start_local"] = analysis["first_unix_time_local"]
            self.metadata["unix_end_local"] = analysis["last_unix_time_local"]
            self.metadata["started_at"] = analysis["first_unix_time_local"]
            self.metadata["ended_at"] = analysis["last_unix_time_local"]
            self._note_mag_unix_timeline(int(analysis["first_unix_time_us"]))
        self._write_metadata()
        return analysis

    def commit_end(self) -> None:
        if self.session_dir is None:
            return
        if self.metadata.get("status") not in {"ending", "ended"}:
            raise RuntimeError("online END was not staged")
        self.metadata["status"] = "ended"
        self.metadata["terminal_source"] = "online_end_ack"
        self.metadata["terminal_reason"] = "end_ack_committed"
        self.metadata["host_ended_at"] = now_iso()
        self.metadata["updated_at"] = self.metadata["host_ended_at"]
        self._write_metadata()
        self.finalize_stress_report()

    def finish(self, end: OnlineEndPayload) -> dict[str, Any]:
        """Compatibility helper for offline analysis/tests."""
        analysis = self.stage_end(end)
        self.commit_end()
        return analysis

    def abort(
        self,
        reason: str,
        *,
        terminal_source: str = "host_error",
        device_state: int | None = None,
    ) -> None:
        if self.session_dir is None:
            return
        self.metadata["status"] = "aborted"
        self.metadata["terminal_source"] = terminal_source
        self.metadata["terminal_reason"] = reason
        if device_state is not None:
            self.metadata["terminal_device_state"] = int(device_state) & 0xFF
        self.metadata["last_error"] = reason
        self.metadata["host_ended_at"] = now_iso()
        self.metadata["updated_at"] = self.metadata["host_ended_at"]
        self._write_metadata()
        self.finalize_stress_report()

    def note_stress_status(self, status: dict, *, terminal: bool = False) -> None:
        if self.session_dir is None or self.metadata.get("stress_config") is None:
            raise ValueError("STRESS status outside configured session")
        if status["session_id"] != self.metadata["session_id"]:
            raise ValueError("STRESS status wrong session")
        if status["rate_Bps"] != self.metadata["stress_config"]["rate_Bps"]:
            raise ValueError("STRESS status wrong rate")
        self._stress_samples.append(dict(status))
        if terminal:
            self._stress_terminal = dict(status)
        with (self.session_dir / "stress_status.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(status, ensure_ascii=False) + "\n")

    def finalize_stress_report(self) -> dict | None:
        if self.session_dir is None or self.metadata.get("stress_config") is None:
            return None
        from .online_stress_report import build_stress_report, report_text
        report = build_stress_report(self.session_dir, self.metadata, self._stress_samples, self._stress_terminal)
        self._write_json_atomic(self.session_dir / "stress_report.json", report)
        self._write_bytes_atomic(self.session_dir / "stress_report.txt", report_text(report).encode("utf-8"))
        self.metadata["stress_verdict"] = report["verdict"]
        self.metadata["stress_report"] = str(self.session_dir / "stress_report.json")
        self._write_metadata()
        return report

    def reset(self) -> None:
        self.session_dir = None
        self.metadata = {}
        self.metadata_dirty = False

    def write_record_zip(self, session_key: str, record_kind: str, destination: str | Path) -> Path:
        kind = _normalize_record_kind(record_kind)
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"online session not found: {session_key}")
        metadata_path = session_dir / ONLINE_METADATA_NAME
        if not metadata_path.is_file():
            raise ValueError("online session metadata is missing")
        metadata = self._read_json(metadata_path)
        if metadata.get("status") == "corrupt":
            raise ValueError("online session metadata is corrupt")
        calibration_record_path = self._ensure_calibration_record(
            session_dir,
            metadata,
        )

        files = self._record_files(session_dir, kind)
        if not files:
            raise ValueError(f"online session has no {kind} records")

        destination_path = Path(destination)
        with self._atomic_zip(destination_path) as zf:
            zf.write(metadata_path, arcname=ONLINE_METADATA_NAME)
            calibration_path = session_dir / ONLINE_CALIBRATION_INFO_NAME
            if calibration_path.is_file():
                zf.write(calibration_path, arcname=ONLINE_CALIBRATION_INFO_NAME)
            if calibration_record_path is not None:
                zf.write(
                    calibration_record_path,
                    arcname=ONLINE_CALIBRATION_RECORD_NAME,
                )
            for path in files:
                zf.write(path, arcname=f"{kind}/{path.name}")
                header_path = self._header_path(session_dir, path.stem)
                if header_path.is_file():
                    zf.write(header_path, arcname=f"{ONLINE_HEADER_DIR_NAME}/{header_path.name}")
        return destination_path

    def write_session_zip(self, session_key: str, destination: str | Path) -> Path:
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"online session not found: {session_key}")
        metadata_path = session_dir / ONLINE_METADATA_NAME
        if not metadata_path.is_file():
            raise ValueError("online session metadata is missing")
        metadata = self._read_json(metadata_path)
        if metadata.get("status") == "corrupt":
            raise ValueError("online session metadata is corrupt")
        calibration_record_path = self._ensure_calibration_record(
            session_dir,
            metadata,
        )

        destination_path = Path(destination)
        with self._atomic_zip(destination_path) as zf:
            zf.write(metadata_path, arcname=ONLINE_METADATA_NAME)
            calibration_path = session_dir / ONLINE_CALIBRATION_INFO_NAME
            if calibration_path.is_file():
                zf.write(calibration_path, arcname=ONLINE_CALIBRATION_INFO_NAME)
            if calibration_record_path is not None:
                zf.write(
                    calibration_record_path,
                    arcname=ONLINE_CALIBRATION_RECORD_NAME,
                )
            zf.writestr(f"{ONLINE_SAMPLE_DIR_NAME}/", b"")
            for path in self._sample_files(session_dir):
                zf.write(path, arcname=f"{ONLINE_SAMPLE_DIR_NAME}/{path.name}")
            zf.writestr(f"{ONLINE_HEADER_DIR_NAME}/", b"")
            for path in self._header_files(session_dir):
                zf.write(path, arcname=f"{ONLINE_HEADER_DIR_NAME}/{path.name}")
        return destination_path

    @staticmethod
    @contextmanager
    def _atomic_zip(destination: Path):
        # A failed background export must not destroy a previously exported ZIP.
        with tempfile.NamedTemporaryFile(dir=destination.parent,
                prefix=destination.name + ".", suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
        try:
            with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                yield archive
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)

    def delete_session(self, session_key: str) -> None:
        session_dir = self._find_session_dir(session_key)
        if session_dir is None:
            raise FileNotFoundError(f"online session not found: {session_key}")
        root = self.root.resolve()
        target = session_dir.resolve()
        if target == root or root not in target.parents:
            raise ValueError(f"refusing to delete outside online session root: {target}")
        shutil.rmtree(target)

    def _write_metadata(self) -> None:
        if self.session_dir is None:
            return
        self.metadata_dirty = True
        target = self.session_dir / ONLINE_METADATA_NAME
        self._write_json_atomic(target, self.metadata)
        self.metadata_dirty = False

    def flush_metadata(self) -> None:
        """Persist current counters without incrementing an already sent ACK."""
        if self.metadata_dirty:
            self._write_metadata()

    def _write_json_atomic(self, target: Path, value: dict[str, Any]) -> None:
        tmp: Path | None = None
        prefix = "metadata" if target.name == ONLINE_METADATA_NAME else "header"
        stage = "json_encode"
        try:
            with self._save_stage(prefix + "_encode"):
                encoded = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)
            stage = "json_write"
            with self._save_stage(prefix + "_open"):
                handle = tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8",
                    dir=target.parent, prefix=target.name + ".", suffix=".tmp",
                    delete=False)
            with handle as f:
                tmp = Path(f.name)
                with self._save_stage(prefix + "_write"):
                    f.write(encoded)
                stage = "json_flush"
                with self._save_stage(prefix + "_flush"):
                    f.flush()
                stage = "json_verify"
                with self._save_stage(prefix + "_verify"):
                    self._verify_json_handle(f, value)
            stage = "json_replace"
            with self._save_stage(prefix + "_replace"):
                os.replace(tmp, target)
        except (OSError, ValueError, TypeError) as exc:
            raise OnlineStorageError(stage, target, exc) from exc
        finally:
            if tmp is not None:
                with self._save_stage(prefix + "_cleanup"):
                    try:
                        tmp.unlink(missing_ok=True)
                    except OSError:
                        pass  # Preserve the original failure; never delete the target.

    @staticmethod
    def _verify_json_handle(handle, expected: dict[str, Any]) -> None:
        # flush+seek switches TextIOWrapper to actual readback on the same
        # handle; do not replace this with json.loads(encoded), which would
        # only verify the memory buffer rather than the written file.
        handle.seek(0)
        if json.load(handle) != expected:
            raise ValueError("JSON read-back mismatch")

    def _write_bytes_atomic(self, target: Path, payload: bytes) -> None:
        tmp = target.with_name(target.name + ".tmp")
        with tmp.open("wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        if tmp.read_bytes() != payload:
            raise ValueError(f"online calibration record write verify failed: {target}")
        os.replace(tmp, target)

    @staticmethod
    def _record_matches_summary(
        record: CalibrationRecord,
        calibration: dict[str, Any],
    ) -> bool:
        return (
            bool(calibration.get("valid"))
            and record.generation == int(calibration.get("generation") or 0)
            and record.valid_flags == int(calibration.get("valid_flags") or 0)
            and record.record_bytes == int(calibration.get("record_bytes") or 0)
            and record.crc32 == (int(calibration.get("crc32") or 0) & 0xFFFFFFFF)
        )

    def _validate_session_calibration(
        self,
        calibration: dict[str, Any],
        payload: bytes | None,
    ) -> CalibrationRecord | None:
        if not bool(calibration.get("valid")):
            if payload is not None:
                raise ValueError("uncalibrated session must not contain a calibration record")
            return None
        if payload is None:
            raise ValueError("valid calibration summary requires a complete calibration record")
        record = parse_calibration_record(payload)
        if not self._record_matches_summary(record, calibration):
            raise ValueError("calibration record does not match the session summary")
        return record

    @staticmethod
    def _calibration_record_metadata(
        record: CalibrationRecord | None,
    ) -> dict[str, Any]:
        if record is None:
            return {
                "present": False,
                "filename": "",
                "record_bytes": 0,
                "generation": 0,
                "crc32": 0,
                "sha256": "",
            }
        return {
            "present": True,
            "filename": ONLINE_CALIBRATION_RECORD_NAME,
            "record_bytes": record.record_bytes,
            "generation": record.generation,
            "crc32": record.crc32,
            "sha256": hashlib.sha256(record.raw).hexdigest().upper(),
        }

    def _ensure_calibration_record(
        self,
        session_dir: Path,
        metadata: dict[str, Any],
    ) -> Path | None:
        calibration = dict(metadata.get("calibration") or {})
        if not bool(calibration.get("valid")):
            return None
        target = session_dir / ONLINE_CALIBRATION_RECORD_NAME
        if target.is_file():
            record = parse_calibration_record(target.read_bytes())
            if not self._record_matches_summary(record, calibration):
                raise ValueError("stored calibration record does not match session summary")
            return target

        record = self.calibration_store.find_verified_record(
            generation=int(calibration.get("generation") or 0),
            crc32=int(calibration.get("crc32") or 0),
            record_bytes=int(calibration.get("record_bytes") or 0),
            device_name=str(metadata.get("device_name") or ""),
            device_address=str(metadata.get("device_address") or ""),
        )
        if record is None or not self._record_matches_summary(record, calibration):
            raise ValueError(
                "matching complete calibration record is unavailable for this session"
            )
        self._write_bytes_atomic(target, record.raw)
        metadata["calibration_record"] = self._calibration_record_metadata(record)
        self._write_json_atomic(session_dir / ONLINE_METADATA_NAME, metadata)
        return target

    def _write_record_header(self, record: OnlineCompletedRecord, sample_name: str) -> None:
        if self.session_dir is None:
            raise RuntimeError("online session is not active")
        target = self._header_path(self.session_dir, Path(sample_name).stem)
        target.parent.mkdir(exist_ok=True)
        value: dict[str, Any] = {
            "captured_at": now_iso(),
            "source_file": f"{ONLINE_SAMPLE_DIR_NAME}/{sample_name}",
            "record": record.to_dict(),
            "online_spool_header": record.header.to_dict(),
            "online_spool_header_hex": record.data[:record.header.header_len].hex(),
        }
        self._write_json_atomic(target, value)

    def _note_segment_metadata(self, record: OnlineCompletedRecord) -> None:
        stats = self.metadata.setdefault("segment_metadata", {})
        header = record.header
        if not header.has_segment_metadata:
            stats["records_without_segment_metadata"] = (
                int(stats.get("records_without_segment_metadata") or 0) + 1
            )
            return
        segment_id = header.capture_segment_id
        if segment_id is None:
            raise ValueError("segment metadata flag set without capture_segment_id")
        stats["records_with_segment_metadata"] = (
            int(stats.get("records_with_segment_metadata") or 0) + 1
        )
        min_segment_id = stats.get("min_capture_segment_id")
        max_segment_id = stats.get("max_capture_segment_id")
        stats["min_capture_segment_id"] = (
            segment_id if min_segment_id is None else min(int(min_segment_id), segment_id)
        )
        stats["max_capture_segment_id"] = (
            segment_id if max_segment_id is None else max(int(max_segment_id), segment_id)
        )
        stats["last_capture_segment_id"] = segment_id
        stats["last_segment_start_offset_ms"] = header.segment_start_offset_ms
        stats["last_source_id"] = header.source_id

    def _validate_continuous_sequence(self, record: OnlineCompletedRecord) -> None:
        info = record.continuous_raw
        if info is None:
            return
        current_count = int(self.metadata.get("packet_count") or 0)
        expected = self.metadata.get("next_packet_sequence")
        if current_count == 0:
            if info.start_seq == 0:
                raise ValueError("continuous RAW first sequence is zero")
            return
        if expected is None or info.start_seq != int(expected):
            self.metadata["packet_sequence_continuous"] = False
            self.metadata["last_error"] = (
                "continuous RAW record sequence mismatch: "
                f"got={info.start_seq} expected={expected}"
            )
            self.metadata["updated_at"] = now_iso()
            self._write_metadata()
            raise ValueError(str(self.metadata["last_error"]))

    def _note_continuous_raw(self, record: OnlineCompletedRecord) -> None:
        info = record.continuous_raw
        if info is None:
            return
        current_count = int(self.metadata.get("packet_count") or 0)
        if current_count == 0:
            self.metadata["first_packet_sequence"] = info.start_seq
            self.metadata["first_imu_timestamp_raw"] = info.first_timestamp_raw
        self.metadata["packet_count"] = current_count + info.packet_count
        self.metadata["continuous_raw_record_count"] = (
            int(self.metadata.get("continuous_raw_record_count") or 0) + 1
        )
        self.metadata["last_packet_sequence"] = info.end_seq
        self.metadata["next_packet_sequence"] = (info.end_seq + 1) & 0xFFFFFFFF
        self.metadata["last_imu_timestamp_raw"] = info.last_timestamp_raw

    def _validate_mag_sequence(self, record: OnlineCompletedRecord) -> None:
        info = record.mag_raw
        if info is None:
            return
        previous = self.metadata.get("last_mag_elapsed_raw_u32")
        if previous is None:
            return
        delta_us = (info.first_elapsed_us - int(previous)) & 0xFFFFFFFF
        if delta_us == 0 or delta_us >= (1 << 31):
            self.metadata["mag_elapsed_monotonic"] = False
            self.metadata["last_error"] = (
                "MAG elapsed_us modulo sequence mismatch: "
                f"got={info.first_elapsed_us} previous={previous}"
            )
            self.metadata["updated_at"] = now_iso()
            self._write_metadata()
            raise ValueError(str(self.metadata["last_error"]))

    def _note_mag_raw(self, record: OnlineCompletedRecord) -> None:
        info = record.mag_raw
        if info is None:
            return
        current_count = int(self.metadata.get("mag_sample_count") or 0)
        if current_count == 0:
            first_elapsed_us = info.first_elapsed_us
        else:
            previous_raw = int(self.metadata["last_mag_elapsed_raw_u32"])
            previous_elapsed = int(self.metadata["last_mag_elapsed_us"])
            first_elapsed_us = previous_elapsed + (
                (info.first_elapsed_us - previous_raw) & 0xFFFFFFFF
            )
        last_elapsed_us = first_elapsed_us
        previous_raw = info.first_elapsed_us
        for sample in info.samples[1:]:
            last_elapsed_us += (sample.elapsed_us - previous_raw) & 0xFFFFFFFF
            previous_raw = sample.elapsed_us
        if current_count == 0:
            self.metadata["first_mag_elapsed_us"] = first_elapsed_us
        self.metadata["mag_sample_count"] = current_count + info.sample_count
        self.metadata["mag_raw_record_count"] = (
            int(self.metadata.get("mag_raw_record_count") or 0) + 1
        )
        self.metadata["mag_read_error_count"] = (
            int(self.metadata.get("mag_read_error_count") or 0) + info.read_error_count
        )
        self.metadata["mag_missed_deadline_count"] = (
            int(self.metadata.get("mag_missed_deadline_count") or 0)
            + info.missed_deadline_count
        )
        self.metadata["last_mag_elapsed_us"] = last_elapsed_us
        self.metadata["last_mag_elapsed_raw_u32"] = info.last_elapsed_us
        if info.fresh_ready_gated:
            self.metadata["mag_fresh_ready_gated"] = True

    def _note_raw_source_version(self, record: OnlineCompletedRecord) -> None:
        if record.raw_source_version is None:
            return
        versions = self.metadata.setdefault("raw_source_versions", [])
        version = int(record.raw_source_version)
        if version not in versions:
            versions.append(version)
            versions.sort()

    def _note_mag_unix_timeline(self, first_unix_time_us: int) -> None:
        first_elapsed = self.metadata.get("first_mag_elapsed_us")
        last_elapsed = self.metadata.get("last_mag_elapsed_us")
        sample_count = int(self.metadata.get("mag_sample_count") or 0)
        if first_elapsed is None or last_elapsed is None or sample_count == 0:
            return
        first_elapsed_us = int(first_elapsed)
        last_elapsed_us = int(last_elapsed)
        self.metadata["mag_first_unix_time_us"] = first_unix_time_us + first_elapsed_us
        self.metadata["mag_last_unix_time_us"] = first_unix_time_us + last_elapsed_us
        self.metadata["mag_first_unix_time_local"] = unix_us_to_local_iso(
            first_unix_time_us + first_elapsed_us
        )
        self.metadata["mag_last_unix_time_local"] = unix_us_to_local_iso(
            first_unix_time_us + last_elapsed_us
        )
        if sample_count >= 2 and last_elapsed_us > first_elapsed_us:
            duration_us = last_elapsed_us - first_elapsed_us
            self.metadata["mag_duration_us"] = duration_us
            self.metadata["mag_average_rate_hz"] = (
                (sample_count - 1) * 1_000_000.0 / duration_us
            )

    @staticmethod
    def _header_path(session_dir: Path, record_stem: str) -> Path:
        return session_dir / ONLINE_HEADER_DIR_NAME / f"{record_stem}.json"

    @staticmethod
    def _header_files(session_dir: Path) -> list[Path]:
        header_dir = session_dir / ONLINE_HEADER_DIR_NAME
        if not header_dir.is_dir():
            return []
        return sorted(header_dir.glob("*.json"))

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as f:
            value = json.load(f)
        if not isinstance(value, dict):
            raise ValueError(f"metadata is not an object: {path}")
        return value

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

    @staticmethod
    def _record_files(session_dir: Path, kind: str) -> list[Path]:
        record_dir = session_dir / kind
        if not record_dir.is_dir():
            return []
        return sorted(path for path in record_dir.iterdir() if path.is_file() and path.suffix.lower() == ".bin")

    @staticmethod
    def _sample_files(session_dir: Path) -> list[Path]:
        sample_dir = session_dir / ONLINE_SAMPLE_DIR_NAME
        if sample_dir.is_dir():
            return sorted(path for path in sample_dir.iterdir() if path.is_file() and path.suffix.lower() == ".bin")
        # Read-only compatibility for sessions created by earlier releases.
        files: list[Path] = []
        for kind in ONLINE_RECORD_KINDS:
            files.extend(OnlineSessionStore._record_files(session_dir, kind))
        return sorted(files)

    @staticmethod
    def _corrupt_metadata(session_key: str, reason: str) -> dict[str, Any]:
        return {
            "session_key": session_key,
            "status": "corrupt",
            "last_error": reason,
            "started_at": "",
        }


def analyze_online_timebase(
    end: OnlineEndPayload,
    metadata: dict[str, Any],
) -> dict[str, Any]:
    packet_count = int(metadata.get("packet_count") or 0)
    sequence_continuous = bool(metadata.get("packet_sequence_continuous", True))
    first_file_timestamp = metadata.get("first_imu_timestamp_raw")
    last_file_timestamp = metadata.get("last_imu_timestamp_raw")
    clock = end.clock_meta
    result: dict[str, Any] = {
        "valid": False,
        "invalid_reason": "",
        "packet_count": packet_count,
        "packet_sequence_continuous": sequence_continuous,
        "mcu_accepted_packet_count": (
            clock.accepted_packet_count if clock is not None else None
        ),
        "packet_count_match": False,
        "first_timestamp_match": False,
        "last_timestamp_match": False,
        "endpoint_version": clock.version if clock is not None else None,
        "duration_us": None,
        "duration_s": None,
        "average_interval_us": None,
        "average_rate_hz": None,
        "deviation_ppm_from_800": None,
        "first_unix_time_us": None,
        "last_unix_time_us": None,
        "first_unix_time_local": "",
        "last_unix_time_local": "",
    }

    def invalid(reason: str) -> dict[str, Any]:
        result["invalid_reason"] = reason
        return result

    if clock is None:
        return invalid("legacy_end_without_time_metadata")
    if not clock.supported:
        return invalid(f"unsupported_time_metadata_version_{clock.version}")
    if clock.version != 3 or clock.meta_bytes != 32:
        return invalid("fixed40_unix_endpoint_v3_required")
    if end.stop_reason != 1:
        return invalid(f"stop_reason_not_host_stop_{end.stop_reason}")
    if clock.status != 0:
        return invalid(f"mcu_timebase_status_{clock.status}")
    if not sequence_continuous:
        return invalid("packet_sequence_discontinuous")

    accepted_count = int(clock.accepted_packet_count or 0)
    result["packet_count_match"] = packet_count == accepted_count
    result["first_timestamp_match"] = (
        first_file_timestamp is not None
        and int(first_file_timestamp) == int(clock.first_imu_timestamp_raw or 0)
    )
    result["last_timestamp_match"] = (
        last_file_timestamp is not None
        and int(last_file_timestamp) == int(clock.last_imu_timestamp_raw or 0)
    )
    if packet_count < 2 or accepted_count < 2:
        return invalid("less_than_two_packets")
    if not result["packet_count_match"]:
        return invalid("mcu_host_packet_count_mismatch")
    if not result["first_timestamp_match"]:
        return invalid("first_packet_timestamp_binding_mismatch")
    if not result["last_timestamp_match"]:
        return invalid("last_packet_timestamp_binding_mismatch")

    first_unix_us = int(clock.first_unix_time_us or 0)
    last_unix_us = int(clock.last_unix_time_us or 0)
    if first_unix_us <= 0 or last_unix_us <= first_unix_us:
        return invalid("invalid_unix_endpoint")
    duration_us = last_unix_us - first_unix_us
    intervals = packet_count - 1
    duration_s = duration_us / 1_000_000.0
    average_interval_us = duration_us / intervals
    average_rate_hz = (intervals * 1_000_000.0) / duration_us
    result.update(
        {
            "valid": True,
            "invalid_reason": "",
            "duration_us": duration_us,
            "duration_s": duration_s,
            "average_interval_us": average_interval_us,
            "average_rate_hz": average_rate_hz,
            "deviation_ppm_from_800": (average_rate_hz / 800.0 - 1.0) * 1_000_000.0,
            "first_unix_time_us": first_unix_us,
            "last_unix_time_us": last_unix_us,
            "first_unix_time_local": unix_us_to_local_iso(first_unix_us),
            "last_unix_time_local": unix_us_to_local_iso(last_unix_us),
        }
    )
    return result


def analyze_online_rtc(
    end: OnlineEndPayload,
    metadata: dict[str, Any],
) -> dict[str, Any]:
    """Compatibility alias for callers built before Fixed40 Endpoint V3."""
    return analyze_online_timebase(end, metadata)


def default_online_root() -> Path:
    base = os.environ.get("LOCALAPPDATA")
    if base:
        return Path(base) / "ZY100_BLE_Tool" / "online_sessions"
    return Path.home() / "AppData" / "Local" / "ZY100_BLE_Tool" / "online_sessions"


def _record_dir_name(record_type: int) -> str:
    if record_type == ONLINE_RECORD_RAW:
        return "raw"
    if record_type == ONLINE_RECORD_SUMMARY:
        return "summary"
    if record_type == ONLINE_RECORD_EVENT:
        return "event"
    if record_type == RECORD_STRESS:
        return "stress"
    raise ValueError(f"unsupported online record type: {online_record_type_name(record_type)}")


def _normalize_record_kind(value: str) -> str:
    kind = str(value).strip().lower()
    if kind not in ONLINE_RECORD_KINDS:
        raise ValueError(f"unsupported online record kind: {value}")
    return kind


def _safe_dir_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", value)


def _online_session_sort_key(session: dict[str, Any]) -> tuple[str, str]:
    return (
        str(session.get("unix_start_local") or session.get("host_started_at") or ""),
        str(session.get("session_key") or ""),
    )


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def unix_ms_to_local_iso(value: int) -> str:
    return (
        datetime.fromtimestamp(int(value) / 1000.0, tz=timezone.utc)
        .astimezone()
        .isoformat(timespec="milliseconds")
    )


def unix_us_to_local_iso(value: int) -> str:
    seconds, microseconds = divmod(int(value), 1_000_000)
    return (
        datetime.fromtimestamp(seconds, tz=timezone.utc)
        .replace(microsecond=microseconds)
        .astimezone()
        .isoformat(timespec="microseconds")
    )
