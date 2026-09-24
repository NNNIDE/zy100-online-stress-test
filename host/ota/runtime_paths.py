from __future__ import annotations

import sys
from pathlib import Path


def _iter_root_candidates() -> list[Path]:
    roots: list[Path] = []

    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        roots.append(Path(meipass))

    executable = getattr(sys, "executable", "")
    if executable:
        roots.append(Path(executable).resolve().parent)

    roots.append(Path(__file__).resolve().parents[1])
    roots.append(Path.cwd())

    deduped: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root).lower()
        if key in seen:
            continue
        seen.add(key)
        deduped.append(root)
    return deduped


def get_firmware_directory() -> Path | None:
    for root in _iter_root_candidates():
        candidate = root / "firmware"
        if candidate.exists() and candidate.is_dir():
            return candidate
    return None


def list_firmware_images() -> list[Path]:
    directory = get_firmware_directory()
    if directory is None:
        return []

    files = [path for path in directory.glob("*.bin") if path.is_file()]
    files.sort(key=lambda path: path.name.lower())
    return files
