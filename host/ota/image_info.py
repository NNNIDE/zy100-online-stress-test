from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class OtaImageInfo:
    """Metadata for an OTA image file."""

    path: str
    file_name: str
    size_bytes: int
    sha256: str


def load_ota_image_info(path: str) -> OtaImageInfo:
    """Load OTA image metadata from a local file path."""
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists() or not resolved.is_file():
        raise ValueError("固件文件不存在")

    digest = hashlib.sha256()
    with resolved.open("rb") as f:
        while True:
            chunk = f.read(8192)
            if not chunk:
                break
            digest.update(chunk)

    return OtaImageInfo(
        path=str(resolved),
        file_name=resolved.name,
        size_bytes=resolved.stat().st_size,
        sha256=digest.hexdigest(),
    )
