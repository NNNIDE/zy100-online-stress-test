from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


class GattTrustStore:
    """Persist only devices that have completed the full BUSINESS_READY gate."""

    def __init__(self, path: Path | None = None) -> None:
        root = Path(os.environ.get("LOCALAPPDATA", Path.cwd())) / "ZY100_BLE_Tool"
        self.path = path or root / "gatt_trust.json"

    def get(self, address: str) -> dict[str, Any] | None:
        return self._load().get(address.upper())

    def save(self, address: str, record: dict[str, Any]) -> None:
        data = self._load()
        data[address.upper()] = record
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def invalidate(self, address: str) -> None:
        data = self._load()
        if data.pop(address.upper(), None) is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def _load(self) -> dict[str, dict[str, Any]]:
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            return {}
        if not isinstance(value, dict):
            return {}
        return {str(key): item for key, item in value.items() if isinstance(item, dict)}
