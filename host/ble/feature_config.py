from __future__ import annotations

import binascii
import json
import os
import struct
import tempfile
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Final


SCHEMA_VERSION: Final[int] = 1
TARGET_NOTIFY: Final[int] = 1
TARGET_LOGO: Final[int] = 2
TARGET_ALL: Final[int] = 3
EFFECT_SOLID: Final[int] = 1
EFFECT_BLINK: Final[int] = 2
EFFECT_BREATH: Final[int] = 3
EFFECT_MARQUEE: Final[int] = 4
SPEED_CAPTURE_DEFAULT: Final[int] = 0
SPEED_SLOW: Final[int] = 1
SPEED_STANDARD: Final[int] = 2
SPEED_FAST: Final[int] = 3


@dataclass(frozen=True, slots=True)
class FeatureConfig:
    auto_capture: bool = False
    training_led: bool = True
    target: int = TARGET_NOTIFY
    effect: int = EFFECT_BREATH
    red: int = 0
    green: int = 0
    blue: int = 255
    brightness: int = 100
    speed: int = SPEED_CAPTURE_DEFAULT
    reverse: bool = False

    def validate(self) -> None:
        if self.target not in {TARGET_NOTIFY, TARGET_LOGO, TARGET_ALL}:
            raise ValueError("invalid LED target")
        if self.effect not in {
            EFFECT_SOLID,
            EFFECT_BLINK,
            EFFECT_BREATH,
            EFFECT_MARQUEE,
        }:
            raise ValueError("invalid LED effect")
        if not all(0 <= value <= 255 for value in (self.red, self.green, self.blue)):
            raise ValueError("RGB must be 0..255")
        if not 1 <= self.brightness <= 100:
            raise ValueError("brightness must be 1..100")
        if self.speed not in {
            SPEED_CAPTURE_DEFAULT,
            SPEED_SLOW,
            SPEED_STANDARD,
            SPEED_FAST,
        }:
            raise ValueError("invalid LED speed")
        if self.training_led and (self.red, self.green, self.blue) == (0, 0, 0):
            raise ValueError("训练灯开启时 RGB 不能全黑；全部熄灭请关闭训练灯")
        if self.effect == EFFECT_MARQUEE and self.target == TARGET_NOTIFY:
            raise ValueError("Notify 灯区不支持跑马")
        if self.effect == EFFECT_SOLID and self.speed != SPEED_CAPTURE_DEFAULT:
            raise ValueError("常亮不使用速度")
        if self.effect in {EFFECT_BLINK, EFFECT_MARQUEE} and self.speed == SPEED_CAPTURE_DEFAULT:
            raise ValueError("闪烁和跑马必须选择慢/标准/快")
        if self.effect != EFFECT_MARQUEE and self.reverse:
            raise ValueError("只有跑马支持反向")

    def wire_bytes(self) -> bytes:
        self.validate()
        flags = int(self.auto_capture)
        flags |= int(self.training_led) << 1
        flags |= int(self.reverse) << 2
        flags |= (self.speed & 0x03) << 3
        return bytes(
            (
                SCHEMA_VERSION,
                flags,
                self.target,
                self.effect,
                self.red,
                self.green,
                self.blue,
                self.brightness,
            )
        )

    def crc32(self, user_id: int) -> int:
        if not 1 <= user_id <= 0xFFFFFFFF:
            raise ValueError("user_id must be 1..0xFFFFFFFF")
        return binascii.crc32(struct.pack("<I", user_id) + self.wire_bytes()) & 0xFFFFFFFF

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "FeatureConfig":
        if int(payload.get("schema", SCHEMA_VERSION)) != SCHEMA_VERSION:
            raise ValueError("unsupported feature config schema")
        config = cls(
            auto_capture=bool(payload.get("auto_capture", False)),
            training_led=bool(payload.get("training_led", True)),
            target=int(payload.get("target", TARGET_NOTIFY)),
            effect=int(payload.get("effect", EFFECT_BREATH)),
            red=int(payload.get("red", 0)),
            green=int(payload.get("green", 0)),
            blue=int(payload.get("blue", 255)),
            brightness=int(payload.get("brightness", 100)),
            speed=int(payload.get("speed", SPEED_CAPTURE_DEFAULT)),
            reverse=bool(payload.get("reverse", False)),
        )
        config.validate()
        return config

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["schema"] = SCHEMA_VERSION
        return payload


class FeatureConfigStore:
    def __init__(self, root: Path | None = None) -> None:
        if root is None:
            local = os.environ.get("LOCALAPPDATA")
            root = Path(local) / "ZY100_BLE_Tool" if local else Path.home() / ".zy100_ble_tool"
        self.root = Path(root)

    def path_for_user(self, user_id: int) -> Path:
        if not 1 <= user_id <= 0xFFFFFFFF:
            raise ValueError("user_id must be 1..0xFFFFFFFF")
        return self.root / f"feature_config_user_{user_id}.json"

    def load(self, user_id: int) -> tuple[FeatureConfig, str | None]:
        path = self.path_for_user(user_id)
        if not path.exists():
            return FeatureConfig(), None
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict) or int(payload.get("user_id", 0)) != user_id:
                raise ValueError("profile user_id mismatch")
            return FeatureConfig.from_dict(payload), None
        except Exception as exc:
            self.root.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            corrupt = path.with_suffix(path.suffix + f".corrupt-{stamp}")
            os.replace(path, corrupt)
            return FeatureConfig(), f"配置文件损坏，已保留为 {corrupt.name}：{exc}"

    def save(self, user_id: int, config: FeatureConfig) -> Path:
        config.validate()
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.path_for_user(user_id)
        payload = config.to_dict()
        payload["user_id"] = user_id
        payload["updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        handle = tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=path.name + ".",
            suffix=".tmp",
            dir=self.root,
            delete=False,
        )
        temp_path = Path(handle.name)
        try:
            with handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_path, path)
        finally:
            if temp_path.exists():
                temp_path.unlink()
        return path
