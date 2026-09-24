from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class DfuProtocolConfig:
    """Protocol placeholders to be filled after official OTA spec is confirmed."""

    ota_service_uuid: str | None = None
    dfu_service_uuid: str | None = None
    control_char_uuid: str | None = None
    data_char_uuid: str | None = None
    ack_char_uuid: str | None = None
    start_dfu_opcode: int | None = None
    validate_opcode: int | None = None
    activate_reset_opcode: int | None = None
    chunk_size: int | None = None
    crc_rule: str | None = None
    aes_rule: str | None = None
    extra_pending_items: list[str] = field(default_factory=list)

    def pending_items(self) -> list[str]:
        pending: list[str] = []
        required_map = {
            "OTA Service UUID": self.ota_service_uuid,
            "DFU Service UUID": self.dfu_service_uuid,
            "Control Characteristic UUID": self.control_char_uuid,
            "Data Characteristic UUID": self.data_char_uuid,
            "ACK Characteristic UUID": self.ack_char_uuid,
            "Start DFU opcode": self.start_dfu_opcode,
            "Validate opcode": self.validate_opcode,
            "Activate+Reset opcode": self.activate_reset_opcode,
            "Chunk size": self.chunk_size,
            "CRC rule": self.crc_rule,
            "AES rule": self.aes_rule,
        }
        for key, value in required_map.items():
            if value is None:
                pending.append(key)
        pending.extend(self.extra_pending_items)
        return pending

    def is_ready(self) -> bool:
        return len(self.pending_items()) == 0
