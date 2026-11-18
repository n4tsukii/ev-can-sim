"""DBC-based encoding/decoding with E2E protection (alive counter + CRC8)."""
from __future__ import annotations

from pathlib import Path

import cantools

DEFAULT_DBC = Path(__file__).resolve().parent.parent / "dbc" / "ev_network.dbc"


def crc8_j1850(data: bytes) -> int:
    """CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xorout 0xFF). Check value for b'123456789' is 0x4B."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


class Codec:
    """Wraps a cantools database. Every protected message has AliveCounter (4 bit) and Checksum (byte 7)."""

    def __init__(self, dbc_path: str | Path = DEFAULT_DBC):
        self.db = cantools.database.load_file(str(dbc_path))

    def message(self, name_or_id):
        if isinstance(name_or_id, int):
            return self.db.get_message_by_frame_id(name_or_id)
        return self.db.get_message_by_name(name_or_id)

    def frame_id(self, name: str) -> int:
        return self.message(name).frame_id

    def cycle_time_s(self, name: str) -> float:
        return self.message(name).cycle_time / 1000.0

    def encode(self, name: str, signals: dict, counter: int, corrupt: bool = False) -> bytes:
        msg = self.message(name)
        values = {s.name: 0 for s in msg.signals}
        values.update(signals)
        values["AliveCounter"] = counter & 0x0F
        values["Checksum"] = 0
        data = bytearray(msg.encode(values))
        data[7] = crc8_j1850(bytes(data[:7]))
        if corrupt:
            data[7] ^= 0xFF
        return bytes(data)

    def decode(self, frame_id: int, data: bytes) -> dict:
        return self.message(frame_id).decode(bytes(data), decode_choices=False)

    @staticmethod
    def checksum_ok(data: bytes) -> bool:
        return len(data) == 8 and crc8_j1850(bytes(data[:7])) == data[7]

    def choice_name(self, msg_name: str, signal: str, raw) -> str:
        sig = self.message(msg_name).get_signal_by_name(signal)
        if sig.choices and int(raw) in sig.choices:
            return str(sig.choices[int(raw)])
        return str(raw)
