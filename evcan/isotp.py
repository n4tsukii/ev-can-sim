"""Minimal ISO 15765-2 (ISO-TP) transport: single, first, consecutive and flow-control frames."""
from __future__ import annotations

import time
from typing import Callable, Optional

import can

PAD = 0xCC
SF, FF, CF, FC = 0x0, 0x1, 0x2, 0x3
MAX_PAYLOAD = 4095


def _pad(frame: bytes) -> bytes:
    return frame + bytes([PAD] * (8 - len(frame)))


def segment(payload: bytes) -> list[bytes]:
    """Split a payload into ISO-TP frames. Index 0 is SF or FF, the rest are CFs."""
    n = len(payload)
    if n == 0 or n > MAX_PAYLOAD:
        raise ValueError(f"payload length {n} not supported")
    if n <= 7:
        return [_pad(bytes([n]) + payload)]
    frames = [bytes([0x10 | (n >> 8), n & 0xFF]) + payload[:6]]
    seq, pos = 1, 6
    while pos < n:
        frames.append(_pad(bytes([0x20 | (seq & 0x0F)]) + payload[pos:pos + 7]))
        seq += 1
        pos += 7
    return frames


def flow_control(block_size: int = 0, st_min_ms: int = 0) -> bytes:
    return _pad(bytes([0x30, block_size, st_min_ms]))


class IsoTpError(Exception):
    pass


RxFn = Callable[[float], Optional[can.Message]]


class IsoTpChannel:
    """One direction pair (tx_id -> peer, rx_id <- peer). `rx` returns the next frame for rx_id or None."""

    def __init__(self, bus: can.BusABC, tx_id: int, rx_id: int, rx: RxFn, timeout: float = 1.0):
        self.bus, self.tx_id, self.rx_id, self._rx, self.timeout = bus, tx_id, rx_id, rx, timeout

    def _send_raw(self, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=self.tx_id, data=data, is_extended_id=False))

    def _next(self, deadline: float) -> can.Message:
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise IsoTpError("timeout")
            msg = self._rx(remaining)
            if msg is not None and msg.arbitration_id == self.rx_id:
                return msg

    def send(self, payload: bytes) -> None:
        frames = segment(payload)
        self._send_raw(frames[0])
        if len(frames) == 1:
            return
        fc = self._next(time.time() + self.timeout)
        if fc.data[0] >> 4 != FC:
            raise IsoTpError("expected flow control")
        st_min = fc.data[2] / 1000.0 if fc.data[2] <= 0x7F else 0.0
        for frame in frames[1:]:
            if st_min:
                time.sleep(st_min)
            self._send_raw(frame)

    def recv(self, timeout: float) -> Optional[bytes]:
        try:
            first = self._next(time.time() + timeout)
        except IsoTpError:
            return None
        pci = first.data[0] >> 4
        if pci == SF:
            n = first.data[0] & 0x0F
            return bytes(first.data[1:1 + n])
        if pci != FF:
            return None  # stray CF/FC, ignore
        total = ((first.data[0] & 0x0F) << 8) | first.data[1]
        buf = bytearray(first.data[2:8])
        self._send_raw(flow_control())
        expected = 1
        while len(buf) < total:
            cf = self._next(time.time() + self.timeout)
            if cf.data[0] >> 4 != CF:
                raise IsoTpError("expected consecutive frame")
            if cf.data[0] & 0x0F != expected & 0x0F:
                raise IsoTpError("sequence number error")
            buf.extend(cf.data[1:8])
            expected += 1
        return bytes(buf[:total])
