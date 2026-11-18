"""Subset of UDS (ISO 14229) over ISO-TP: server (in the ECU) and tester client."""
from __future__ import annotations

import queue
import threading

import can

from .isotp import IsoTpChannel

REQ_ID, RESP_ID = 0x7E0, 0x7E8

# Service IDs
DSC, CLEAR_DTC, READ_DTC, READ_DID, TESTER_PRESENT = 0x10, 0x14, 0x19, 0x22, 0x3E
# Negative response codes
NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUBFUNC_NOT_SUPPORTED = 0x12
NRC_INCORRECT_LENGTH = 0x13
NRC_REQUEST_OUT_OF_RANGE = 0x31
NRC_NOT_IN_ACTIVE_SESSION = 0x7F
NRC_NAMES = {0x11: "serviceNotSupported", 0x12: "subFunctionNotSupported",
             0x13: "incorrectMessageLengthOrInvalidFormat", 0x31: "requestOutOfRange",
             0x7F: "serviceNotSupportedInActiveSession"}

DEFAULT_SESSION, EXTENDED_SESSION = 0x01, 0x03

# DTC status bits
TEST_FAILED, CONFIRMED = 0x01, 0x08


def dtc_to_str(code: int) -> str:
    """3-byte DTC (e.g. 0x0A7E00) -> 'P0A7E-00'."""
    hi, lo, ftb = (code >> 16) & 0xFF, (code >> 8) & 0xFF, code & 0xFF
    letter = "PCBU"[hi >> 6]
    return f"{letter}{(hi >> 4) & 0x3}{hi & 0xF:X}{lo:02X}-{ftb:02X}"


def _neg(sid: int, nrc: int) -> bytes:
    return bytes([0x7F, sid, nrc])


class DiagServer:
    """Runs in its own thread so multi-frame transfers don't block the ECU's receive path."""

    def __init__(self, bus: can.BusABC, dids: dict, dtcs: dict, lock: threading.Lock):
        self.dids, self.dtcs, self.lock = dids, dtcs, lock  # dids: {id: callable -> bytes}
        self.session = DEFAULT_SESSION
        self.rx_queue: "queue.Queue[can.Message]" = queue.Queue()
        self.tp = IsoTpChannel(bus, RESP_ID, REQ_ID, self._rx)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _rx(self, timeout):
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1)

    def _run(self):
        while not self._stop.is_set():
            req = self.tp.recv(timeout=0.1)
            if req:
                self.tp.send(self.handle(req))

    def handle(self, req: bytes) -> bytes:
        sid = req[0]
        if sid == DSC:
            if len(req) != 2:
                return _neg(sid, NRC_INCORRECT_LENGTH)
            if req[1] not in (DEFAULT_SESSION, EXTENDED_SESSION):
                return _neg(sid, NRC_SUBFUNC_NOT_SUPPORTED)
            self.session = req[1]
            return bytes([0x50, req[1], 0x00, 0x32, 0x01, 0xF4])  # P2=50ms, P2*=5000ms
        if sid == TESTER_PRESENT:
            return bytes([0x7E, 0x00])
        if sid == READ_DID:
            if len(req) != 3:
                return _neg(sid, NRC_INCORRECT_LENGTH)
            did = (req[1] << 8) | req[2]
            if did not in self.dids:
                return _neg(sid, NRC_REQUEST_OUT_OF_RANGE)
            return bytes([0x62, req[1], req[2]]) + self.dids[did]()
        if sid == READ_DTC:
            if len(req) != 3 or req[1] != 0x02:
                return _neg(sid, NRC_SUBFUNC_NOT_SUPPORTED if len(req) == 3 else NRC_INCORRECT_LENGTH)
            mask = req[2]
            out = bytearray([0x59, 0x02, 0xFF])
            with self.lock:
                for code, status in sorted(self.dtcs.items()):
                    if status & mask:
                        out += bytes([(code >> 16) & 0xFF, (code >> 8) & 0xFF, code & 0xFF, status])
            return bytes(out)
        if sid == CLEAR_DTC:
            if len(req) != 4:
                return _neg(sid, NRC_INCORRECT_LENGTH)
            if self.session != EXTENDED_SESSION:
                return _neg(sid, NRC_NOT_IN_ACTIVE_SESSION)
            with self.lock:
                self.dtcs.clear()
            return bytes([0x54])
        return _neg(sid, NRC_SERVICE_NOT_SUPPORTED)


class UdsNegativeResponse(Exception):
    def __init__(self, sid: int, nrc: int):
        self.sid, self.nrc = sid, nrc
        super().__init__(f"NRC 0x{nrc:02X} ({NRC_NAMES.get(nrc, '?')}) for SID 0x{sid:02X}")


class UdsClient:
    """Tester tool. Owns its own bus connection like a CANoe/diagnostic tester would."""

    def __init__(self, channel: str, interface: str = "virtual", timeout: float = 1.0):
        self.bus = can.Bus(interface=interface, channel=channel)
        self.bus.set_filters([{"can_id": RESP_ID, "can_mask": 0x7FF}])
        self.tp = IsoTpChannel(self.bus, REQ_ID, RESP_ID, self.bus.recv, timeout=timeout)
        self.timeout = timeout

    def request(self, payload: bytes) -> bytes:
        self.tp.send(payload)
        resp = self.tp.recv(timeout=self.timeout)
        if resp is None:
            raise TimeoutError(f"no response to SID 0x{payload[0]:02X}")
        if resp[0] == 0x7F:
            raise UdsNegativeResponse(resp[1], resp[2])
        if resp[0] != payload[0] + 0x40:
            raise RuntimeError(f"unexpected response {resp.hex()}")
        return resp

    def change_session(self, session: int) -> None:
        self.request(bytes([DSC, session]))

    def read_did(self, did: int) -> bytes:
        return self.request(bytes([READ_DID, did >> 8, did & 0xFF]))[3:]

    def read_dtcs(self, mask: int = 0xFF) -> list[tuple[str, int]]:
        body = self.request(bytes([READ_DTC, 0x02, mask]))[3:]
        return [(dtc_to_str(int.from_bytes(body[i:i + 3], "big")), body[i + 3]) for i in range(0, len(body), 4)]

    def clear_dtcs(self) -> None:
        self.request(bytes([CLEAR_DTC, 0xFF, 0xFF, 0xFF]))

    def close(self):
        self.bus.shutdown()
