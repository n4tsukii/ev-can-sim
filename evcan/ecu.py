"""Simulated ECUs. Each ECU owns its own bus connection and only talks to others through CAN frames."""
from __future__ import annotations

import sys
import threading
import time

import can

from .codec import Codec
from .uds import CONFIRMED, REQ_ID, TEST_FAILED, DiagServer

DTC_PACK_OVERTEMP = 0x0A7E00   # P0A7E-00 Hybrid/EV battery pack over temperature
DTC_LOST_COMM_VCU = 0xC10000   # U0100-00 Lost communication with VCU

if sys.platform == "win32":
    # Windows timers default to ~15.6 ms resolution, too coarse for a 20 ms cycle. Request 1 ms.
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)

BMS_INIT, BMS_STANDBY, BMS_DRIVE, BMS_CHARGING, BMS_FAULT = range(5)


class ECU:
    def __init__(self, name: str, codec: Codec, channel: str, interface: str = "virtual"):
        self.name, self.codec = name, codec
        self.bus = can.Bus(interface=interface, channel=channel)
        self.tx_enabled = True       # set False to simulate an ECU dropping off the bus
        self.corrupt_next = 0        # number of upcoming frames sent with a wrong checksum
        self._counters: dict[str, int] = {}
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._notifier: can.Notifier | None = None
        self.t0 = time.time()

    # --- tx ---
    def send(self, msg_name: str, signals: dict) -> None:
        if not self.tx_enabled:
            return
        cnt = self._counters.get(msg_name, 0)
        self._counters[msg_name] = (cnt + 1) & 0x0F
        corrupt = self.corrupt_next > 0
        if corrupt:
            self.corrupt_next -= 1
        data = self.codec.encode(msg_name, signals, cnt, corrupt=corrupt)
        self.bus.send(can.Message(arbitration_id=self.codec.frame_id(msg_name), data=data, is_extended_id=False))

    def add_periodic(self, msg_name: str, build) -> None:
        period = self.codec.cycle_time_s(msg_name)

        def loop():
            next_t = time.time()
            while not self._stop.is_set():
                self.send(msg_name, build(period))
                next_t += period                      # deadline-based, no drift
                self._stop.wait(max(0.0, next_t - time.time()))
        self._threads.append(threading.Thread(target=loop, name=f"{self.name}-{msg_name}", daemon=True))

    # --- rx ---
    def on_message(self, msg: can.Message) -> None:  # override
        pass

    def start(self):
        self.t0 = time.time()
        self._notifier = can.Notifier(self.bus, [self.on_message], timeout=0.05)
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1)
        if self._notifier:
            self._notifier.stop()
        self.bus.shutdown()

    def elapsed(self) -> float:
        return time.time() - self.t0


def driver_profile(t: float) -> tuple[float, float]:
    """Repeating 10 s drive cycle -> (accelerator %, brake bar)."""
    t = t % 10.0
    if t < 4:
        return 60.0, 0.0
    if t < 7:
        return 25.0, 0.0
    if t < 9:
        return 0.0, 40.0
    return 0.0, 0.0


class VCU(ECU):
    """Vehicle control unit: integrates speed from pedal and brake pressure received from BRK."""

    def __init__(self, codec, channel, **kw):
        super().__init__("VCU", codec, channel, **kw)
        self.speed_kmh = 0.0
        self.brake_bar = 0.0
        self.add_periodic("VCU_Drive", self._build)

    def on_message(self, msg):
        if msg.arbitration_id == self.codec.frame_id("BRK_Status") and self.codec.checksum_ok(msg.data):
            self.brake_bar = self.codec.decode(msg.arbitration_id, msg.data)["BrakePressure"]

    def _build(self, dt):
        pedal, _ = driver_profile(self.elapsed())
        v = self.speed_kmh / 3.6
        accel = pedal / 100 * 3.0 - self.brake_bar / 40 * 6.0 - 0.05 * v
        self.speed_kmh = min(250.0, max(0.0, (v + accel * dt) * 3.6))
        return {"VehicleSpeed": round(self.speed_kmh, 2), "AccelPedal": pedal, "Gear": 3}


class BrakeECU(ECU):
    def __init__(self, codec, channel, **kw):
        super().__init__("BRK", codec, channel, **kw)
        self.add_periodic("BRK_Status", self._build)

    def _build(self, dt):
        _, brake = driver_profile(self.elapsed())
        return {"BrakePressure": brake, "BrakePedalPressed": int(brake > 0)}


class BMS(ECU):
    """Battery management system with a simple electro-thermal model, fault handling and a UDS server."""

    N_CELLS = 96
    CAPACITY_AS = 100 * 3600     # 100 Ah
    R_PACK = 0.08                # ohm
    OVERTEMP_C = 60.0
    VCU_TIMEOUT_S = 0.2          # 10x VCU cycle time

    def __init__(self, codec, channel, **kw):
        super().__init__("BMS", codec, channel, **kw)
        self.soc = 80.0
        self.temp_c = 25.0
        self.current_a = 0.0
        self.state = BMS_STANDBY
        self.speed_kmh = 0.0
        self.pedal = 0.0
        self.last_vcu = None
        self.overheat_rate = 0.0     # extra degC/s for fault injection
        self.lock = threading.Lock()
        self.dtcs: dict[int, int] = {}
        self.diag = DiagServer(self.bus, {
            0xF190: lambda: b"RLLVF8EV0TN000123",              # VIN (17 bytes -> multi-frame)
            0x0101: lambda: bytes([int(round(self.soc * 2))]),  # SOC, 0.5 %/bit
            0x0102: lambda: bytes([int(round(self.temp_c + 40))]),
            0x0103: lambda: bytes([self.state]),
        }, self.dtcs, self.lock)
        self.add_periodic("BMS_Status", self._build)

    def inject_overheat(self, rate_c_per_s: float = 15.0):
        self.overheat_rate = rate_c_per_s

    def _set_dtc(self, code: int, failed: bool):
        with self.lock:
            if failed:
                self.dtcs[code] = TEST_FAILED | CONFIRMED
            elif code in self.dtcs:
                self.dtcs[code] &= ~TEST_FAILED     # keep confirmed (history) bit

    def on_message(self, msg):
        if msg.arbitration_id == REQ_ID:
            self.diag.rx_queue.put(msg)
        elif msg.arbitration_id == self.codec.frame_id("VCU_Drive") and self.codec.checksum_ok(msg.data):
            sig = self.codec.decode(msg.arbitration_id, msg.data)
            self.speed_kmh, self.pedal = sig["VehicleSpeed"], sig["AccelPedal"]
            self.last_vcu = time.time()

    def _build(self, dt):
        # electrical model
        if self.pedal > 0:
            self.current_a = self.pedal * 2.5 + self.speed_kmh * 0.3 + 2.0
        elif self.speed_kmh > 5:
            self.current_a = -self.speed_kmh * 0.4   # regen
        else:
            self.current_a = 2.0                     # auxiliaries
        if self.state == BMS_FAULT:
            self.current_a = 0.0                     # contactors open
        self.soc = min(100.0, max(0.0, self.soc - self.current_a * dt / self.CAPACITY_AS * 100))
        ocv_cell = 3.0 + 1.2 * self.soc / 100
        pack_v = self.N_CELLS * ocv_cell - self.current_a * self.R_PACK
        # thermal model
        self.temp_c += (self.current_a ** 2 * 2e-5 - (self.temp_c - 25.0) * 0.01 + self.overheat_rate) * dt
        self.temp_c = min(125.0, self.temp_c)

        # diagnostics
        vcu_lost = self.last_vcu is not None and time.time() - self.last_vcu > self.VCU_TIMEOUT_S
        self._set_dtc(DTC_LOST_COMM_VCU, vcu_lost)
        overtemp = self.temp_c >= self.OVERTEMP_C
        self._set_dtc(DTC_PACK_OVERTEMP, overtemp)

        # state machine
        if overtemp:
            self.state = BMS_FAULT                   # latched until restart
        elif self.state != BMS_FAULT:
            self.state = BMS_DRIVE if (self.speed_kmh > 0 or self.pedal > 0) else BMS_STANDBY

        return {"SOC": round(self.soc * 2) / 2, "PackVoltage": round(pack_v, 1),
                "PackCurrent": round(self.current_a, 1), "MaxCellTemp": round(self.temp_c),
                "BMS_State": self.state}

    def start(self):
        super().start()
        self.diag.start()

    def stop(self):
        self.diag.stop()
        super().stop()
