# EV CAN Network Simulator

A simulated in-vehicle **CAN network for an electric vehicle**, written in Python with `python-can` and `cantools`.
Three ECUs (BMS, VCU, Brake) exchange periodic frames defined in a **DBC file**. A bus monitor checks
**E2E protection** (alive counter + CRC8) and **message timeouts**. A tester runs **UDS diagnostics over ISO-TP**
against the BMS.

Everything runs on a virtual CAN bus, so no hardware is needed. The same code can run on SocketCAN
(`vcan0`) or a real interface by changing the `interface`/`channel` arguments.

## Architecture

```
            ┌──────────┐   VCU_Drive 0x200 / 20 ms    ┌──────────┐
            │   VCU    │ ───────────────────────────► │   BMS    │◄── UDS 0x7E0 (tester)
            │ speed    │ ◄─── BRK_Status 0x300 / 50 ms│ SOC/temp │──► UDS 0x7E8
            └──────────┘                               │ DTCs     │
                 ▲                                     └──────────┘
                 │ BRK_Status                               │ BMS_Status 0x100 / 100 ms
            ┌──────────┐                                    ▼
            │  Brake   │        ┌─────────────────────────────────────────┐
            └──────────┘        │ BusMonitor: decode · CRC8 · counter ·   │
                                │ timeout watchdog · ASC + CSV logging    │
                                └─────────────────────────────────────────┘
```

Each ECU has its **own bus connection** and only learns about the others through CAN frames. For example,
the BMS computes pack current from the `VehicleSpeed`/`AccelPedal` it receives from the VCU, and the VCU
brakes using the `BrakePressure` it receives from the brake ECU.

## Features

| Area | What is implemented |
|---|---|
| **DBC** | `dbc/ev_network.dbc`: 3 messages, Intel byte order, signed signals, scale/offset, value tables (`BMS_State`, `Gear`), `GenMsgCycleTime` attributes |
| **E2E protection** | 4-bit `AliveCounter` and CRC-8 SAE J1850 `Checksum` (byte 7) on every frame |
| **Scheduling** | Deadline-based periodic TX threads. Measured cycle times match the DBC (20/50/100 ms) |
| **BMS model** | SOC by coulomb counting, OCV-based pack voltage, I²R thermal model, state machine (STANDBY → DRIVE → FAULT) |
| **Monitoring** | Checksum errors, counter jumps, timeouts (3 × cycle time) with recovery, temperature warning, BMS fault |
| **Diagnostics (UDS)** | `0x10` session control, `0x22` ReadDataByIdentifier, `0x19 02` ReadDTCByStatusMask, `0x14` ClearDTC (extended session only), `0x3E` TesterPresent, negative responses (`0x11`, `0x12`, `0x13`, `0x31`, `0x7F`) |
| **ISO-TP** | Single, First, Consecutive and Flow-Control frames. The 17-byte VIN is sent as a multi-frame response |
| **DTCs** | `P0A7E-00` battery over-temperature, `U0100-00` lost communication with VCU, with testFailed/confirmed status bits |
| **Logging** | `trace.asc` (Vector ASC format, opens in CANalyzer/CANoe), `signals.csv`, `events.csv` |
| **Tests** | 15 `pytest` tests: codec, CRC, ISO-TP framing, UDS services, and end-to-end fault scenarios |

## Quick start

```bash
pip install -r requirements.txt
python run_sim.py                          # normal drive cycle, 6 s
python run_sim.py --scenario overheat      # battery over-temperature -> FAULT + DTC P0A7E
python run_sim.py --scenario brake_loss    # brake ECU stops transmitting -> TIMEOUT
python run_sim.py --scenario bad_checksum  # 3 corrupted BMS frames -> CHECKSUM_ERROR + COUNTER_JUMP
python -m pytest -q
```

Example output (`--scenario overheat --duration 5`):

```
=== UDS session (tester 0x7E0 -> BMS 0x7E8) ===
  22 F190 VIN        : RLLVF8EV0TN000123
  22 0101 SOC        : 80.0 %
  22 0102 MaxCellTemp: 76 degC
  19 02 FF DTCs      : ['P0A7E-00 status=0x09']
  14 FFFFFF (default): NRC 0x7F (serviceNotSupportedInActiveSession) for SID 0x14
  10 03 + 14 FFFFFF  : cleared -> none

Cycle times (DBC vs measured):
  0x100 BMS_Status   100 ms  ->   100.0 ms  (52 frames)
  0x200 VCU_Drive     20 ms  ->    20.0 ms  (257 frames)
  0x300 BRK_Status    50 ms  ->    50.0 ms  (104 frames)
Events:
  t= 3.602s  TEMP_WARNING    BMS_Status  MaxCellTemp=55 degC
  t= 4.002s  BMS_FAULT       BMS_Status  BMS entered FAULT state
```

## Decoding logs from the C firmware

`tools/decode_log.py` decodes any log that python-can can read (candump `.log`, Vector `.asc`, …)
with the DBC, and checks the CRC and alive counter of every frame. It is used to validate the
`BMS_Status` frames produced by the companion **bms-firmware-c** project:

```bash
python tools/decode_log.py ../bms-firmware-c/bms_can.log
#   7.400  BMS_Status  SOC=79.0  PackVoltage=383.2  PackCurrent=0.0  MaxCellTemp=59  BMS_State=FAULT
#  13.000  BMS_Status  SOC=79.0  PackVoltage=383.2  PackCurrent=0.0  MaxCellTemp=40  BMS_State=STANDBY
#  150 frames decoded, 0 E2E errors
```

## Project layout

```
dbc/ev_network.dbc   CAN database
evcan/codec.py       DBC encode/decode + CRC8 / alive counter
evcan/ecu.py         ECU base class, VCU, Brake ECU, BMS (model, state machine, DTCs)
evcan/isotp.py       ISO 15765-2 transport layer
evcan/uds.py         UDS server (in BMS) and tester client
evcan/monitor.py     Bus monitor, timeout watchdog, ASC/CSV logging
run_sim.py           Scenario runner
tools/decode_log.py  Offline log decoder + E2E checker
tests/               pytest suite
```

## Running on Linux SocketCAN

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
candump vcan0   # in another terminal, to watch the traffic
```

Then pass `interface="socketcan", channel="vcan0"` to the ECUs, monitor and `UdsClient`.

## Notes

This is a learning project. The vehicle and battery models are simplified, and the UDS/ISO-TP stacks
implement only the subset listed above. There is no security access (0x27), no block-size handling and
no response-pending (NRC 0x78).
