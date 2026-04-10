# Smart Parking System — Basys3 FPGA

A hardware-based smart parking management system implemented in **Verilog HDL**, targeting the **Digilent Basys3 FPGA board** (Artix-7 XC7A35T-1CPG236C).

The system manages up to **8 parking slots** and **16 unique car IDs** in real time — handling entry, exit, fee calculation, and slot tracking entirely through on-board I/O (buttons, switches, LEDs, and a 7-segment display).

---

## Module Hierarchy

```
parking_system  (TOP)
├── debounce        × 3   (entry, exit, pay buttons)
├── time_tracker          (global tick + per-car entry timestamps)
├── parking_alu           (duration, fee, slot-finder — pipelined)
├── parking_memory        (car status + slot occupancy map)
├── control_unit          (FSM — transaction coordinator)
└── seg7_display          (4-digit multiplexed display driver)
```

---

## Basys3 I/O Mapping

| Signal | Pin | Component | Description |
|---|---|---|---|
| `clk` | W5 | Oscillator | 100 MHz system clock |
| `rst` | U18 | BTNC | Synchronous reset |
| `btn_entry_raw` | T18 | BTNU | Car entry request |
| `btn_exit_raw` | W19 | BTNL | Car exit request |
| `btn_pay_raw` | T17 | BTNR | Payment confirmation |
| `sw[3:0]` | V17–W17 | SW3–SW0 | 4-bit car ID input |
| `led[7:0]` | U16–V14 | LD7–LD0 | Slot occupancy (`1` = occupied) |
| `seg[6:0]` | W7–U7 | 7-seg | Cathodes (active-low) |
| `an[3:0]` | U2–W4 | 7-seg | Digit anodes (active-low) |

---

## Display Modes

| Mode | FSM State(s) | Display |
|---|---|---|
| `00` | `IDLE`, `ERROR`, `FREE_SLOT` | `Fr N` — free slot count (0–8) |
| `01` | `READ_ID` → `ASSIGN`, `EXIT_REQUEST` | `CA X` — car ID being processed |
| `10` | `CALC_DURATION` | `NNNN` — parking duration in seconds |
| `11` | `CALC_FEE`, `WAIT_PAYMENT` | `NNNN` — fee amount |

---

## FSM State Diagram

### Entry Flow
```
IDLE → READ_ID → CHECK_ENTRY → CHECK_FULL → FIND_SLOT → ASSIGN_SLOT → START_TIMER → IDLE
```

### Exit Flow
```
IDLE → EXIT_REQUEST → CHECK_EXIT → CALC_DURATION → CALC_FEE → WAIT_PAYMENT → FREE_SLOT → IDLE
```

> Any invalid condition routes to `ERROR` (one cycle) before returning to `IDLE`.

The FSM is **transaction-based** — it returns to `IDLE` after each operation, allowing multiple cars to be parked simultaneously via memory.

---

## Key Parameters

| Parameter | Value | Description |
|---|---|---|
| `NUM_SLOTS` | `8` | Parking slots (matches 8 LEDs) |
| `NUM_CARS` | `16` | Supported car IDs (4-bit) |
| `CLK_FREQ` | `100_000_000` | 100 MHz Basys3 oscillator |
| `TICK_DIV` | `1_000_000_000` | 1 tick = 10 seconds |
| `FEE_RATE` | `1` unit/tick | 6 units/min, 360 units/hour |
| `EMPTY_ID` | `4'hF` | Reserved ID — not assignable |

---

## Error Codes

| Error | Code | Cause |
|---|---|---|
| `ERR_DUP_ENTRY` | `1` | Car already inside |
| `ERR_FULL` | `2` | All 8 slots occupied |
| `ERR_NO_ENTRY` | `3` | Exit attempted without prior entry |
| `ERR_INVALID_ID` | `4` | Car ID is `0xF` (reserved) |
| `ERR_OVERFLOW` | `5` | Timer overflow detected at entry |

---

## Getting Started

### Requirements
- [Vivado Design Suite](https://www.xilinx.com/products/design-tools/vivado.html) (2020.x or later recommended)
- Digilent Basys3 board

### Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/Arefin-Nahid/Parking-System-BASIS3
   cd smart-parking-system
   ```

2. **Open Vivado** and create a new RTL project

3. **Add sources** — add all `.v` files as design sources

4. **Add constraints** — add `parking_system.xdc` as a constraint file

5. **Set top module** to `parking_system`

6. **Run** Synthesis → Implementation → Generate Bitstream

7. **Program** the Basys3 board via Vivado Hardware Manager

---

## How to Use

| Action | Steps |
|---|---|
| **Park a car** | Set car ID on `SW[3:0]` → Press **BTNU** (Entry) |
| **Exit a car** | Set car ID on `SW[3:0]` → Press **BTNL** (Exit) → Press **BTNR** (Pay) |
| **View free slots** | Check 7-segment display (`Fr N`) when idle |
| **View occupancy** | Check LEDs — each LED = one slot (`1` = occupied) |
| **Reset system** | Press **BTNC** at any time |

---

## License

This project is licensed under the [MIT License](LICENSE).
