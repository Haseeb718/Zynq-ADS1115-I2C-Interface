# ADS1115 I2C ADC Reader — Zynq-7010

A Verilog implementation of a 16-bit ADC readout system on the **Zynq-7010 FPGA**, reading from an **ADS1115** over I2C and outputting results via UART and ILA debug probe.

> **Ported from** the original iCE40/iCEBreaker implementation by [holla2040](https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker/blob/main/src/adc-read-i2c/i2c_master.v). See [Attribution](#attribution) for full details.

---

## Overview

- Reads 16-bit ADC value from ADS1115 over I2C every **200 ms**
- Outputs result as `0xNNNN\r\n` over **UART at 115200 baud**
- ADC value visible directly on **Vivado ILA** probe (`dbg_adc_value[15:0]`)
- ADC value can be passed to **Zynq PS** via AXI register for C processing
- **UART is optional** — remove `uart_tx` instantiation if not needed, ILA and PS access continue to work

---

## Hardware Requirements

| Component | Details |
|---|---|
| FPGA Board | Zynq-7010 |
| ADC | ADS1115 16-bit I2C ADC breakout |
| Pull-up Resistors | 2 × 4.7 kΩ — SCL and SDA to 3.3 V (**mandatory**) |
| Potentiometer | 10 kΩ recommended, wiper to AIN0 |
| UART | On-board USB-UART or external (115200 baud) |

> ⚠️ **Pull-up resistors are required.** The I2C bus is open-drain and cannot drive lines HIGH without them. Without pull-ups `scl_in` and `sda_in` will read 0 at all times and all ACKs will appear false.

---

## Wiring

| Signal | FPGA Pin | ADS1115 Pin |
|---|---|---|
| SCL | See XDC file | SCL |
| SDA | See XDC file | SDA |
| 3.3 V | 3V3 header | VDD |
| GND | GND header | GND + ADDR (tie ADDR to GND for address 0x48) |
| AIN0 | — | Potentiometer wiper |
| UART TX | See XDC file | USB-UART RX |

---

## Repository Structure

```
ads1115-zynq/
├── README.md
├── rtl/
│   ├── top.v              # Top-level state machine
│   ├── i2c_master.v       # I2C master (Xilinx IOBUF open-drain)
│   └── uart_tx.v          # UART transmitter (optional)
├── constraints/
│   └── pins.xdc           # Pin assignments and pull-up settings
├── software/
│   └── adc_read.c         # Zynq PS C code to read ADC value
└── docs/
    ├── ADS1115_Zynq_Project.pdf
    └── ads1115_datasheet.pdf   # Download from ti.com/product/ADS1115
```

---

## ADS1115 Configuration

The config register `0xC2C3` is written once at startup:

| Bits | Field | Value | Meaning |
|---|---|---|---|
| 14:12 | MUX | 100 | AIN0 single-ended vs GND |
| 11:9 | PGA | 001 | ±4.096 V full scale |
| 8 | MODE | 0 | Continuous conversion |
| 7:5 | DR | 110 | 250 SPS |

---

## Building in Vivado

1. **Create Project** — select Zynq-7010 board part
2. **Add Sources** — add all files from `rtl/`
3. **Add Constraints** — add `constraints/pins.xdc`
4. **Add ILA** — in block design, connect probes:
   - `dbg_state[4:0]`
   - `dbg_scl_in`, `dbg_scl_oe`
   - `dbg_sda_in`, `dbg_sda_oe`
   - `i2c_ack`
   - `dbg_adc_value[15:0]`
5. **Run Synthesis → Implementation → Generate Bitstream**
6. **Program Device** via Hardware Manager

---

## Reading ADC Value

### Option 1 — UART (easiest)
Open any serial terminal at **115200 baud**. On boot you will see:
```
ads1115
0x67A0
0x67B2
0x67A8
...
```

### Option 2 — ILA Probe
`dbg_adc_value[15:0]` updates every 200 ms. View directly in Vivado waveform window.

### Option 3 — Zynq PS (C code)
```c
#include <stdint.h>
#include "xil_io.h"

while (1) {
    // Cast to int16_t — ADS1115 is SIGNED 16-bit
    int16_t adc = (int16_t)(Xil_In32(XPAR_ADC_VAL_BASEADDR) & 0xFFFF);

    // PGA = ±4.096 V, full scale = 32767 counts
    float voltage = ((float)adc / 32767.0f) * 4.096f;

    printf("ADC: %d   Voltage: %.4f V\r\n", adc, voltage);
    usleep(200000);
}
```

> ⚠️ Always divide by **32767** (not 65535) and multiply by **4.096** (not 3.3 or 5). The ADS1115 is a signed 16-bit device with PGA-defined full scale.

---

## UART is Optional

If you do not need serial output, remove the `uart_tx` instantiation and all `uart_*` signals from `top.v`. The I2C state machine, ILA probes, and PS-side register access all continue to work unchanged.

| Method | Requires UART? |
|---|---|
| ILA probe (`dbg_adc_value`) | No |
| Zynq PS via AXI register | No |
| Serial terminal output | Yes |

---

## Voltage Conversion

```
voltage = (adc_signed / 32767.0) × 4.096 V
```

| ADC Count | Voltage |
|---|---|
| 32767 | 4.096 V (positive full scale) |
| ~26672 | 3.33 V (potentiometer at max) |
| 0 | 0.000 V |
| -48 | -0.006 V (noise floor — normal) |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `scl_in` / `sda_in` stuck at 0 | No pull-up resistors | Add 4.7 kΩ to 3.3 V or enable `PULLUP` in XDC |
| UART shows `0x0000` always | No pull-ups or no device | Expected without hardware |
| Voltage reads ~half expected | Dividing by 65535 | Divide by 32767 |
| Voltage reads wrong scale | Multiplying by 3.3 or 5 | Multiply by 4.096 |
| Jumps to 8 V near 0 V | No `int16_t` cast in C | Cast `Xil_In32` result to `int16_t` |
| 28 SCL pulses per read | Not a bug | 1 START + 9 addr + 9 MSB + 9 LSB = 28 |
| Small negative values near 0 V | ADC noise floor | Normal — no fix needed |

---

## State Machine Reference

| Hex | State | Description |
|---|---|---|
| 0x00 | ST_STARTUP | Send startup message via UART |
| 0x01–0x06 | ST_CFG_* | Write ADS1115 config register (0xC2C3) |
| 0x07–0x0A | ST_PTR_* | Set pointer to conversion register |
| 0x0B | ST_IDLE | Wait for 200 ms interval tick |
| 0x0C–0x10 | ST_RD_* | Read 16-bit ADC value over I2C |
| 0x11 | ST_SEND_ADC | Send hex string via UART |
| 0x12 | ST_ERROR | NACK received — send 'E', return to IDLE |

---

## Attribution

This project is a port of the original iCE40/iCEBreaker implementation:

- **Original Author:** [holla2040](https://github.com/holla2040)
- **Original Repository:** [Agentic_Verilog_iCE40_iCEBreaker](https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker)
- **Original File:** `src/adc-read-i2c/i2c_master.v`

### Changes Made for Zynq-7010

- Replaced iCE40 `SB_IO` primitives with Xilinx `IOBUF` for open-drain I2C
- Updated `HALF_PERIOD` = 250 (50 MHz / 100 kHz / 2)
- Widened timer from `[6:0]` to `[8:0]`
- Updated UART `CLOCKS_PER_BIT` = 434 (50 MHz / 115200)
- Widened `baud_counter` from `[7:0]` to `[9:0]`
- Updated interval counter to 10,000,000 clocks (200 ms at 50 MHz)
- Widened `interval_counter` from `[23:0]` to `[24:0]`
- Removed iCEBreaker-specific `btn_addr` / `ads_addr`
- Added ILA debug ports: `dbg_scl_in`, `dbg_sda_in`, `dbg_scl_oe`, `dbg_sda_oe`, `dbg_state`, `dbg_adc_value`

---

## References

- [ADS1115 Datasheet — Texas Instruments](https://www.ti.com/product/ADS1115) (SBAS444)
- [Original iCE40 Source — holla2040](https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker)
- Full project documentation: `docs/ADS1115_Zynq_Project.pdf`
