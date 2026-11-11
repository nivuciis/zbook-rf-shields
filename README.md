# zbook-rf-shields

RF shields for the [ZBook](https://github.com/zephyr-book/zbook) (RP2350B)
development board: Altium hardware and Zephyr RTOS firmware.

| Branch                       | Content                                  |
|------------------------------|------------------------------------------|
| `zbook-rf-shields-lora-hw`   | LoRa shield PCB (RFM95W)                 |
| `zbook-rf-shields-ble-hw`    | BLE shield PCB (CH592X)                  |
| `zbook-rf-shields-lora-fw`   | Zephyr firmware for the LoRa shield      |

## Repository layout

```
apps/
  rfm95_spi_check/        SPI link check with the RFM95W (no RF transmission)
boards/shields/
  zbook_lora/             "zbook_lora" shield: RFM95W on the ZBook 2x7 header
zephyr/module.yml         Makes this repository a Zephyr module (exposes boards/shields)
west.yml                  Optional manifest for a standalone workspace
```

## Hardware

The RFM95W sits on the ZBook P2 2x7 expansion header, on `spi1`, shared with
the on-board BMI323 IMU (each device has its own chip select).

| ZBook header | GPIO | RFM95W pin | Function |
|--------------|------|------------|----------|
| +3V3         | -    | 13 (3.3V)  | Power (**3.3 V only, never +5V**) |
| GND          | -    | 1, 8, 10   | Ground |
| MOSI         | GP11 | 3 (MOSI)   | SPI data in |
| MISO         | GP8  | 2 (MISO)   | SPI data out |
| SCLK         | GP10 | 4 (SCK)    | SPI clock |
| IO45         | GP45 | 5 (NSS)    | SPI chip select |
| IO01         | GP1  | 6 (RESET)  | Reset |
| IO4x         | GP46 | 14 (DIO0)  | TxDone/RxDone IRQ (*pin to be confirmed*) |
| IO39         | GP39 | 15 (DIO1)  | RxTimeout IRQ |

RFM95W pin numbers follow the HopeRF datasheet (top view, metal shield up).
SPI wiring is straight (MOSI to MOSI, MISO to MISO). Never transmit without an
antenna on pin 9 (ANT).

## Getting started

### Requirements

- A Zephyr workspace with the Zephyr SDK installed (tested with Zephyr 4.5.99
  and SDK 1.0.1)
- [probe-rs](https://probe.rs) to flash through the ZBook **PROG** USB port

### Setup (existing Zephyr workspace)

From the workspace root (for example `~/zephyrproject`):

```powershell
git clone https://github.com/zephyr-book/zbook.git modules/zbook
git clone -b zbook-rf-shields-lora-fw https://github.com/nivuciis/zbook-rf-shields.git
```

Register both as extra Zephyr modules for the current terminal session:

```powershell
# PowerShell
$env:ZEPHYR_EXTRA_MODULES = "$HOME/zephyrproject/modules/zbook;$HOME/zephyrproject/zbook-rf-shields"
```

```bash
# bash
export ZEPHYR_EXTRA_MODULES="$HOME/zephyrproject/modules/zbook;$HOME/zephyrproject/zbook-rf-shields"
```

### Build, flash and monitor

```powershell
west build -p -b zbook/rp2350b/m33 --shield zbook_lora zbook-rf-shields/apps/rfm95_spi_check
west flash --runner probe-rs
python -m serial.tools.miniterm COMx 115200
```

The console is the RP2350 UART0, bridged to a virtual COM port by the on-board
RP2040 behind the **PROG** USB port.

## Applications

### `rfm95_spi_check`

Reads the SX1276 `RegVersion` register (0x42) in batches of 20000 and reports
how many reads returned a valid silicon revision (`0x11` per the HopeRF
datasheet, `0x12` for Semtech production silicon V1b). The Zephyr LoRa driver
is disabled in this app and nothing is transmitted.

Expected output:

```
<inf> app: OK 20000 / 20000   (last value 0x12)
```

| Result                          | Likely cause |
|---------------------------------|--------------|
| `0x00` on every read            | Module not powered, or NSS not reaching it |
| `0xff` on every read            | MISO not connected, or MOSI/MISO crossed |
| Random values, OK count < 20000 | Bad solder joints, long wires or weak ground |

## Status

- [x] SPI link validated (`RegVersion = 0x12`, 100 kHz)
- [ ] RESET and DIO0 wired, Zephyr LoRa driver (`semtech,sx1276`) running
- [ ] LoRa point-to-point TX/RX with a second device

## License

Apache-2.0. See [LICENSE](LICENSE).