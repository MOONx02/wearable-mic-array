# Bill of materials: 2-mic bench rig (XIAO ESP32-S3 Sense)

What the working 2-mic direction-finding rig is built from. Prices are approximate list prices; check before ordering.

| # | Item | Part / spec | Qty | Approx. unit price | Approx. total | Notes |
|---|---|---|---|---|---|---|
| 1 | Microcontroller | Seeed Studio XIAO ESP32-S3 Sense (113991115) | 1 | $14 | $14 | Includes the camera/mic/microSD expansion board. |
| 2 | PDM microphone | Adafruit PDM MEMS Microphone Breakout #3492 (ST MP34DT01-M) | 2 | $4.95 | $9.90 | Both share one clock (D1) and one data line (D0); one SEL to GND, one to 3V3. |
| 3 | Breadboard | Solderless breadboard, half or full size | 1 | $5 | $5 | |
| 4 | Jumper wires | Male-to-male, assorted | ~10 | — | $3 | 3V3, GND, CLK, DAT to each mic. |
| 5 | Header pins | 0.1" male headers | — | — | (included) | Ship with the mic boards and XIAO; must be soldered. |
| 6 | USB cable | USB-C data cable (not charge-only) | 1 | $5 | $5 | Powers the board and streams audio to the laptop. |
| | | | | | **≈ $37** | |

## Not needed for the bench rig

- Battery: the rig runs from USB. A 3.7 V LiPo on the XIAO's BAT pads is only for going wireless later.
- Clock resistor (33–100 Ω at the clock pin): skipped for the short bench wires. Add it for the 8-mic shirt build.

## Software (free)

- Arduino CLI with the ESP32 core 3.3.5. Firmware: `two_mic_stream/two_mic_stream.ino`.
- Python 3.9+. The viewer `mic_live_view.py` installs numpy, PySide6 and pyserial on first run.
