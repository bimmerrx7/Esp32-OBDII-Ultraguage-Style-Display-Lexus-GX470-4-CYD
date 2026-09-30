# GX470 UltraGauge — ESP32 OBD-II Display

A DIY, UltraGauge-style OBD-II gauge display built around an ESP32 and a
4" touchscreen. Connects over Bluetooth Classic to an ELM327-style
adapter and shows live engine data across several swipeable gauge pages.

Built and tested on a **2007 Lexus GX470** (1GR-FE V8), but the standard
OBD-II PIDs will work on most OBD-II-compliant vehicles (2008+ in the
US, most earlier ones too). A couple of gauges use Toyota-specific
enhanced PIDs and won't apply to other makes — see Notes below.

## Features

- Six gauge pages, laid out like a UltraGauge/ScanGauge:
  - **ENGINE** — RPM, speed, coolant, throttle, trans pan/TC temp
  - **MILEAGE** — instant MPG, speed, MAF, engine load
  - **VITALS** — RPM, speed, coolant, trans temp, battery, intake temp, load, throttle
  - **TRANS** — full-screen transmission pan temp with a bar gauge
  - **TRIMS** — short/long fuel trim on both banks, plus upstream O2 (wideband AFR) on both banks
  - **EXTRA** — timing advance, module voltage, barometric pressure, engine runtime, distance since codes cleared
  - **DIAG** — read and clear diagnostic trouble codes (DTCs)
- Tap anywhere to advance to the next page (like a single button press); swipe left/right also works
- Per-gauge alarms — values flash red when outside a configured safe range
- Only polls the PIDs the current page actually needs, so fewer gauges on screen = faster updates
- Touch calibration runs automatically on first boot and is saved to flash

## Hardware

- ESP32 dev board (any variant with Bluetooth Classic / SPP support)
- 4.0" 320x480 TFT display, ST7796S driver, resistive touch (XPT2046) — e.g. a Hosyond 4" display
- An ELM327-compatible Bluetooth OBD-II adapter (tested with a Vgate Scan Advanced; most genuine — non-clone — ELM327 adapters should work)

## Setup

1. **Arduino IDE**: install the ESP32 board package, and the **TFT_eSPI**
   library via Library Manager. BluetoothSerial and Preferences ship
   with the ESP32 board package.
2. **TFT_eSPI configuration**: edit TFT_eSPI's `User_Setup.h` for the
   `ST7796_DRIVER`, your display's wiring/pins, and `TOUCH_CS`.
3. **Save the sketch** in a folder with the same name as the file:
   `esp32_gx470_ultragauge/esp32_gx470_ultragauge.ino`
4. **Find your ELM327 adapter's MAC address and PIN** — this sketch
   connects by MAC address, not by name, so it won't find your adapter
   until you do this:
   - Flash `bt_scanner.ino` (included in this repo) to your ESP32 and
     open Serial Monitor. It lists nearby Bluetooth Classic devices with
     their names and MAC addresses.
   - Find your adapter in that list and copy its MAC address.
   - In `esp32_gx470_ultragauge.ino`, set `elmAddress` to that MAC
     (in the `{ 0x.., 0x.., ... }` format already there).
   - Set `elmPin` to your adapter's PIN. `1234` and `0000` are the most
     common defaults; some (like OBDLink) use `6789` — check your
     adapter's manual/listing if pairing fails.
5. **Set your units and orientation** at the top of the sketch:
   `USE_IMPERIAL` (mph/°F/MPG vs km/h/°C/km-L) and `SCREEN_ROTATION`
   (to match how you're mounting the display). Changing the rotation
   triggers one automatic touch recalibration on the next boot.
6. Flash the sketch, power up with the adapter plugged into the OBD-II
   port and the key on, and it should connect automatically.

## Notes / limitations

- Transmission pan/torque-converter temps use Toyota's enhanced Mode 21
  PID (`21D9`), not a standard PID — this is Toyota/Lexus-specific and
  won't work on other makes.
- A few gauges that seemed like they should exist (ambient air temp,
  fuel level, fuel rate) were removed after testing showed this
  particular vehicle's ECU doesn't expose those over standard OBD-II at
  all. Your vehicle may differ — check your own PID support with
  `01 00`, `01 20`, and `01 40` (mode 1, PID support bitmaps) in a
  terminal before assuming a gauge should work.
- Instant MPG is calculated from MAF + speed assuming gasoline at a
  14.7:1 air/fuel ratio; not exact, but a reasonable live estimate.

## License

MIT — do whatever you'd like with it.
