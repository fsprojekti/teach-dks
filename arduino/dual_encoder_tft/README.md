# Dual Encoder + TFT (ESP32)

This sketch reads two quadrature encoders and displays live values on TFT.

## Files
- `dual_encoder_tft.ino`

## What it shows
- Magnetic encoder: count, angle (deg), rpm
- Optical encoder: count, motor rpm, output rpm

## Defaults
- Magnetic encoder: `4096` counts/rev
- Optical encoder: `48` counts/motor rev
- Gear ratio: `20.4:1`

## Pin mapping (change in sketch if needed)
- `PIN_MAG_A = 32`
- `PIN_MAG_B = 33`
- `PIN_OPT_A = 36`
- `PIN_OPT_B = 37`

## Required library
- `TFT_eSPI`

Make sure `TFT_eSPI` is configured for your LilyGO TFT in the library setup.

## Upload
1. Open `dual_encoder_tft.ino` in Arduino IDE.
2. Select your ESP32 board and COM port.
3. Install `TFT_eSPI` library if not installed.
4. Verify/adjust pin constants.
5. Upload.
