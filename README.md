# ESP32 Red Object Tracker

An ESP32-CAM prototype that detects red pixels, estimates the center of the detected red region, and reports its offset from the center of the image over the Serial Monitor/UART.

## Current behavior

- Captures 160 × 120 frames in RGB565 format.
- Converts every RGB565 pixel into 8-bit red, green, and blue values.
- Classifies a pixel as red when red is at least 100 and at least 40% stronger than both green and blue.
- Marks a target as valid when red pixels cover at least 3% of the frame.
- Calculates the centroid of all qualifying red pixels.
- Reports horizontal and vertical error relative to the image center.

## Serial output

The board prints one record per analyzed frame at 115200 baud:

```text
TRACK,valid,error_x_px,error_y_px,red_pixel_count
```

Example:

```text
TRACK,1,-12.45,7.20,3840
```

- `valid`: `1` when the red-pixel percentage passes the threshold, otherwise `0`.
- `error_x_px`: centroid offset from the image center; positive is right.
- `error_y_px`: centroid offset from the image center; positive is down.
- `red_pixel_count`: number of pixels that passed the red filter.

When `valid` is `0`, both error values are placeholders and should not be used to drive motion.

## Hardware and software

- AI Thinker ESP32-CAM-compatible board
- Camera module supported by `esp_camera`
- USB-to-UART adapter for programming and serial monitoring
- Arduino IDE with the ESP32 board package installed

## Upload and run

1. Open `ESP32RedTracker/ESP32RedTracker.ino` in Arduino IDE.
2. Select **AI Thinker ESP32-CAM** and the adapter's serial port.
3. Put the board in flashing mode using the required IO0-to-GND connection, then reset it.
4. Upload the sketch.
5. Remove the IO0-to-GND flashing connection and reset the board again.
6. Open Serial Monitor at **115200 baud**.

## Processing pipeline

1. Initialize the camera in RGB565 mode.
2. Capture and validate a 160 × 120 frame.
3. Decode each pixel into red, green, and blue values.
4. Apply brightness and red-dominance thresholds.
5. Count red pixels and accumulate their X/Y positions.
6. Calculate red percentage and centroid.
7. Compare the centroid with the image center.
8. Print the detection flag, offsets, and pixel count.

## Tunable parameters

The initial values are near the top of the sketch:

```cpp
constexpr uint8_t kMinimumRed = 100;
constexpr uint16_t kRedDominancePercent = 140;
constexpr float kRedPresentPercent = 3.0f;
```

These values should be calibrated under the lighting conditions used in the lab.

## Known limitations and next experiments

- Fixed RGB thresholds are sensitive to lighting, shadows, and red backgrounds.
- Multiple red regions are combined into one centroid.
- The 160 × 120 frame size limits precision for small or distant objects.
- A useful next step is connected-component filtering to isolate the largest continuous red object.
- UART output should be tested with the motion-control hardware for latency and steering accuracy.

## Repository structure

```text
ESP32-Red-Object-Tracker/
├── README.md
└── ESP32RedTracker/
    └── ESP32RedTracker.ino
```
