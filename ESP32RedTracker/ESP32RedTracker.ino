#include "esp_camera.h"

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

namespace {

// Initial red filter tuning values. A pixel counts as red only when
// it is bright enough and clearly stronger than both green and blue.
constexpr uint8_t kMinimumRed = 100;
constexpr uint16_t kRedDominancePercent = 140;
constexpr float kRedPresentPercent = 3.0f;

bool isRedPixel(uint8_t red, uint8_t green, uint8_t blue) {
  if (red < kMinimumRed) {
    return false;
  }

  return static_cast<uint16_t>(red) * 100U >=
             static_cast<uint16_t>(green) * kRedDominancePercent &&
         static_cast<uint16_t>(red) * 100U >=
             static_cast<uint16_t>(blue) * kRedDominancePercent;
}

void analyzeRgb565Frame(const camera_fb_t *frame) {
  if (frame == nullptr || frame->format != PIXFORMAT_RGB565 ||
      frame->width == 0 || frame->height == 0 ||
      frame->len != frame->width * frame->height * 2U) {
    Serial.println("Frame is not RGB565; color analysis skipped.");
    return;
  }

  uint64_t redTotal = 0;
  uint64_t greenTotal = 0;
  uint64_t blueTotal = 0;
  uint32_t pixelCount = 0;
  uint32_t redPixelCount = 0;
  uint32_t sumX = 0, sumY = 0;

  // RGB565 uses two bytes per pixel, so advancing by two examines every pixel.
  for (size_t i = 0; i + 1 < frame->len; i += 2) {
    const uint16_t pixel =
        (static_cast<uint16_t>(frame->buf[i]) << 8) | frame->buf[i + 1];

    const uint8_t red5 = (pixel >> 11) & 0x1F;
    const uint8_t green6 = (pixel >> 5) & 0x3F;
    const uint8_t blue5 = pixel & 0x1F;

    const uint8_t red = (red5 * 255U) / 31U;
    const uint8_t green = (green6 * 255U) / 63U;
    const uint8_t blue = (blue5 * 255U) / 31U;

    redTotal += red;
    greenTotal += green;
    blueTotal += blue;
    ++pixelCount;

    // This is the segmentation step: every pixel becomes either red or not
    // red according to the threshold function above.
    if (isRedPixel(red, green, blue)) {
      ++redPixelCount;
      const size_t index = i / 2;
      sumX += index % frame->width;
      sumY += index / frame->width;
    }
  }

  if (pixelCount == 0) {
    Serial.println("No pixels were available to analyze.");
    return;
  }

  const float redPercent =
      (100.0f * static_cast<float>(redPixelCount)) / pixelCount;

  // Positive X is right; positive Y is down in the delivered camera image.
  // TRACK,valid,error_x_px,error_y_px,red_pixel_count
  // valid=0 means errors are placeholders and must not drive motion.
  const bool valid = redPixelCount > 0 && redPercent >= kRedPresentPercent;
  const float errorX = valid ? float(sumX) / redPixelCount - (frame->width - 1) * 0.5f : 0;
  const float errorY = valid ? float(sumY) / redPixelCount - (frame->height - 1) * 0.5f : 0;
  Serial.printf("TRACK,%u,%.2f,%.2f,%lu\n", valid ? 1 : 0,
                errorX, errorY, (unsigned long)redPixelCount);
}

bool cameraReady = false;

bool startCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;  // 160 x 120 for reliable bring-up.
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  const esp_err_t result = esp_camera_init(&config);
  if (result != ESP_OK) {
    Serial.printf("Camera initialization failed: 0x%x\n", result);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    Serial.printf("Camera sensor PID: 0x%04x\n", sensor->id.PID);

    // The supplied module is marked OV3660. Espressif recommends correcting
    // its default vertical orientation and slightly reducing saturation.
    if (sensor->id.PID == OV3660_PID) {
      sensor->set_vflip(sensor, 1);
      sensor->set_saturation(sensor, -2);
      Serial.println("OV3660 adjustments applied.");
    }
  }

  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("ESP32-CAM bring-up starting...");

  if (!startCamera()) {
    Serial.println("Check the camera ribbon and restart the board.");
    return;
  }

  cameraReady = true;
  Serial.println("Camera initialized successfully.");
}

void loop() {
  if (!cameraReady) { delay(100); return; }
  camera_fb_t *frame = esp_camera_fb_get();
  if (frame == nullptr) {
    Serial.println("Frame capture failed.");
    delay(100);
    return;
  }

  analyzeRgb565Frame(frame);
  esp_camera_fb_return(frame);

  delay(100);
}
