#include <Arduino.h>
#include <Preferences.h>
#include <TFT_eSPI.h>

#include <cydkit/board_profile.h>
#include <cydkit/touch_driver.h>

#include "sender/apps/beacon_sender.h"

TFT_eSPI tft;
Preferences prefs;
cydkit::TouchDriver touch(
    cydkit::kFakeCydProfile.touchCs,
    cydkit::kFakeCydProfile.touchIrq,
    cydkit::kFakeCydProfile.touchMosi,
    cydkit::kFakeCydProfile.touchMiso,
    cydkit::kFakeCydProfile.touchClk);

namespace {

constexpr int kScreenWidth = 240;
constexpr int kScreenHeight = 320;
constexpr int kSenderRotation = 0;
constexpr const char* kTouchPrefsNamespace = "touchsnd";
constexpr uint32_t kBootWaitMs = 1400;
constexpr uint32_t kCalibrateHoldMs = 900;
constexpr int kCalibratePointCount = 4;

void drawBootScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BT Beacon Sender", kScreenWidth / 2, 110, 4);
  tft.drawString("Touch halten fur", kScreenWidth / 2, 154, 2);
  tft.drawString("Kalibrierung", kScreenWidth / 2, 178, 2);
  tft.setTextColor(0x7BEF, TFT_BLACK);
  tft.drawString("Startet gleich...", kScreenWidth / 2, 230, 2);
}

void drawCalibrationPoint(int x, int y, const char* label) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Touch-Kalibrierung", kScreenWidth / 2, 28, 2);
  tft.drawString("Bitte Punkt antippen", kScreenWidth / 2, 52, 2);
  tft.drawString(label, kScreenWidth / 2, 76, 2);
  tft.drawFastHLine(x - 12, y, 24, TFT_RED);
  tft.drawFastVLine(x, y - 12, 24, TFT_RED);
  tft.drawCircle(x, y, 14, TFT_RED);
}

bool captureCalibrationPoint(int targetX, int targetY, const char* label,
                             int16_t* rawX, int16_t* rawY) {
  drawCalibrationPoint(targetX, targetY, label);

  uint32_t stableSince = 0;
  int16_t sampleX = 0;
  int16_t sampleY = 0;
  bool pressed = false;

  while (true) {
    int16_t currentX = 0;
    int16_t currentY = 0;
    int16_t currentZ = 0;
    bool touching = touch.readRaw(&currentX, &currentY, &currentZ);

    if (touching) {
      if (!pressed) {
        pressed = true;
        stableSince = millis();
      }
      sampleX = currentX;
      sampleY = currentY;
    } else if (pressed) {
      if (millis() - stableSince >= 120) {
        *rawX = sampleX;
        *rawY = sampleY;
        while (touch.readRaw(&currentX, &currentY, &currentZ)) {
          delay(10);
        }
        delay(120);
        return true;
      }
      pressed = false;
    }

    delay(10);
  }
}

void drawCalibrationResult(bool success) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(success ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(success ? "Kalibrierung gespeichert" : "Kalibrierung fehlgeschlagen",
                 kScreenWidth / 2, 142, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Weiter zum Sender...", kScreenWidth / 2, 176, 2);
}

bool runTouchCalibration() {
  const int16_t screenX[kCalibratePointCount] = {24, kScreenWidth - 24, kScreenWidth - 24, 24};
  const int16_t screenY[kCalibratePointCount] = {32, 32, kScreenHeight - 32, kScreenHeight - 32};
  const char* labels[kCalibratePointCount] = {"oben links", "oben rechts", "unten rechts", "unten links"};
  int16_t rawX[kCalibratePointCount] = {0};
  int16_t rawY[kCalibratePointCount] = {0};

  for (int i = 0; i < kCalibratePointCount; ++i) {
    if (!captureCalibrationPoint(screenX[i], screenY[i], labels[i], &rawX[i], &rawY[i])) {
      return false;
    }
  }

  cydkit::TouchCalibration calibration{};
  if (!cydkit::TouchDriver::computeAffineCalibration(rawX, rawY, screenX, screenY,
                                                     kCalibratePointCount, &calibration)) {
    return false;
  }

  touch.setCalibration(calibration);
  touch.saveCalibration(prefs, kTouchPrefsNamespace);
  return true;
}

void maybeRunTouchCalibration() {
  drawBootScreen();

  uint32_t startMs = millis();
  uint32_t holdSince = 0;
  bool calibrationRequested = false;

  while (millis() - startMs < kBootWaitMs) {
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;
    bool touching = touch.readRaw(&rawX, &rawY, &rawZ);

    if (touching) {
      if (holdSince == 0) {
        holdSince = millis();
      }
      if (millis() - holdSince >= kCalibrateHoldMs) {
        calibrationRequested = true;
        break;
      }
    } else {
      holdSince = 0;
    }

    delay(10);
  }

  if (!calibrationRequested) {
    return;
  }

  while (true) {
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;
    if (!touch.readRaw(&rawX, &rawY, &rawZ)) {
      break;
    }
    delay(10);
  }

  bool success = runTouchCalibration();
  drawCalibrationResult(success);
  delay(800);
}

}  // namespace

bool touchReadScreen(uint16_t* x, uint16_t* y) {
  return touch.readScreen(x, y, kScreenWidth, kScreenHeight);
}

void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(cydkit::kFakeCydProfile.tftBacklight, OUTPUT);
  digitalWrite(cydkit::kFakeCydProfile.tftBacklight, HIGH);

  touch.beginPins();

  tft.init();
  tft.setRotation(kSenderRotation);
  tft.setSwapBytes(true);
  tft.invertDisplay(true);
  tft.setTextWrap(false, false);

  touch.loadCalibration(prefs, kTouchPrefsNamespace);
  maybeRunTouchCalibration();

  btSenderEnter();
}

void loop() {
  btSenderLoop();
  delay(10);
}
