#include "touch_input.h"
#include "hw_config.h"
#include "display.h"
#include <Arduino.h>
#include <Preferences.h>

#define T_CLK  TOUCH_PIN_CLK
#define T_CS   TOUCH_PIN_CS
#define T_DIN  TOUCH_PIN_MOSI
#define T_DOUT TOUCH_PIN_MISO
#define T_IRQ  TOUCH_PIN_IRQ

#define CMD_X  0xD0
#define CMD_Y  0x90
#define CMD_Z1 0xB0
#define CMD_Z2 0xC0

static TouchCalibration cal;
static Preferences prefs;
static volatile uint16_t cur_btns = 0;
static volatile int16_t scr_x = -1, scr_y = -1;
static volatile bool pressed = false;
static uint32_t last_ms = 0;

// ─── Bit-bang SPI ───────────────────────────────────────────────────────────
static uint16_t spi16(uint8_t cmd) {
    for (int i = 7; i >= 0; i--) {
        digitalWrite(T_DIN, (cmd >> i) & 1);
        digitalWrite(T_CLK, HIGH); delayMicroseconds(1);
        digitalWrite(T_CLK, LOW);  delayMicroseconds(1);
    }
    uint16_t r = 0;
    for (int i = 0; i < 13; i++) {
        digitalWrite(T_CLK, HIGH); delayMicroseconds(1);
        if (i > 0) { r <<= 1; r |= digitalRead(T_DOUT); }
        digitalWrite(T_CLK, LOW); delayMicroseconds(1);
    }
    return r;
}

static bool read_raw(int16_t* rx, int16_t* ry, int16_t* rz) {
    digitalWrite(T_CS, LOW);
    uint16_t z1 = spi16(CMD_Z1), z2 = spi16(CMD_Z2);
    int16_t z = z1 - z2 + 4095;
    if (z < 200) { digitalWrite(T_CS, HIGH); *rz = 0; return false; }
    uint32_t sx = 0, sy = 0; int n = 0;
    for (int i = 0; i < 6; i++) {  // 6 samples for better accuracy
        uint16_t x = spi16(CMD_X), y = spi16(CMD_Y);
        if (x > 100 && x < 4000 && y > 100 && y < 4000) { sx += x; sy += y; n++; }
    }
    digitalWrite(T_CS, HIGH);
    if (n < 2) { *rz = 0; return false; }  // need at least 2 good samples
    *rx = sx / n; *ry = sy / n; *rz = z;
    return true;
}

// ─── NVS Save/Load ──────────────────────────────────────────────────────────
static void save_cal_to_nvs() {
    prefs.begin("touch", false);
    prefs.putShort("xmin", cal.x_min);
    prefs.putShort("xmax", cal.x_max);
    prefs.putShort("ymin", cal.y_min);
    prefs.putShort("ymax", cal.y_max);
    prefs.putBool("swap", cal.swapped);
    prefs.putBool("invx", cal.invert_x);
    prefs.putBool("invy", cal.invert_y);
    prefs.putBool("valid", true);
    prefs.end();
    Serial.println("[CAL] Saved to NVS");
}

static bool load_cal_from_nvs() {
    prefs.begin("touch", true);
    bool valid = prefs.getBool("valid", false);
    if (valid) {
        cal.x_min = prefs.getShort("xmin", 200);
        cal.x_max = prefs.getShort("xmax", 3800);
        cal.y_min = prefs.getShort("ymin", 200);
        cal.y_max = prefs.getShort("ymax", 3800);
        cal.swapped = prefs.getBool("swap", true);
        cal.invert_x = prefs.getBool("invx", true);
        cal.invert_y = prefs.getBool("invy", false);
        Serial.printf("[CAL] Loaded from NVS: x[%d-%d] y[%d-%d] sw=%d ix=%d iy=%d\n",
                      cal.x_min, cal.x_max, cal.y_min, cal.y_max,
                      cal.swapped, cal.invert_x, cal.invert_y);
    }
    prefs.end();
    return valid;
}

// ─── Settings NVS ───────────────────────────────────────────────────────────
void touch_save_settings(uint8_t palette, uint8_t fskip, uint8_t brightness,
                         bool show_fps, bool show_save_overlay, uint8_t morse_speed) {
    prefs.begin("settings", false);
    prefs.putUChar("pal", palette);
    prefs.putUChar("fskip", fskip);
    prefs.putUChar("bright", brightness);
    prefs.putBool("ov_fps", show_fps);
    prefs.putBool("ov_save", show_save_overlay);
    prefs.putUChar("morse_spd", morse_speed);
    prefs.end();
}

bool touch_load_settings(uint8_t* palette, uint8_t* fskip, uint8_t* brightness,
                         bool* show_fps, bool* show_save_overlay, uint8_t* morse_speed) {
    prefs.begin("settings", true);
    bool has = prefs.isKey("pal");
    if (has) {
        *palette = prefs.getUChar("pal", 0);
        *fskip = prefs.getUChar("fskip", 0);
        *brightness = prefs.getUChar("bright", 255);
        if (show_fps) *show_fps = prefs.getBool("ov_fps", false);
        if (show_save_overlay) *show_save_overlay = prefs.getBool("ov_save", false);
        if (morse_speed) *morse_speed = prefs.getUChar("morse_spd", 50);
    }
    prefs.end();
    return has;
}

// ─── Init ───────────────────────────────────────────────────────────────────
TouchCalibration touch_get_default_calibration() {
    return {200, 3800, 200, 3800, true, true, false};
}

void touch_init() {
    pinMode(T_CLK, OUTPUT); pinMode(T_CS, OUTPUT);
    pinMode(T_DIN, OUTPUT); pinMode(T_DOUT, INPUT); pinMode(T_IRQ, INPUT);
    digitalWrite(T_CS, HIGH); digitalWrite(T_CLK, LOW);

    // Try loading saved calibration from NVS
    if (!load_cal_from_nvs()) {
        cal = touch_get_default_calibration();
        Serial.println("[CAL] Using defaults (no saved calibration)");
    }

    Serial.printf("[TOUCH] CLK=%d CS=%d DIN=%d DOUT=%d IRQ=%d\n",
                  T_CLK, T_CS, T_DIN, T_DOUT, T_IRQ);
}

void touch_set_calibration(TouchCalibration c) { cal = c; }

// ─── Mapping ────────────────────────────────────────────────────────────────
static void map_screen(int16_t rx, int16_t ry, int16_t* ox, int16_t* oy) {
    if (cal.swapped) { int16_t t = rx; rx = ry; ry = t; }
    int32_t mx = (int32_t)(rx - cal.x_min) * SCREEN_W / (cal.x_max - cal.x_min);
    int32_t my = (int32_t)(ry - cal.y_min) * SCREEN_H / (cal.y_max - cal.y_min);
    if (cal.invert_x) mx = SCREEN_W - 1 - mx;
    if (cal.invert_y) my = SCREEN_H - 1 - my;
    *ox = constrain(mx, 0, SCREEN_W - 1);
    *oy = constrain(my, 0, SCREEN_H - 1);
}

// ─── Button classification ──────────────────────────────────────────────────
static uint16_t classify(int16_t x, int16_t y) {
    uint16_t b = 0;

    // D-pad
    int dx = x - DPAD_CX, dy = y - DPAD_CY;
    int32_t d2 = (int32_t)dx*dx + (int32_t)dy*dy;
    if (d2 <= (int32_t)(DPAD_R+12)*(DPAD_R+12) && d2 > 49) {
        int ax = abs(dx), ay = abs(dy);
        if (ax > ay/3) b |= (dx < 0) ? GB_BTN_LEFT : GB_BTN_RIGHT;
        if (ay > ax/3) b |= (dy < 0) ? GB_BTN_UP : GB_BTN_DOWN;
    }

    // A
    { int32_t d=(int32_t)(x-BTN_A_X)*(x-BTN_A_X)+(int32_t)(y-BTN_A_Y)*(y-BTN_A_Y);
      if(d<=(int32_t)(BTN_A_R+10)*(BTN_A_R+10)) b|=GB_BTN_A; }
    // B
    { int32_t d=(int32_t)(x-BTN_B_X)*(x-BTN_B_X)+(int32_t)(y-BTN_B_Y)*(y-BTN_B_Y);
      if(d<=(int32_t)(BTN_B_R+10)*(BTN_B_R+10)) b|=GB_BTN_B; }
    // Start
    if(x>=BTN_ST_X-BTN_ST_W/2-6&&x<=BTN_ST_X+BTN_ST_W/2+6&&
       y>=BTN_ST_Y-BTN_ST_H/2-6&&y<=BTN_ST_Y+BTN_ST_H/2+6) b|=GB_BTN_START;
    // Select
    if(x>=BTN_SE_X-BTN_SE_W/2-6&&x<=BTN_SE_X+BTN_SE_W/2+6&&
       y>=BTN_SE_Y-BTN_SE_H/2-6&&y<=BTN_SE_Y+BTN_SE_H/2+6) b|=GB_BTN_SELECT;
    // Menu
    { int32_t d=(int32_t)(x-BTN_M_X)*(x-BTN_M_X)+(int32_t)(y-BTN_M_Y)*(y-BTN_M_Y);
      if(d<=(int32_t)(BTN_M_R+8)*(BTN_M_R+8)) b|=GB_BTN_MENU; }

    return b;
}

// ─── Update ─────────────────────────────────────────────────────────────────
void touch_update() {
    uint32_t now = millis();
    if (now - last_ms < 14) return;
    last_ms = now;
    if (digitalRead(T_IRQ) == HIGH) {
        pressed = false; cur_btns = 0; scr_x = scr_y = -1; return;
    }
    int16_t rx, ry, rz;
    if (read_raw(&rx, &ry, &rz)) {
        int16_t mx, my; map_screen(rx, ry, &mx, &my);
        scr_x = mx; scr_y = my; pressed = true; cur_btns = classify(mx, my);
    } else { pressed = false; cur_btns = 0; scr_x = scr_y = -1; }
}

uint16_t touch_get_buttons() { return cur_btns; }
bool touch_is_pressed() { return pressed; }
int16_t touch_get_x() { return scr_x; }
int16_t touch_get_y() { return scr_y; }

// ─── Smart Calibration ─────────────────────────────────────────────────────
// Uses 5 points (4 corners + center) for better accuracy
// Calculates linear regression for X and Y mapping
// Saves to NVS automatically
void touch_run_calibration() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(0xFFE0);
    tft.drawString("CALIBRATION", SCREEN_W/2, 16, 4);
    tft.setTextColor(0xAD55);
    tft.drawString("Touch each + carefully", SCREEN_W/2, 46, 2);

    // 5 calibration points: 4 corners + center
    struct { int16_t sx, sy; } targets[5] = {
        {22, 80}, {218, 80}, {22, 280}, {218, 280}, {120, 180}
    };
    const char* labels[5] = {"Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right", "Center"};
    int16_t raw_x[5], raw_y[5];
    bool got[5] = {false};

    for (int i = 0; i < 5; i++) {
        // Clear instruction area
        tft.fillRect(0, 42, SCREEN_W, 28, TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM);
        char msg[32]; snprintf(msg, 32, "%d/5: %s", i + 1, labels[i]);
        tft.drawString(msg, SCREEN_W/2, 70, 2);

        // Draw crosshair with circle
        int tx = targets[i].sx, ty = targets[i].sy;
        tft.drawCircle(tx, ty, 10, 0x07E0);
        tft.drawCircle(tx, ty, 4, 0x07E0);
        tft.drawLine(tx - 14, ty, tx + 14, ty, 0x07E0);
        tft.drawLine(tx, ty - 14, tx, ty + 14, 0x07E0);

        // Wait for touch with timeout
        uint32_t t0 = millis();
        while (digitalRead(T_IRQ) == HIGH) {
            delay(10);
            if (millis() - t0 > 15000) goto cal_fail;  // 15s timeout
        }
        delay(100);  // settle time

        // Take multiple samples and median-filter
        int16_t samples_x[8], samples_y[8];
        int ns = 0;
        for (int s = 0; s < 8; s++) {
            int16_t rx, ry, rz;
            if (read_raw(&rx, &ry, &rz)) {
                samples_x[ns] = rx;
                samples_y[ns] = ry;
                ns++;
            }
            delay(30);
        }

        if (ns >= 3) {
            // Sort and take median
            for (int a = 0; a < ns-1; a++) for (int b = a+1; b < ns; b++) {
                if (samples_x[a] > samples_x[b]) { int16_t t = samples_x[a]; samples_x[a] = samples_x[b]; samples_x[b] = t; }
                if (samples_y[a] > samples_y[b]) { int16_t t = samples_y[a]; samples_y[a] = samples_y[b]; samples_y[b] = t; }
            }
            raw_x[i] = samples_x[ns / 2];
            raw_y[i] = samples_y[ns / 2];
            got[i] = true;
            Serial.printf("[CAL] %d: raw(%d,%d) -> screen(%d,%d) [%d samples]\n",
                          i, raw_x[i], raw_y[i], tx, ty, ns);
        }

        // Mark done
        tft.fillCircle(tx, ty, 8, got[i] ? TFT_GREEN : TFT_RED);

        // Wait release
        while (digitalRead(T_IRQ) == LOW) delay(10);
        delay(300);
    }

    // ─── Calculate calibration from 5 points ────────────────────────────────
    {
        // Check we got all points
        int valid = 0;
        for (int i = 0; i < 5; i++) if (got[i]) valid++;
        if (valid < 4) goto cal_fail;

        TouchCalibration nc;

        // Determine swap: top-left to top-right should change screen X
        // If raw_x changes more -> not swapped. If raw_y changes more -> swapped
        int16_t dx_raw = abs(raw_x[1] - raw_x[0]);
        int16_t dy_raw = abs(raw_y[1] - raw_y[0]);
        nc.swapped = (dx_raw < dy_raw);

        int16_t mx[5], my[5];
        for (int i = 0; i < 5; i++) {
            mx[i] = nc.swapped ? raw_y[i] : raw_x[i];
            my[i] = nc.swapped ? raw_x[i] : raw_y[i];
        }

        // Determine inversion
        nc.invert_x = (mx[0] > mx[1]);  // left has higher raw than right
        nc.invert_y = (my[0] > my[2]);  // top has higher raw than bottom

        // Compute axis bounds from known calibration point geometry.
        // This avoids symmetric "magic" extrapolation and improves Y accuracy.
        int32_t ox[5], oy[5];
        for (int i = 0; i < 5; i++) {
            ox[i] = nc.invert_x ? -(int32_t)mx[i] : (int32_t)mx[i];
            oy[i] = nc.invert_y ? -(int32_t)my[i] : (int32_t)my[i];
        }

        // Pairwise averages from corner points in oriented space.
        int32_t left_raw_o = (ox[0] + ox[2]) / 2;
        int32_t right_raw_o = (ox[1] + ox[3]) / 2;
        int32_t top_raw_o = (oy[0] + oy[1]) / 2;
        int32_t bottom_raw_o = (oy[2] + oy[3]) / 2;

        int32_t x_span_raw = max((int32_t)1, right_raw_o - left_raw_o);
        int32_t y_span_raw = max((int32_t)1, bottom_raw_o - top_raw_o);
        int32_t x_span_px = max((int32_t)1, (int32_t)targets[1].sx - (int32_t)targets[0].sx);
        int32_t y_span_px = max((int32_t)1, (int32_t)targets[2].sy - (int32_t)targets[0].sy);

        int32_t x_left_px = targets[0].sx;
        int32_t x_right_px = (SCREEN_W - 1) - targets[1].sx;
        int32_t y_top_px = targets[0].sy;
        int32_t y_bottom_px = (SCREEN_H - 1) - targets[2].sy;

        int32_t x_left_ext = (x_span_raw * x_left_px + x_span_px / 2) / x_span_px;
        int32_t x_right_ext = (x_span_raw * x_right_px + x_span_px / 2) / x_span_px;
        int32_t y_top_ext = (y_span_raw * y_top_px + y_span_px / 2) / y_span_px;
        int32_t y_bottom_ext = (y_span_raw * y_bottom_px + y_span_px / 2) / y_span_px;

        int32_t x_min_o = left_raw_o - x_left_ext;
        int32_t x_max_o = right_raw_o + x_right_ext;
        int32_t y_min_o = top_raw_o - y_top_ext;
        int32_t y_max_o = bottom_raw_o + y_bottom_ext;

        if (!nc.invert_x) {
            nc.x_min = (int16_t)x_min_o;
            nc.x_max = (int16_t)x_max_o;
        } else {
            nc.x_min = (int16_t)(-x_max_o);
            nc.x_max = (int16_t)(-x_min_o);
        }

        if (!nc.invert_y) {
            nc.y_min = (int16_t)y_min_o;
            nc.y_max = (int16_t)y_max_o;
        } else {
            nc.y_min = (int16_t)(-y_max_o);
            nc.y_max = (int16_t)(-y_min_o);
        }

        if (nc.x_min >= nc.x_max) { int16_t t = nc.x_min; nc.x_min = nc.x_max; nc.x_max = t; }
        if (nc.y_min >= nc.y_max) { int16_t t = nc.y_min; nc.y_min = nc.y_max; nc.y_max = t; }

        cal = nc;
        save_cal_to_nvs();

        Serial.printf("[CAL] Done: sw=%d ix=%d iy=%d x[%d-%d] y[%d-%d]\n",
                      cal.swapped, cal.invert_x, cal.invert_y,
                      cal.x_min, cal.x_max, cal.y_min, cal.y_max);
    }

    // ─── Verification ───────────────────────────────────────────────────────
    {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(0x07E0);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("VERIFY", SCREEN_W/2, 13, 4);
        tft.setTextColor(0xAD55);
        tft.drawString("Draw to test accuracy", SCREEN_W/2, 46, 2);
        tft.setTextColor(0x7BEF);
        tft.drawString("Wait 5s or lift to exit", SCREEN_W/2, 304, 1);

        // Draw reference grid (scaled steps)
        for (int x = 0; x <= SCREEN_W; x += 48) tft.drawFastVLine(x, 66, 228, 0x18C3);
        for (int y = 66; y <= 293; y += 56) tft.drawFastHLine(0, y, SCREEN_W, 0x18C3);
        // Draw corner markers (scaled)
        tft.drawCircle(22, 80, 4, 0x4A69);
        tft.drawCircle(218, 80, 4, 0x4A69);
        tft.drawCircle(22, 280, 4, 0x4A69);
        tft.drawCircle(218, 280, 4, 0x4A69);
        tft.drawCircle(120, 180, 4, 0x4A69);

        uint32_t t0 = millis();
        uint32_t no_touch_since = 0;
        bool was_touching = false;

        while (millis() - t0 < 8000) {
            if (digitalRead(T_IRQ) == LOW) {
                int16_t rx, ry, rz;
                if (read_raw(&rx, &ry, &rz)) {
                    int16_t mx, my; map_screen(rx, ry, &mx, &my);
                    tft.fillCircle(mx, my, 2, TFT_RED);
                }
                was_touching = true;
                no_touch_since = millis();
            } else {
                if (was_touching && millis() - no_touch_since > 2000) break;
            }
            delay(15);
        }
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(0x07E0);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Calibration Saved!", SCREEN_W/2, 146, 4);
    delay(1200);
    return;

cal_fail:
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Calibration Failed!", SCREEN_W/2, 133, 4);
    tft.setTextColor(0x7BEF);
    tft.drawString("Using previous values", SCREEN_W/2, 186, 2);
    delay(2000);
}
