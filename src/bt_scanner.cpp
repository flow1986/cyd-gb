#include "bt_scanner.h"

#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <Preferences.h>
#include <string.h>
#include <ctype.h>

#include "display.h"
#include "hw_config.h"
#include "touch_input.h"
#include "button_input.h"

static constexpr int SCREEN_WIDTH = 240;
static constexpr int SCREEN_HEIGHT = 320;

static constexpr uint16_t COLOR_BG = TFT_BLACK;
static constexpr uint16_t COLOR_HEADER_BG = 0x18C3;
static constexpr uint16_t COLOR_HEADER_TEXT = TFT_WHITE;
static constexpr uint16_t COLOR_HEADER_ACCENT = 0x7BEF;
static constexpr uint16_t COLOR_ITEM_BG = 0x0000;
static constexpr uint16_t COLOR_ITEM_TEXT = TFT_WHITE;
static constexpr uint16_t COLOR_ITEM_HL_BG = 0x0014;
static constexpr uint16_t COLOR_ITEM_HL_TEXT = 0xFFE0;
static constexpr uint16_t COLOR_FOOTER_BG = 0x18C3;
static constexpr uint16_t COLOR_FOOTER_TEXT = 0x7BEF;

static constexpr unsigned long BT_SCAN_INTERVAL_MS = 1800;
static constexpr unsigned long MORSE_BASE_UNIT_MS = 220;
static constexpr unsigned long MORSE_PREP_DELAY_MS = 1800;
static constexpr unsigned long MORSE_REPEAT_GAP_MS = 1200;
static constexpr unsigned long MORSE_FAKE_ACTIVITY_MS = 1400;
static constexpr uint8_t MORSE_WORD_MAX_LEN = 12;
static constexpr unsigned long SETTINGS_LONGPRESS_MS = 900;
static constexpr uint8_t MORSE_DEFAULT_SPEED_PERCENT = 50;
static constexpr uint8_t MORSE_MIN_SPEED_PERCENT = 10;
static constexpr uint8_t MORSE_MAX_SPEED_PERCENT = 100;
static constexpr uint8_t SCANNER_ID_MIN = 1;
static constexpr uint8_t SCANNER_ID_MAX = 8;

static constexpr char kBleServiceUuid[] = "7f4ac8ec-9b6e-46f0-b6ee-6f95a2d0a920";
static constexpr char kBlePasswordCharUuid[] = "7f4ac8ec-9b6e-46f0-b6ee-6f95a2d0a921";

struct BeaconStation {
    char name[24];
    char mac[18];
    int rssi;
    bool rssiValid;
};

enum KeyboardMode {
    KB_ALPHA,
    KB_MAC
};

enum KeyboardAction {
    KEY_CHAR,
    KEY_SPACE,
    KEY_BACKSPACE,
    KEY_SAVE,
    KEY_SWITCH,
    KEY_CLEAR
};

struct KeyDef {
    const char* label;
    KeyboardAction action;
    char value;
};

struct KeyboardState {
    bool active;
    KeyboardMode mode;
    uint8_t stationIndex;
    uint8_t fieldIndex;
    char buffer[32];
    uint8_t maxLen;
};

enum ScannerView {
    VIEW_LIST,
    VIEW_MONITOR,
    VIEW_EDIT_MENU,
    VIEW_EDIT_NAME,
    VIEW_EDIT_MAC,
    VIEW_MORSE_MENU,
    VIEW_MORSE_EDIT
};

struct MorseCodeEntry {
    char ch;
    const char* code;
};

static const KeyDef alphaRow1[] = {
    {"Q", KEY_CHAR, 'Q'}, {"W", KEY_CHAR, 'W'}, {"E", KEY_CHAR, 'E'}, {"R", KEY_CHAR, 'R'}, {"T", KEY_CHAR, 'T'},
    {"Z", KEY_CHAR, 'Z'}, {"U", KEY_CHAR, 'U'}, {"I", KEY_CHAR, 'I'}, {"O", KEY_CHAR, 'O'}, {"P", KEY_CHAR, 'P'}
};
static const KeyDef alphaRow2[] = {
    {"A", KEY_CHAR, 'A'}, {"S", KEY_CHAR, 'S'}, {"D", KEY_CHAR, 'D'}, {"F", KEY_CHAR, 'F'}, {"G", KEY_CHAR, 'G'},
    {"H", KEY_CHAR, 'H'}, {"J", KEY_CHAR, 'J'}, {"K", KEY_CHAR, 'K'}, {"L", KEY_CHAR, 'L'}
};
static const KeyDef alphaRow3[] = {
    {"Y", KEY_CHAR, 'Y'}, {"X", KEY_CHAR, 'X'}, {"C", KEY_CHAR, 'C'}, {"V", KEY_CHAR, 'V'}, {"B", KEY_CHAR, 'B'}, {"N", KEY_CHAR, 'N'}, {"M", KEY_CHAR, 'M'}
};
static const KeyDef alphaRow4[] = {
    {"SPACE", KEY_SPACE, ' '}, {"BKSP", KEY_BACKSPACE, 0}, {"123", KEY_SWITCH, 0}, {"OK", KEY_SAVE, 0}
};

static const KeyDef macRow1[] = {
    {"0", KEY_CHAR, '0'}, {"1", KEY_CHAR, '1'}, {"2", KEY_CHAR, '2'}, {"3", KEY_CHAR, '3'}, {"4", KEY_CHAR, '4'},
    {"5", KEY_CHAR, '5'}, {"6", KEY_CHAR, '6'}, {"7", KEY_CHAR, '7'}, {"8", KEY_CHAR, '8'}, {"9", KEY_CHAR, '9'}
};
static const KeyDef macRow2[] = {
    {"A", KEY_CHAR, 'A'}, {"B", KEY_CHAR, 'B'}, {"C", KEY_CHAR, 'C'}, {"D", KEY_CHAR, 'D'}, {"E", KEY_CHAR, 'E'}, {"F", KEY_CHAR, 'F'}, {":", KEY_CHAR, ':'}
};
static const KeyDef macRow3[] = {
    {"BKSP", KEY_BACKSPACE, 0}, {"ABC", KEY_SWITCH, 0}, {"CLR", KEY_CLEAR, 0}, {"OK", KEY_SAVE, 0}
};

static const MorseCodeEntry kMorseTable[] = {
    {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."}, {'E', "."},
    {'F', "..-."}, {'G', "--."}, {'H', "...."}, {'I', ".."}, {'J', ".---"},
    {'K', "-.-"}, {'L', ".-.."}, {'M', "--"}, {'N', "-."}, {'O', "---"},
    {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
    {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"}, {'Y', "-.--"},
    {'Z', "--.."}, {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
    {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."},
    {'9', "----."}
};

static Preferences btPrefs;
static BLEScan* bleScan = nullptr;
static BeaconStation stations[4];
static KeyboardState keyboardState{};
static ScannerView scannerView = VIEW_LIST;
static int selectedStationIndex = 0;
static int selectedListIndex = 0;
static int selectedEditChoice = 0;
static int selectedMorseChoice = 0;
static unsigned long lastScanMs = 0;
static bool needsScan = true;
static uint16_t lastButtons = 0;
static bool touchPressed = false;
static bool touchLongHandled = false;
static bool ignoreTouchUntilRelease = false;
static unsigned long touchPressStartMs = 0;
static uint16_t touchPressX = 0;
static uint16_t touchPressY = 0;
static char morseWord[16] = "HINWEIS";
static uint8_t morseSpeedPercent = MORSE_DEFAULT_SPEED_PERCENT;
static uint8_t scannerId = 1;

static inline bool list_selection_is_morse() {
    return selectedListIndex >= 4;
}

static inline void clamp_list_selection() {
    if (selectedListIndex < 0) selectedListIndex = 0;
    if (selectedListIndex > 4) selectedListIndex = 4;
}

static inline bool btn_pressed_edge(uint16_t now, uint16_t prev, uint16_t mask) {
    return (now & mask) && !(prev & mask);
}

static uint16_t read_buttons() {
    button_update();
    return button_get_buttons();
}

static bool read_touch(uint16_t* x, uint16_t* y) {
    touch_update();
    if (!touch_is_pressed()) return false;
    int16_t tx = touch_get_x();
    int16_t ty = touch_get_y();
    if (tx < 0 || ty < 0) return false;
    *x = (uint16_t)tx;
    *y = (uint16_t)ty;
    return true;
}

static void reset_touch_gesture_state() {
    touchPressed = false;
    touchLongHandled = false;
}

static void block_touch_until_release() {
    ignoreTouchUntilRelease = true;
    reset_touch_gesture_state();
}

static void wait_for_button_release() {
    while (read_buttons() != 0) delay(20);
    delay(40);
}

static uint16_t rssi_to_color(int rssi) {
    int clamped = constrain(rssi, -100, -40);
    int scale = map(clamped, -100, -40, 0, 255);
    uint8_t red = (uint8_t)(255 - scale);
    uint8_t green = (uint8_t)(scale);
    return tft.color565(red, green, 0);
}

static void draw_header(const char* title) {
    tft.fillRect(0, 0, SCREEN_WIDTH, 36, COLOR_HEADER_BG);
    tft.setTextColor(COLOR_HEADER_TEXT, COLOR_HEADER_BG);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(title, 10, 18, 2);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(COLOR_HEADER_ACCENT, COLOR_HEADER_BG);
    tft.drawString("CYD-GB", SCREEN_WIDTH - 10, 18, 1);
}

static void draw_footer(const char* text) {
    tft.fillRect(0, SCREEN_HEIGHT - 20, SCREEN_WIDTH, 20, COLOR_FOOTER_BG);
    tft.setTextColor(COLOR_FOOTER_TEXT, COLOR_FOOTER_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, SCREEN_WIDTH / 2, SCREEN_HEIGHT - 10, 1);
}

static void normalize_mac(char* mac) {
    char cleaned[18] = {0};
    uint8_t out = 0;

    for (uint8_t i = 0; mac[i] != 0 && out < sizeof(cleaned) - 1; ++i) {
        char c = mac[i];
        if (c == '-') c = ':';
        if (isxdigit((unsigned char)c) || c == ':') {
            cleaned[out++] = (char)toupper((unsigned char)c);
        }
    }

    cleaned[out] = 0;
    strncpy(mac, cleaned, 18);
    mac[17] = 0;
}

static void normalize_morse_word(char* text) {
    char cleaned[16] = {0};
    uint8_t out = 0;

    for (uint8_t i = 0; text[i] != 0 && out < MORSE_WORD_MAX_LEN; ++i) {
        char c = (char)toupper((unsigned char)text[i]);
        if (isalnum((unsigned char)c)) cleaned[out++] = c;
    }

    cleaned[out] = 0;
    strncpy(text, cleaned, sizeof(cleaned));
    text[sizeof(cleaned) - 1] = 0;
}

static const char* morse_code_for_char(char c) {
    for (const MorseCodeEntry& entry : kMorseTable) {
        if (entry.ch == c) return entry.code;
    }
    return nullptr;
}

static void set_default_stations() {
    snprintf(stations[0].name, sizeof(stations[0].name), "Station 1");
    snprintf(stations[0].mac, sizeof(stations[0].mac), "AA:BB:CC:DD:EE:01");
    snprintf(stations[1].name, sizeof(stations[1].name), "Station 2");
    snprintf(stations[1].mac, sizeof(stations[1].mac), "AA:BB:CC:DD:EE:02");
    snprintf(stations[2].name, sizeof(stations[2].name), "Station 3");
    snprintf(stations[2].mac, sizeof(stations[2].mac), "AA:BB:CC:DD:EE:03");
    snprintf(stations[3].name, sizeof(stations[3].name), "Station 4");
    snprintf(stations[3].mac, sizeof(stations[3].mac), "AA:BB:CC:DD:EE:04");

    for (int i = 0; i < 4; ++i) {
        stations[i].rssi = -127;
        stations[i].rssiValid = false;
    }
}

static void load_stations() {
    set_default_stations();
    btPrefs.begin("bt", true);
    if (btPrefs.getBool("valid", false)) {
        for (int i = 0; i < 4; ++i) {
            String nameKey = "n" + String(i);
            String macKey = "m" + String(i);
            String nameValue = btPrefs.getString(nameKey.c_str(), stations[i].name);
            String macValue = btPrefs.getString(macKey.c_str(), stations[i].mac);
            nameValue.trim();
            macValue.trim();
            if (nameValue.length() > 0) {
                strncpy(stations[i].name, nameValue.c_str(), sizeof(stations[i].name) - 1);
                stations[i].name[sizeof(stations[i].name) - 1] = 0;
            }
            if (macValue.length() > 0) {
                strncpy(stations[i].mac, macValue.c_str(), sizeof(stations[i].mac) - 1);
                stations[i].mac[sizeof(stations[i].mac) - 1] = 0;
                normalize_mac(stations[i].mac);
            }
        }

        String morseValue = btPrefs.getString("morse", morseWord);
        morseValue.trim();
        if (morseValue.length() > 0) {
            strncpy(morseWord, morseValue.c_str(), sizeof(morseWord) - 1);
            morseWord[sizeof(morseWord) - 1] = 0;
            normalize_morse_word(morseWord);
        }

        morseSpeedPercent = constrain(
            btPrefs.getUChar("mspd", MORSE_DEFAULT_SPEED_PERCENT),
            MORSE_MIN_SPEED_PERCENT,
            MORSE_MAX_SPEED_PERCENT
        );

        scannerId = constrain(
            btPrefs.getUChar("sid", 1),
            SCANNER_ID_MIN,
            SCANNER_ID_MAX
        );
    }
    btPrefs.end();
}

static void save_stations() {
    btPrefs.begin("bt", false);
    btPrefs.putBool("valid", true);
    for (int i = 0; i < 4; ++i) {
        String nameKey = "n" + String(i);
        String macKey = "m" + String(i);
        btPrefs.putString(nameKey.c_str(), stations[i].name);
        btPrefs.putString(macKey.c_str(), stations[i].mac);
    }
    btPrefs.putString("morse", morseWord);
    btPrefs.putUChar("mspd", morseSpeedPercent);
    btPrefs.putUChar("sid", scannerId);
    btPrefs.end();
}

static void adjust_morse_speed(int delta) {
    int next = (int)morseSpeedPercent + delta;
    next = constrain(next, (int)MORSE_MIN_SPEED_PERCENT, (int)MORSE_MAX_SPEED_PERCENT);
    uint8_t speed = (uint8_t)next;
    if (speed == morseSpeedPercent) return;
    morseSpeedPercent = speed;
    save_stations();
}

static void adjust_scanner_id(int delta) {
    int next = (int)scannerId + delta;
    next = constrain(next, (int)SCANNER_ID_MIN, (int)SCANNER_ID_MAX);
    uint8_t value = (uint8_t)next;
    if (value == scannerId) return;
    scannerId = value;
    save_stations();
}

static void init_ble_scanner() {
    if (bleScan != nullptr) return;

    BLEDevice::init("cyd-gb-scanner");
    bleScan = BLEDevice::getScan();
    bleScan->setActiveScan(true);
    bleScan->setInterval(100);
    bleScan->setWindow(80);
}

static void scan_selected_beacon() {
    init_ble_scanner();
    if (bleScan == nullptr) {
        stations[selectedStationIndex].rssi = -127;
        stations[selectedStationIndex].rssiValid = false;
        return;
    }

    String target = String(stations[selectedStationIndex].mac);
    target.toUpperCase();
    int bestRssi = -127;

    BLEScanResults results = bleScan->start(1, false);
    int deviceCount = results.getCount();
    for (int i = 0; i < deviceCount; ++i) {
        BLEAdvertisedDevice device = results.getDevice(i);
        String foundMac = String(device.getAddress().toString().c_str());
        foundMac.toUpperCase();
        if (foundMac == target) {
            bestRssi = max(bestRssi, device.getRSSI());
        }
    }
    bleScan->clearResults();

    stations[selectedStationIndex].rssi = bestRssi;
    stations[selectedStationIndex].rssiValid = bestRssi > -127;
    lastScanMs = millis();
    needsScan = false;
}

static void draw_list_row(int index, bool selected) {
    int top = 46 + (index * 52);
    uint16_t bg = selected ? COLOR_ITEM_HL_BG : COLOR_ITEM_BG;
    uint16_t fg = selected ? COLOR_ITEM_HL_TEXT : COLOR_ITEM_TEXT;

    tft.fillRoundRect(8, top, SCREEN_WIDTH - 16, 46, 6, bg);
    tft.drawRoundRect(8, top, SCREEN_WIDTH - 16, 46, 6, selected ? COLOR_ITEM_HL_TEXT : 0x7BEF);

    tft.setTextColor(fg, bg);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(stations[index].name, 16, top + 13, 2);
    tft.drawString(stations[index].mac, 16, top + 30, 1);

    uint16_t badgeColor = stations[index].rssiValid ? rssi_to_color(stations[index].rssi) : 0x7BEF;
    tft.fillRoundRect(SCREEN_WIDTH - 72, top + 8, 58, 30, 4, badgeColor);
    tft.setTextColor(TFT_BLACK, badgeColor);
    tft.setTextDatum(MC_DATUM);
    if (stations[index].rssiValid) {
        char rssiText[12];
        snprintf(rssiText, sizeof(rssiText), "%d", stations[index].rssi);
        tft.drawString(rssiText, SCREEN_WIDTH - 43, top + 23, 2);
    } else {
        tft.drawString("--", SCREEN_WIDTH - 43, top + 23, 2);
    }
}

static void draw_list_screen() {
    tft.fillScreen(COLOR_BG);
    draw_header("BT Scanner");
    for (int i = 0; i < 4; ++i) draw_list_row(i, i == selectedListIndex);

    uint16_t morseBg = list_selection_is_morse() ? COLOR_ITEM_HL_BG : 0x0014;
    uint16_t morseFg = list_selection_is_morse() ? COLOR_ITEM_HL_TEXT : TFT_WHITE;
    tft.fillRoundRect(12, 256, SCREEN_WIDTH - 24, 28, 6, morseBg);
    tft.drawRoundRect(12, 256, SCREEN_WIDTH - 24, 28, 6, morseFg);
    tft.setTextColor(morseFg, morseBg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Morse Sender", SCREEN_WIDTH / 2, 270, 2);
    draw_footer("Up/Down Select  A Open  B ROM  LongTouch Edit");
}

static void draw_monitor_screen() {
    tft.fillScreen(COLOR_BG);
    draw_header("Beacon Monitor");

    BeaconStation& station = stations[selectedStationIndex];
    uint16_t cardColor = station.rssiValid ? rssi_to_color(station.rssi) : TFT_RED;

    tft.fillRoundRect(8, 52, SCREEN_WIDTH - 16, 150, 8, cardColor);
    tft.setTextColor(TFT_BLACK, cardColor);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(station.name, 18, 66, 2);
    tft.drawString(station.mac, 18, 92, 1);
    if (station.rssiValid) {
        char rssiText[16];
        snprintf(rssiText, sizeof(rssiText), "RSSI %d dBm", station.rssi);
        tft.drawString(rssiText, 18, 124, 2);
    } else {
        tft.drawString("No signal", 18, 124, 2);
    }

    int barWidth = station.rssiValid ? map(constrain(station.rssi, -100, -40), -100, -40, 0, SCREEN_WIDTH - 40) : 0;
    tft.fillRoundRect(18, 164, SCREEN_WIDTH - 36, 18, 4, TFT_DARKGREY);
    tft.fillRoundRect(18, 164, barWidth, 18, 4, station.rssiValid ? rssi_to_color(station.rssi) : TFT_RED);

    tft.fillRoundRect(8, 218, SCREEN_WIDTH - 16, 56, 8, 0x0014);
    tft.setTextColor(TFT_WHITE, 0x0014);
    tft.drawString("B = List", 18, 235, 1);
    tft.drawString("A = Rescan", 18, 252, 1);
    tft.drawString("Touch = Morse", 128, 243, 1);
    draw_footer("Start+Select = ROM Menu");
}

static void draw_edit_menu_screen() {
    tft.fillScreen(COLOR_BG);
    draw_header("Edit Station");

    BeaconStation& station = stations[selectedStationIndex];
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString(station.name, 12, 46, 2);
    tft.drawString(station.mac, 12, 72, 1);

    const char* labels[7] = {"Edit Name", "Edit MAC", "Morse", "Morse Speed", "Morse Kalibrierung", "LDR Kalibrierung", "Back"};
    for (int i = 0; i < 7; ++i) {
        int top = 78 + (i * 34);
        uint16_t bg = (i == selectedEditChoice) ? COLOR_ITEM_HL_BG : COLOR_ITEM_BG;
        uint16_t fg = (i == selectedEditChoice) ? COLOR_ITEM_HL_TEXT : COLOR_ITEM_TEXT;
        tft.fillRoundRect(12, top, SCREEN_WIDTH - 24, 28, 6, bg);
        tft.drawRoundRect(12, top, SCREEN_WIDTH - 24, 28, 6, fg);
        tft.setTextColor(fg, bg);
        tft.setTextDatum(MC_DATUM);
        if (i == 3) {
            char speedLabel[28];
            snprintf(speedLabel, sizeof(speedLabel), "Morse Speed: %u%%", morseSpeedPercent);
            tft.drawString(speedLabel, SCREEN_WIDTH / 2, top + 14, 2);
        } else {
            tft.drawString(labels[i], SCREEN_WIDTH / 2, top + 14, 2);
        }
    }

    draw_footer("Up/Down, L/R Speed, A, B");
}

static void draw_morse_menu_screen() {
    tft.fillScreen(COLOR_BG);
    draw_header("Morse Sender");

    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Codewort", 12, 52, 2);
    tft.fillRoundRect(12, 66, SCREEN_WIDTH - 24, 34, 6, 0x7BEF);
    tft.setTextColor(TFT_BLACK, 0x7BEF);
    tft.drawString(morseWord[0] ? morseWord : "-", 18, 83, 2);

    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Scanner vor Receiver halten.", 12, 110, 1);
    char scannerLabel[28];
    snprintf(scannerLabel, sizeof(scannerLabel), "Scanner-ID: %u", scannerId);
    tft.drawString(scannerLabel, 12, 124, 1);

    const char* labels[3] = {"Wort bearbeiten", "Senden", "Zuruck"};
    for (int i = 0; i < 3; ++i) {
        int top = 126 + (i * 48);
        uint16_t bg = (i == selectedMorseChoice) ? COLOR_ITEM_HL_BG : 0x0014;
        uint16_t fg = (i == selectedMorseChoice) ? COLOR_ITEM_HL_TEXT : TFT_WHITE;
        tft.fillRoundRect(12, top, SCREEN_WIDTH - 24, 38, 6, bg);
        tft.drawRoundRect(12, top, SCREEN_WIDTH - 24, 38, 6, COLOR_ITEM_HL_TEXT);
        tft.setTextColor(fg, bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(labels[i], SCREEN_WIDTH / 2, top + 19, 2);
    }

    draw_footer("Buttons: Up/Down, A/B, L/R ID");
}

static void init_keyboard_buffer(const char* text, uint8_t maxLen, KeyboardMode mode, uint8_t fieldIndex) {
    keyboardState.active = true;
    keyboardState.mode = mode;
    keyboardState.stationIndex = (uint8_t)selectedStationIndex;
    keyboardState.fieldIndex = fieldIndex;
    keyboardState.maxLen = maxLen;
    strncpy(keyboardState.buffer, text, sizeof(keyboardState.buffer) - 1);
    keyboardState.buffer[sizeof(keyboardState.buffer) - 1] = 0;
}

static void append_keyboard_char(char c) {
    size_t len = strlen(keyboardState.buffer);
    if (len >= keyboardState.maxLen) return;
    keyboardState.buffer[len] = c;
    keyboardState.buffer[len + 1] = 0;
}

static void keyboard_backspace() {
    size_t len = strlen(keyboardState.buffer);
    if (len > 0) keyboardState.buffer[len - 1] = 0;
}

static void keyboard_clear() {
    keyboardState.buffer[0] = 0;
}

static void keyboard_switch_layout() {
    keyboardState.mode = (keyboardState.mode == KB_ALPHA) ? KB_MAC : KB_ALPHA;
}

static void save_keyboard_buffer() {
    if (keyboardState.fieldIndex == 2) {
        strncpy(morseWord, keyboardState.buffer, sizeof(morseWord) - 1);
        morseWord[sizeof(morseWord) - 1] = 0;
        normalize_morse_word(morseWord);
    } else {
        BeaconStation& station = stations[keyboardState.stationIndex];
        if (keyboardState.fieldIndex == 0) {
            strncpy(station.name, keyboardState.buffer, sizeof(station.name) - 1);
            station.name[sizeof(station.name) - 1] = 0;
        } else {
            strncpy(station.mac, keyboardState.buffer, sizeof(station.mac) - 1);
            station.mac[sizeof(station.mac) - 1] = 0;
            normalize_mac(station.mac);
        }
    }
    save_stations();
}

static void wait_for_touch_release() {
    uint16_t x = 0;
    uint16_t y = 0;
    while (read_touch(&x, &y)) delay(20);
}

static void wait_for_touch_tap() {
    uint16_t x = 0;
    uint16_t y = 0;
    while (!read_touch(&x, &y)) {
        (void)read_buttons();
        delay(20);
    }
    while (read_touch(&x, &y)) {
        (void)read_buttons();
        delay(20);
    }
    delay(40);
}

static uint8_t load_configured_backlight_level() {
    uint8_t palette = 0;
    uint8_t frameSkip = 0;
    uint8_t brightness = 255;
    (void)touch_load_settings(&palette, &frameSkip, &brightness, nullptr, nullptr);
    return brightness;
}

static uint8_t load_configured_morse_speed_percent() {
    return constrain(morseSpeedPercent, MORSE_MIN_SPEED_PERCENT, MORSE_MAX_SPEED_PERCENT);
}

static unsigned long morse_unit_ms() {
    uint8_t morseSpeed = load_configured_morse_speed_percent();
    return max(1UL, (MORSE_BASE_UNIT_MS * 100UL) / morseSpeed);
}

static unsigned long morse_scaled_ms(unsigned long baseMs) {
    uint8_t morseSpeed = load_configured_morse_speed_percent();
    return max(1UL, (baseMs * 100UL) / morseSpeed);
}

static void run_ldr_calibration_preview() {
    wait_for_touch_release();

    uint8_t backlightLevel = load_configured_backlight_level();

    tft.fillScreen(TFT_BLACK);
    display_set_backlight(0);
    draw_footer("Touch = Weiss  nochmal = Zuruck");

    wait_for_touch_tap();

    display_set_backlight(backlightLevel);
    tft.fillScreen(TFT_WHITE);
    draw_footer("Touch = Zuruck ins Menu");

    wait_for_touch_tap();
}

static void show_pre_calibration_wait_screen() {
    tft.fillScreen(COLOR_BG);
    draw_header("Touch Setup");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Kalibrierung startet", SCREEN_WIDTH / 2, 120, 2);
    tft.drawString("Bitte kurz loslassen...", SCREEN_WIDTH / 2, 150, 2);
    draw_footer("Warte kurz");

    uint32_t start = millis();
    while (millis() - start < 900) {
        uint16_t x = 0;
        uint16_t y = 0;
        (void)read_touch(&x, &y);
        (void)read_buttons();
        delay(20);
    }

    wait_for_touch_release();

    start = millis();
    while (millis() - start < 250) {
        (void)read_buttons();
        delay(20);
    }
}

static bool morse_stop_requested() {
    uint16_t buttons = read_buttons();
    return (buttons & GB_BTN_B) != 0;
}

static bool morse_wait_cancelable(unsigned long durationMs) {
    unsigned long startMs = millis();
    while (millis() - startMs < durationMs) {
        if (morse_stop_requested()) return false;
        delay(10);
    }
    return true;
}

static bool morse_transmit_pulse(uint8_t onUnits, uint8_t offUnits) {
    tft.fillScreen(TFT_WHITE);
    if (!morse_wait_cancelable(morse_unit_ms() * onUnits)) return false;

    tft.fillScreen(TFT_BLACK);
    if (offUnits == 0) return true;
    return morse_wait_cancelable(morse_unit_ms() * offUnits);
}

static bool transmit_morse_calibration_pattern() {
    for (int i = 0; i < 3; ++i) {
        if (!morse_transmit_pulse(1, 1)) return false;
    }

    if (!morse_wait_cancelable(morse_unit_ms() * 3)) return false;

    for (int i = 0; i < 3; ++i) {
        if (!morse_transmit_pulse(3, 1)) return false;
    }

    return morse_wait_cancelable(morse_unit_ms() * 5);
}

static void run_morse_calibration_loop() {
    block_touch_until_release();
    wait_for_button_release();

    tft.fillScreen(TFT_BLACK);
    draw_header("Morse Kalibrierung");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("3x kurz, 3x lang", SCREEN_WIDTH / 2, 118, 2);
    tft.drawString("Touch ist deaktiviert", SCREEN_WIDTH / 2, 150, 2);
    tft.drawString("B startet und stoppt", SCREEN_WIDTH / 2, 182, 2);
    draw_footer("Mit B fortfahren");

    while ((read_buttons() & GB_BTN_B) == 0) delay(20);
    wait_for_button_release();

    tft.fillScreen(TFT_BLACK);
    draw_header("Morse Kalibrierung");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Laeuft bis B", SCREEN_WIDTH / 2, 126, 2);
    tft.drawString("3x kurz / 3x lang", SCREEN_WIDTH / 2, 158, 2);
    draw_footer("B = Zurueck");

    while (true) {
        if (!transmit_morse_calibration_pattern()) break;
        if (!morse_wait_cancelable(morse_unit_ms() * 3)) break;
    }

    block_touch_until_release();
    draw_morse_menu_screen();
}

static bool transmit_morse_frame(const char* word) {
    for (int i = 0; i < 5; ++i) {
        if (!morse_transmit_pulse(1, 1)) return false;
    }
    if (!morse_wait_cancelable(morse_unit_ms() * 5)) return false;

    size_t wordLen = strlen(word);
    for (size_t i = 0; i < wordLen; ++i) {
        const char* code = morse_code_for_char(word[i]);
        if (code == nullptr) continue;
        size_t codeLen = strlen(code);
        for (size_t j = 0; j < codeLen; ++j) {
            if (!morse_transmit_pulse(code[j] == '.' ? 1 : 3, 1)) return false;
        }
        if (i + 1 < wordLen && !morse_wait_cancelable(morse_unit_ms() * 2)) return false;
    }

    if (!morse_wait_cancelable(morse_unit_ms() * 5)) return false;

    for (int i = 0; i < 5; ++i) {
        if (!morse_transmit_pulse(5, 1)) return false;
    }

    return true;
}

static bool run_fake_morse_activity() {
    bool white = false;
    unsigned long startMs = millis();
    unsigned long pulseMs = max(80UL, morse_unit_ms());

    while (millis() - startMs < MORSE_FAKE_ACTIVITY_MS) {
        if (morse_stop_requested()) return false;
        tft.fillScreen(white ? TFT_WHITE : TFT_BLACK);
        white = !white;
        if (!morse_wait_cancelable(pulseMs)) return false;
    }

    tft.fillScreen(TFT_BLACK);
    return true;
}

static bool send_password_via_ble(const char* payload, char* errorText, size_t errorTextLen) {
    init_ble_scanner();
    if (bleScan == nullptr) {
        snprintf(errorText, errorTextLen, "BLE nicht bereit");
        return false;
    }

    BLEUUID serviceUuid(kBleServiceUuid);
    BLEAdvertisedDevice* bestDevice = nullptr;
    int bestRssi = -127;

    BLEScanResults results = bleScan->start(4, false);
    int deviceCount = results.getCount();
    for (int i = 0; i < deviceCount; ++i) {
        BLEAdvertisedDevice device = results.getDevice(i);
        if (!device.haveServiceUUID()) continue;
        if (!device.isAdvertisingService(serviceUuid)) continue;
        if (device.getRSSI() > bestRssi) {
            bestRssi = device.getRSSI();
            bestDevice = new BLEAdvertisedDevice(device);
        }
    }
    bleScan->clearResults();

    if (bestDevice == nullptr) {
        snprintf(errorText, errorTextLen, "Beacon nicht gefunden");
        return false;
    }

    BLEClient* client = BLEDevice::createClient();
    if (client == nullptr) {
        delete bestDevice;
        snprintf(errorText, errorTextLen, "Client Fehler");
        return false;
    }

    bool success = false;
    do {
        if (!client->connect(bestDevice)) {
            snprintf(errorText, errorTextLen, "Connect fehlgeschlagen");
            break;
        }

        BLERemoteService* service = client->getService(serviceUuid);
        if (service == nullptr) {
            snprintf(errorText, errorTextLen, "Service fehlt");
            break;
        }

        BLERemoteCharacteristic* passwordChar = service->getCharacteristic(BLEUUID(kBlePasswordCharUuid));
        if (passwordChar == nullptr) {
            snprintf(errorText, errorTextLen, "Characteristic fehlt");
            break;
        }

        if (!passwordChar->canWrite() && !passwordChar->canWriteNoResponse()) {
            snprintf(errorText, errorTextLen, "Write nicht erlaubt");
            break;
        }

        bool useResponse = passwordChar->canWrite();
        passwordChar->writeValue((uint8_t*)payload, strlen(payload), useResponse);
        success = true;
    } while (false);

    if (client->isConnected()) client->disconnect();
    delete client;
    delete bestDevice;

    return success;
}

static void run_morse_sender() {
    char word[16] = {0};
    strncpy(word, morseWord, sizeof(word) - 1);
    word[sizeof(word) - 1] = 0;
    normalize_morse_word(word);
    if (word[0] == 0) {
        draw_morse_menu_screen();
        return;
    }

    block_touch_until_release();
    wait_for_button_release();

    tft.fillScreen(TFT_BLACK);
    draw_header("Morse Sender");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Geraete positionieren", SCREEN_WIDTH / 2, 112, 2);
    tft.drawString(word, SCREEN_WIDTH / 2, 146, 4);
    tft.drawString("Touch ist deaktiviert", SCREEN_WIDTH / 2, 188, 2);
    tft.drawString("B startet und stoppt", SCREEN_WIDTH / 2, 212, 2);
    draw_footer("Mit B fortfahren");

    while ((read_buttons() & GB_BTN_B) == 0) delay(20);
    wait_for_button_release();

    tft.fillScreen(TFT_BLACK);
    draw_header("Morse Sender");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Kalibrierung + Sendung", SCREEN_WIDTH / 2, 112, 2);
    tft.drawString(word, SCREEN_WIDTH / 2, 146, 4);
    tft.drawString("Nur B stoppt", SCREEN_WIDTH / 2, 194, 2);
    draw_footer("Start in 2 Sekunden");

    if (!morse_wait_cancelable(MORSE_PREP_DELAY_MS)) {
        block_touch_until_release();
        draw_morse_menu_screen();
        return;
    }

    if (!run_fake_morse_activity()) {
        block_touch_until_release();
        draw_morse_menu_screen();
        return;
    }

    char payload[40];
    snprintf(payload, sizeof(payload), "%u|%s", scannerId, word);

    char errorText[40] = {0};
    bool sent = send_password_via_ble(payload, errorText, sizeof(errorText));

    tft.fillScreen(COLOR_BG);
    draw_header(sent ? "Morse Gesendet" : "Morse Fehler");
    tft.setTextDatum(MC_DATUM);
    tft.fillRoundRect(12, 94, SCREEN_WIDTH - 24, 118, 8, sent ? 0x03E0 : 0x7800);
    tft.setTextColor(TFT_WHITE, sent ? 0x03E0 : 0x7800);
    tft.drawString(sent ? "Uebertragung gestartet" : "Senden fehlgeschlagen", SCREEN_WIDTH / 2, 126, 2);
    tft.drawString(sent ? payload : errorText, SCREEN_WIDTH / 2, 152, 2);
    tft.drawString(sent ? "Beacon prueft jetzt Passwort" : "Bitte erneut versuchen", SCREEN_WIDTH / 2, 178, 1);
    draw_footer("Weiter mit B oder nach 2s");

    unsigned long resultStart = millis();
    while (millis() - resultStart < 2000) {
        if (morse_stop_requested()) break;
        delay(20);
    }

    block_touch_until_release();
    draw_morse_menu_screen();
}

static void draw_keyboard_row(const KeyDef* keys, int count, int top, int gap) {
    int availableWidth = SCREEN_WIDTH - 16 - (gap * (count - 1));
    int keyWidth = availableWidth / count;
    int left = 8;
    for (int i = 0; i < count; ++i) {
        int x = left + i * (keyWidth + gap);
        tft.fillRoundRect(x, top, keyWidth, 28, 4, 0x0014);
        tft.drawRoundRect(x, top, keyWidth, 28, 4, COLOR_ITEM_HL_TEXT);
        tft.setTextColor(TFT_WHITE, 0x0014);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(keys[i].label, x + keyWidth / 2, top + 14, 1);
    }
}

static void draw_keyboard_screen(const char* title) {
    tft.fillScreen(COLOR_BG);
    draw_header(title);

    tft.fillRoundRect(8, 44, SCREEN_WIDTH - 16, 34, 6, 0x7BEF);
    tft.setTextColor(TFT_BLACK, 0x7BEF);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(keyboardState.buffer, 14, 61, 2);

    if (keyboardState.mode == KB_ALPHA) {
        draw_keyboard_row(alphaRow1, 10, 92, 2);
        draw_keyboard_row(alphaRow2, 9, 126, 2);
        draw_keyboard_row(alphaRow3, 7, 160, 2);
        draw_keyboard_row(alphaRow4, 4, 204, 4);
    } else {
        draw_keyboard_row(macRow1, 10, 102, 2);
        draw_keyboard_row(macRow2, 7, 136, 2);
        draw_keyboard_row(macRow3, 4, 180, 4);
    }

    draw_footer("Tap keys  B cancel  A save");
}

static void draw_name_editor() { draw_keyboard_screen("Edit Name"); }
static void draw_mac_editor() { draw_keyboard_screen("Edit MAC"); }
static void draw_morse_editor() { draw_keyboard_screen("Morse Wort"); }

static bool keyboard_tap(uint16_t x, uint16_t y) {
    const KeyDef* rowsAlpha[] = {alphaRow1, alphaRow2, alphaRow3, alphaRow4};
    const int rowCountsAlpha[] = {10, 9, 7, 4};
    const int rowTopAlpha[] = {92, 126, 160, 204};

    const KeyDef* rowsMac[] = {macRow1, macRow2, macRow3};
    const int rowCountsMac[] = {10, 7, 4};
    const int rowTopMac[] = {102, 136, 180};

    const KeyDef** rows = nullptr;
    const int* counts = nullptr;
    const int* tops = nullptr;
    int rowCount = 0;
    if (keyboardState.mode == KB_ALPHA) {
        rows = rowsAlpha;
        counts = rowCountsAlpha;
        tops = rowTopAlpha;
        rowCount = 4;
    } else {
        rows = rowsMac;
        counts = rowCountsMac;
        tops = rowTopMac;
        rowCount = 3;
    }

    for (int row = 0; row < rowCount; ++row) {
        int count = counts[row];
        int top = tops[row];
        int availableWidth = SCREEN_WIDTH - 16 - (2 * (count - 1));
        int keyWidth = availableWidth / count;
        int left = 8;

        for (int col = 0; col < count; ++col) {
            int keyLeft = left + col * (keyWidth + 2);
            int keyRight = keyLeft + keyWidth;
            int keyBottom = top + 28;
            if (x >= keyLeft && x <= keyRight && y >= top && y <= keyBottom) {
                const KeyDef& key = rows[row][col];
                switch (key.action) {
                    case KEY_CHAR:
                        append_keyboard_char(key.value);
                        break;
                    case KEY_SPACE:
                        append_keyboard_char(' ');
                        break;
                    case KEY_BACKSPACE:
                        keyboard_backspace();
                        break;
                    case KEY_SAVE:
                        save_keyboard_buffer();
                        if (scannerView == VIEW_MORSE_EDIT) {
                            scannerView = VIEW_MORSE_MENU;
                            draw_morse_menu_screen();
                        } else {
                            scannerView = VIEW_EDIT_MENU;
                            draw_edit_menu_screen();
                        }
                        return true;
                    case KEY_SWITCH:
                        keyboard_switch_layout();
                        draw_keyboard_screen(scannerView == VIEW_EDIT_NAME ? "Edit Name" :
                                             (scannerView == VIEW_EDIT_MAC ? "Edit MAC" : "Morse Wort"));
                        return true;
                    case KEY_CLEAR:
                        keyboard_clear();
                        break;
                }
                draw_keyboard_screen(scannerView == VIEW_EDIT_NAME ? "Edit Name" :
                                     (scannerView == VIEW_EDIT_MAC ? "Edit MAC" : "Morse Wort"));
                return true;
            }
        }
    }

    return false;
}

static void enter_list() {
    scannerView = VIEW_LIST;
    draw_list_screen();
}

static void enter_monitor() {
    scannerView = VIEW_MONITOR;
    needsScan = true;
    draw_monitor_screen();
}

static void enter_edit_menu() {
    scannerView = VIEW_EDIT_MENU;
    selectedEditChoice = 0;
    draw_edit_menu_screen();
}

static void enter_name_editor() {
    scannerView = VIEW_EDIT_NAME;
    init_keyboard_buffer(stations[selectedStationIndex].name, sizeof(stations[selectedStationIndex].name) - 1, KB_ALPHA, 0);
    draw_name_editor();
}

static void enter_mac_editor() {
    scannerView = VIEW_EDIT_MAC;
    init_keyboard_buffer(stations[selectedStationIndex].mac, sizeof(stations[selectedStationIndex].mac) - 1, KB_MAC, 1);
    draw_mac_editor();
}

static void enter_morse_menu() {
    scannerView = VIEW_MORSE_MENU;
    selectedMorseChoice = 0;
    draw_morse_menu_screen();
}

static void enter_morse_editor() {
    scannerView = VIEW_MORSE_EDIT;
    init_keyboard_buffer(morseWord, MORSE_WORD_MAX_LEN, KB_ALPHA, 2);
    draw_morse_editor();
}

static bool touch_tap(uint16_t x, uint16_t y) {
    switch (scannerView) {
        case VIEW_LIST:
            for (int i = 0; i < 4; ++i) {
                int top = 46 + (i * 52);
                if (x >= 8 && x <= SCREEN_WIDTH - 80 && y >= top && y <= top + 46) {
                    selectedStationIndex = i;
                    selectedListIndex = i;
                    enter_monitor();
                    return true;
                }
                if (x >= SCREEN_WIDTH - 72 && x <= SCREEN_WIDTH - 14 && y >= top + 8 && y <= top + 38) {
                    selectedStationIndex = i;
                    selectedListIndex = i;
                    enter_morse_menu();
                    return true;
                }
            }
            if (x >= 12 && x <= SCREEN_WIDTH - 12 && y >= 256 && y <= 284) {
                selectedListIndex = 4;
                enter_morse_menu();
                return true;
            }
            break;
        case VIEW_MONITOR:
            if (x >= 12 && x <= 120 && y >= 220 && y <= 270) {
                enter_morse_menu();
                return true;
            }
            if (x >= 124 && x <= 228 && y >= 220 && y <= 270) {
                needsScan = true;
                scan_selected_beacon();
                draw_monitor_screen();
                return true;
            }
            enter_list();
            return true;
        case VIEW_EDIT_MENU:
            if (y >= 78 && y <= 106) {
                selectedEditChoice = 0;
                enter_name_editor();
                return true;
            }
            if (y >= 112 && y <= 140) {
                selectedEditChoice = 1;
                enter_mac_editor();
                return true;
            }
            if (y >= 146 && y <= 174) {
                selectedEditChoice = 2;
                enter_morse_menu();
                return true;
            }
            if (y >= 180 && y <= 208) {
                selectedEditChoice = 3;
                adjust_morse_speed(10);
                draw_edit_menu_screen();
                return true;
            }
            if (y >= 214 && y <= 242) {
                selectedEditChoice = 4;
                run_morse_calibration_loop();
                draw_edit_menu_screen();
                return true;
            }
            if (y >= 248 && y <= 276) {
                selectedEditChoice = 5;
                run_ldr_calibration_preview();
                draw_edit_menu_screen();
                return true;
            }
            if (y >= 282 && y <= 310) {
                selectedEditChoice = 6;
                enter_list();
                return true;
            }
            break;
        case VIEW_MORSE_MENU:
            break;
        case VIEW_EDIT_NAME:
        case VIEW_EDIT_MAC:
        case VIEW_MORSE_EDIT:
            return keyboard_tap(x, y);
    }
    return false;
}

static bool handle_buttons() {
    uint16_t buttons = read_buttons();
    uint16_t changed = buttons ^ lastButtons;
    bool exitToMenu = false;

    if ((buttons & (GB_BTN_START | GB_BTN_SELECT)) == (GB_BTN_START | GB_BTN_SELECT)) {
        lastButtons = buttons;
        return true;
    }

    if (scannerView == VIEW_LIST) {
        if ((changed & GB_BTN_UP) && btn_pressed_edge(buttons, lastButtons, GB_BTN_UP)) {
            selectedListIndex = (selectedListIndex + 4) % 5;
            draw_list_screen();
        }
        if ((changed & GB_BTN_DOWN) && btn_pressed_edge(buttons, lastButtons, GB_BTN_DOWN)) {
            selectedListIndex = (selectedListIndex + 1) % 5;
            draw_list_screen();
        }
        if ((changed & GB_BTN_A) && btn_pressed_edge(buttons, lastButtons, GB_BTN_A)) {
            if (list_selection_is_morse()) {
                enter_morse_menu();
            } else {
                selectedStationIndex = selectedListIndex;
                enter_monitor();
            }
        }
        if ((changed & GB_BTN_B) && btn_pressed_edge(buttons, lastButtons, GB_BTN_B)) {
            exitToMenu = true;
        }
        if ((changed & GB_BTN_RIGHT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_RIGHT)) {
            selectedListIndex = 4;
            enter_morse_menu();
        }
    } else if (scannerView == VIEW_MONITOR) {
        if ((changed & GB_BTN_B) && btn_pressed_edge(buttons, lastButtons, GB_BTN_B)) {
            enter_list();
        }
        if ((changed & GB_BTN_A) && btn_pressed_edge(buttons, lastButtons, GB_BTN_A)) {
            needsScan = true;
            scan_selected_beacon();
            draw_monitor_screen();
        }
        if ((changed & GB_BTN_RIGHT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_RIGHT)) {
            enter_morse_menu();
        }
    } else if (scannerView == VIEW_EDIT_MENU) {
        if ((changed & GB_BTN_UP) && btn_pressed_edge(buttons, lastButtons, GB_BTN_UP)) {
            selectedEditChoice = (selectedEditChoice + 6) % 7;
            draw_edit_menu_screen();
        }
        if ((changed & GB_BTN_DOWN) && btn_pressed_edge(buttons, lastButtons, GB_BTN_DOWN)) {
            selectedEditChoice = (selectedEditChoice + 1) % 7;
            draw_edit_menu_screen();
        }
        if ((changed & GB_BTN_A) && btn_pressed_edge(buttons, lastButtons, GB_BTN_A)) {
            if (selectedEditChoice == 0) enter_name_editor();
            else if (selectedEditChoice == 1) enter_mac_editor();
            else if (selectedEditChoice == 2) enter_morse_menu();
            else if (selectedEditChoice == 3) {
                adjust_morse_speed(10);
                draw_edit_menu_screen();
            }
            else if (selectedEditChoice == 4) {
                run_morse_calibration_loop();
                draw_edit_menu_screen();
            }
            else if (selectedEditChoice == 5) {
                run_ldr_calibration_preview();
                draw_edit_menu_screen();
            }
            else enter_list();
        }
        if ((changed & GB_BTN_LEFT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_LEFT) && selectedEditChoice == 3) {
            adjust_morse_speed(-10);
            draw_edit_menu_screen();
        }
        if ((changed & GB_BTN_RIGHT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_RIGHT) && selectedEditChoice == 3) {
            adjust_morse_speed(10);
            draw_edit_menu_screen();
        }
        if ((changed & GB_BTN_B) && btn_pressed_edge(buttons, lastButtons, GB_BTN_B)) {
            enter_list();
        }
    } else if (scannerView == VIEW_MORSE_MENU) {
        if ((changed & GB_BTN_UP) && btn_pressed_edge(buttons, lastButtons, GB_BTN_UP)) {
            selectedMorseChoice = (selectedMorseChoice + 2) % 3;
            draw_morse_menu_screen();
        }
        if ((changed & GB_BTN_DOWN) && btn_pressed_edge(buttons, lastButtons, GB_BTN_DOWN)) {
            selectedMorseChoice = (selectedMorseChoice + 1) % 3;
            draw_morse_menu_screen();
        }
        if ((changed & GB_BTN_A) && btn_pressed_edge(buttons, lastButtons, GB_BTN_A)) {
            if (selectedMorseChoice == 0) enter_morse_editor();
            else if (selectedMorseChoice == 1) run_morse_sender();
            else enter_monitor();
        }
        if ((changed & GB_BTN_B) && btn_pressed_edge(buttons, lastButtons, GB_BTN_B)) {
            enter_monitor();
        }
        if ((changed & GB_BTN_LEFT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_LEFT)) {
            adjust_scanner_id(-1);
            draw_morse_menu_screen();
        }
        if ((changed & GB_BTN_RIGHT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_RIGHT)) {
            adjust_scanner_id(1);
            draw_morse_menu_screen();
        }
    } else if (scannerView == VIEW_EDIT_NAME || scannerView == VIEW_EDIT_MAC || scannerView == VIEW_MORSE_EDIT) {
        if ((changed & GB_BTN_A) && btn_pressed_edge(buttons, lastButtons, GB_BTN_A)) {
            save_keyboard_buffer();
            if (scannerView == VIEW_MORSE_EDIT) enter_morse_menu();
            else enter_edit_menu();
        }
        if ((changed & GB_BTN_B) && btn_pressed_edge(buttons, lastButtons, GB_BTN_B)) {
            if (scannerView == VIEW_MORSE_EDIT) enter_morse_menu();
            else enter_edit_menu();
        }
        if ((changed & GB_BTN_LEFT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_LEFT)) {
            keyboard_backspace();
            draw_keyboard_screen(scannerView == VIEW_EDIT_NAME ? "Edit Name" :
                                 (scannerView == VIEW_EDIT_MAC ? "Edit MAC" : "Morse Wort"));
        }
        if ((changed & GB_BTN_RIGHT) && btn_pressed_edge(buttons, lastButtons, GB_BTN_RIGHT)) {
            keyboard_switch_layout();
            draw_keyboard_screen(scannerView == VIEW_EDIT_NAME ? "Edit Name" :
                                 (scannerView == VIEW_EDIT_MAC ? "Edit MAC" : "Morse Wort"));
        }
    }

    lastButtons = buttons;
    return exitToMenu;
}

void bt_scanner_enter() {
    set_default_stations();
    load_stations();
    init_ble_scanner();
    selectedStationIndex = 0;
    selectedListIndex = 0;
    clamp_list_selection();
    selectedEditChoice = 0;
    selectedMorseChoice = 0;
    scannerView = VIEW_LIST;
    needsScan = true;
    touchPressed = false;
    touchLongHandled = false;
    lastButtons = read_buttons();
    normalize_morse_word(morseWord);
    draw_list_screen();
}

bool bt_scanner_loop() {
    if (scannerView == VIEW_MONITOR && (needsScan || millis() - lastScanMs > BT_SCAN_INTERVAL_MS)) {
        scan_selected_beacon();
        draw_monitor_screen();
    }

    uint16_t x = 0;
    uint16_t y = 0;
    bool touching = read_touch(&x, &y);
    if (ignoreTouchUntilRelease) {
        if (!touching) {
            ignoreTouchUntilRelease = false;
            reset_touch_gesture_state();
        }
    } else if (touching) {
        if (!touchPressed) {
            touchPressed = true;
            touchLongHandled = false;
            touchPressStartMs = millis();
            touchPressX = x;
            touchPressY = y;
        } else if (!touchLongHandled &&
                   (scannerView == VIEW_LIST || scannerView == VIEW_MONITOR) &&
                   millis() - touchPressStartMs >= SETTINGS_LONGPRESS_MS) {
            touchLongHandled = true;
            enter_edit_menu();
        } else if (!touchLongHandled &&
                   scannerView == VIEW_EDIT_MENU &&
                   millis() - touchPressStartMs >= SETTINGS_LONGPRESS_MS) {
            touchLongHandled = true;
            show_pre_calibration_wait_screen();
            touch_run_calibration();
            draw_edit_menu_screen();
        }
    } else {
        if (touchPressed && !touchLongHandled) touch_tap(touchPressX, touchPressY);
        touchPressed = false;
        touchLongHandled = false;
    }

    if (handle_buttons()) return true;
    return false;
}

void bt_scanner_shutdown() {
    if (bleScan) {
        bleScan->stop();
        bleScan->clearResults();
        bleScan = nullptr;
    }

    // Release BLE stack memory so emulator init can reclaim heap.
    BLEDevice::deinit(true);
}
