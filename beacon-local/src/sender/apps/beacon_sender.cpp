#include "sender/apps/beacon_sender.h"

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <BLEAdvertising.h>
#include <BLEBeacon.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <esp_gap_ble_api.h>

#if defined(BT_CHEST_BUILD)
#include <ESP32Servo.h>
#endif

extern TFT_eSPI tft;
extern bool touchReadScreen(uint16_t* x, uint16_t* y);

static constexpr int SCREEN_W = 240;
static constexpr int SCREEN_H = 320;

static constexpr uint16_t COLOR_BG = TFT_BLACK;
static constexpr uint16_t COLOR_HEADER_BG = 0x18C3;
static constexpr uint16_t COLOR_HEADER_TEXT = TFT_WHITE;
static constexpr uint16_t COLOR_HEADER_ACCENT = 0x7BEF;
static constexpr uint16_t COLOR_CARD_BG = 0x0014;
static constexpr uint16_t COLOR_CARD_TEXT = TFT_WHITE;
static constexpr uint16_t COLOR_VALUE_BG = 0x7BEF;
static constexpr uint16_t COLOR_VALUE_TEXT = TFT_BLACK;
static constexpr uint16_t COLOR_FOOTER_BG = 0x18C3;
static constexpr uint16_t COLOR_FOOTER_TEXT = 0x7BEF;
static constexpr uint16_t COLOR_BUTTON_BG = 0x0014;
static constexpr uint16_t COLOR_BUTTON_HL_BG = 0x7BEF;
static constexpr uint16_t COLOR_BUTTON_TEXT = TFT_WHITE;
static constexpr uint16_t COLOR_BUTTON_HL_TEXT = TFT_BLACK;

static constexpr unsigned long DISPLAY_TIMEOUT_MS = 20000;
static constexpr uint8_t LDR_PIN = 34;
static constexpr unsigned long MORSE_UNIT_MS = 220;
static constexpr unsigned long SETTINGS_LONGPRESS_MS = 900;
static constexpr int LDR_RAW_MIN = 0;
static constexpr int LDR_RAW_MAX = 4095;
static constexpr uint8_t LDR_DEFAULT_BUFFER_PERCENT = 17;
static constexpr uint8_t LDR_CALIBRATION_SAMPLES = 3;
static constexpr uint8_t LDR_MORSE_FILTER_WEIGHT_NEW = 3;
static constexpr uint8_t LDR_MORSE_FILTER_WEIGHT_TOTAL = 4;
static constexpr unsigned long MORSE_STATE_CHANGE_DEBOUNCE_MS = 12;
static constexpr unsigned long MORSE_UI_REFRESH_MS = 180;
static constexpr int MORSE_INFO_BASE_Y = SCREEN_H - 102;
static constexpr int MORSE_STOP_TOUCH_LEFT = SCREEN_W - 92;
static constexpr int MORSE_STOP_TOUCH_TOP = SCREEN_H - 26;
static constexpr int MORSE_STOP_TOUCH_WIDTH = 84;
static constexpr int MORSE_STOP_TOUCH_HEIGHT = 20;
static constexpr char kBleServiceUuid[] = "7f4ac8ec-9b6e-46f0-b6ee-6f95a2d0a920";
static constexpr char kBlePasswordCharUuid[] = "7f4ac8ec-9b6e-46f0-b6ee-6f95a2d0a921";
#if defined(BT_CHEST_BUILD)
static constexpr uint8_t kDefaultServoPin = 27;
#endif

enum SenderView {
  VIEW_STATUS,
  VIEW_EDIT_MENU,
  VIEW_EDIT_NAME,
  VIEW_EDIT_MAC,
  VIEW_EDIT_MORSE_WORD,
  VIEW_EDIT_CODEWORD,
  VIEW_EDIT_LDR_DARK_RAW,
  VIEW_EDIT_LDR_BRIGHT_RAW,
  VIEW_EDIT_OK_MESSAGE,
  VIEW_EDIT_FAIL_MESSAGE,
  VIEW_EDIT_NEXT_PASSWORD,
  VIEW_EDIT_SCANNER_ID,
  VIEW_EDIT_LDR_BUFFER_PERCENT,
#if defined(BT_CHEST_BUILD)
  VIEW_EDIT_SERVO_PIN,
  VIEW_EDIT_SERVO_ANGLE_OPEN,
  VIEW_EDIT_SERVO_ANGLE_CLOSED,
#endif
  VIEW_LDR_CALIBRATION,
  VIEW_MORSE_READY,
  VIEW_MORSE_LISTEN,
  VIEW_MORSE_RESULT
};

enum KeyboardMode {
  KB_ALPHA,
  KB_MAC,
  KB_NUMERIC
};

enum KeyboardAction {
  KEY_CHAR,
  KEY_SPACE,
  KEY_BACKSPACE,
  KEY_SAVE,
  KEY_SWITCH,
  KEY_CLEAR,
  KEY_CANCEL
};

struct KeyDef {
  const char* label;
  KeyboardAction action;
  char value;
};

struct KeyboardState {
  bool active;
  KeyboardMode mode;
  uint8_t fieldIndex;
  char buffer[32];
  uint8_t maxLen;
};

struct SenderConfig {
  char stationName[24];
  char mac[18];
  char morseWord[16];
  char codeword[4];
  char successMessage[32];
  char failMessage[32];
  char nextPassword[32];
  uint8_t allowedScannerId;
  int ldrDarkRaw;
  int ldrBrightRaw;
  uint8_t ldrBufferPercent;
#if defined(BT_CHEST_BUILD)
  uint8_t servoPin;
  uint8_t servoAngleOpen;
  uint8_t servoAngleClosed;
#endif
};

struct MorseCodeEntry {
  char ch;
  const char* code;
};

struct MorseRxState {
  bool listening;
  bool frameStarted;
  bool signalActive;
  int lastRaw;
  int filteredRawQ8;
  unsigned long stateSince;
  unsigned long lastDrawMs;
  char currentSymbol[8];
  uint8_t currentSymbolLen;
  char receivedWord[16];
  uint8_t receivedLen;
  uint8_t preambleCount;
  uint8_t startPatternDashCount;
  uint8_t postambleCount;
};

static Preferences senderPrefs;
static BLEAdvertising* advertising = nullptr;
static BLEServer* bleServer = nullptr;
static BLEService* bleService = nullptr;
static BLECharacteristic* passwordCharacteristic = nullptr;

static SenderConfig config;
static KeyboardState keyboardState{};
static SenderView senderView = VIEW_STATUS;
static int selectedEditChoice = 0;
static bool editMenuInMorseSubmenu = false;
#if defined(BT_CHEST_BUILD)
static bool editMenuInChestSubmenu = false;
static Servo chestServo;
static uint8_t attachedServoPin = 255;
#endif

enum LdrCalibrationStep {
  LDR_CALIBRATION_DARK,
  LDR_CALIBRATION_BRIGHT,
  LDR_CALIBRATION_DONE
};

struct LdrCalibrationState {
  bool active;
  LdrCalibrationStep step;
  int darkRaw;
  int brightRaw;
  int darkSamples[LDR_CALIBRATION_SAMPLES];
  int brightSamples[LDR_CALIBRATION_SAMPLES];
  uint8_t sampleIndex;
  int lastRaw;
  unsigned long lastDrawMs;
};

static bool displayAwake = true;
static bool touchPressed = false;
static bool touchLongHandled = false;
static uint16_t touchPressX = 0;
static uint16_t touchPressY = 0;
static unsigned long touchPressStartMs = 0;
static unsigned long lastInteractionMs = 0;
static bool btReceiveActive = false;
static volatile bool btResultPending = false;
static bool btPendingSuccess = false;
static uint8_t btPendingSenderId = 0;
static bool btPendingSenderIdPresent = false;
static char btPendingPassword[32] = {0};
static char btPendingRawPayload[64] = {0};

static MorseRxState morseRx{};
static bool morseResultSuccess = false;
static char morseResultWord[16] = {0};
static LdrCalibrationState ldrCalibration{};
static bool morseListenUiInitialized = false;
static int morseListenUiRaw = -1;
static char morseListenUiRx[16] = {0};

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
  {"SPACE", KEY_SPACE, ' '}, {"BKSP", KEY_BACKSPACE, 0}, {"123", KEY_SWITCH, 0}, {"ESC", KEY_CANCEL, 0}, {"OK", KEY_SAVE, 0}
};

static const KeyDef macRow1[] = {
  {"0", KEY_CHAR, '0'}, {"1", KEY_CHAR, '1'}, {"2", KEY_CHAR, '2'}, {"3", KEY_CHAR, '3'}, {"4", KEY_CHAR, '4'},
  {"5", KEY_CHAR, '5'}, {"6", KEY_CHAR, '6'}, {"7", KEY_CHAR, '7'}, {"8", KEY_CHAR, '8'}, {"9", KEY_CHAR, '9'}
};
static const KeyDef macRow2[] = {
  {"A", KEY_CHAR, 'A'}, {"B", KEY_CHAR, 'B'}, {"C", KEY_CHAR, 'C'}, {"D", KEY_CHAR, 'D'}, {"E", KEY_CHAR, 'E'}, {"F", KEY_CHAR, 'F'}, {":", KEY_CHAR, ':'}
};
static const KeyDef macRow3[] = {
  {"BKSP", KEY_BACKSPACE, 0}, {"ABC", KEY_SWITCH, 0}, {"CLR", KEY_CLEAR, 0}, {"ESC", KEY_CANCEL, 0}, {"OK", KEY_SAVE, 0}
};

static const KeyDef numericRow1[] = {
  {"1", KEY_CHAR, '1'}, {"2", KEY_CHAR, '2'}, {"3", KEY_CHAR, '3'}
};
static const KeyDef numericRow2[] = {
  {"4", KEY_CHAR, '4'}, {"5", KEY_CHAR, '5'}, {"6", KEY_CHAR, '6'}
};
static const KeyDef numericRow3[] = {
  {"7", KEY_CHAR, '7'}, {"8", KEY_CHAR, '8'}, {"9", KEY_CHAR, '9'}
};
static const KeyDef numericRow4[] = {
  {"CLR", KEY_CLEAR, 0}, {"0", KEY_CHAR, '0'}, {"BKSP", KEY_BACKSPACE, 0}, {"ESC", KEY_CANCEL, 0}, {"OK", KEY_SAVE, 0}
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
static void normalizeMorseWord(char* text);

static void trimInPlace(char* text) {
  if (text == nullptr) {
    return;
  }

  size_t len = strlen(text);
  size_t start = 0;
  while (start < len && isspace(static_cast<unsigned char>(text[start]))) {
    ++start;
  }

  while (len > start && isspace(static_cast<unsigned char>(text[len - 1]))) {
    --len;
  }

  if (start > 0 || len < strlen(text)) {
    size_t out = 0;
    for (size_t i = start; i < len; ++i) {
      text[out++] = text[i];
    }
    text[out] = 0;
  }
}

static bool parseBtPasswordPayload(const char* payload, uint8_t* senderId, bool* senderIdPresent, char* password, size_t passwordSize) {
  if (payload == nullptr || senderId == nullptr || senderIdPresent == nullptr || password == nullptr || passwordSize == 0) {
    return false;
  }

  *senderId = 0;
  *senderIdPresent = false;
  password[0] = 0;

  char buffer[64];
  strncpy(buffer, payload, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = 0;
  trimInPlace(buffer);
  if (buffer[0] == 0) {
    return false;
  }

  char* separator = strchr(buffer, '|');
  if (separator == nullptr) {
    strncpy(password, buffer, passwordSize - 1);
    password[passwordSize - 1] = 0;
    trimInPlace(password);
    return password[0] != 0;
  }

  *separator = 0;
  char* idText = buffer;
  char* pwText = separator + 1;
  trimInPlace(idText);
  trimInPlace(pwText);
  if (idText[0] == 0 || pwText[0] == 0) {
    return false;
  }

  char* endPtr = nullptr;
  long parsedId = strtol(idText, &endPtr, 10);
  if (endPtr == idText || *endPtr != 0 || parsedId < 0 || parsedId > 8) {
    return false;
  }

  *senderId = static_cast<uint8_t>(parsedId);
  *senderIdPresent = true;
  strncpy(password, pwText, passwordSize - 1);
  password[passwordSize - 1] = 0;
  trimInPlace(password);
  return password[0] != 0;
}

static void queueBtPasswordResult(bool success, uint8_t senderId, bool senderIdPresent, const char* password, const char* rawPayload) {
  btPendingSuccess = success;
  btPendingSenderId = senderId;
  btPendingSenderIdPresent = senderIdPresent;
  strncpy(btPendingPassword, password, sizeof(btPendingPassword) - 1);
  btPendingPassword[sizeof(btPendingPassword) - 1] = 0;
  strncpy(btPendingRawPayload, rawPayload, sizeof(btPendingRawPayload) - 1);
  btPendingRawPayload[sizeof(btPendingRawPayload) - 1] = 0;
  btResultPending = true;
}

static void handleBtPasswordPayload(const char* payload) {
  if (!btReceiveActive) {
    return;
  }

  uint8_t senderId = 0;
  bool senderIdPresent = false;
  char password[32];
  if (!parseBtPasswordPayload(payload, &senderId, &senderIdPresent, password, sizeof(password))) {
    queueBtPasswordResult(false, 0, false, "", payload != nullptr ? payload : "");
    return;
  }

  char expected[32];
  strncpy(expected, config.morseWord, sizeof(expected) - 1);
  expected[sizeof(expected) - 1] = 0;
  normalizeMorseWord(expected);
  normalizeMorseWord(password);

  bool idMatches = (config.allowedScannerId == 0) || (senderIdPresent && senderId == config.allowedScannerId);
  bool passwordMatches = strcmp(password, expected) == 0;
  queueBtPasswordResult(idMatches && passwordMatches, senderId, senderIdPresent, password, payload);
}

class PasswordWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string value = characteristic->getValue();
    handleBtPasswordPayload(value.c_str());
  }
};

class AdvertisingServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer* server) override {
    (void)server;
    if (advertising != nullptr) {
      advertising->start();
    }
  }
};

static void drawHeader(const char* title) {
  tft.fillRect(0, 0, SCREEN_W, 36, COLOR_HEADER_BG);
  tft.setTextColor(COLOR_HEADER_TEXT, COLOR_HEADER_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(title, 10, 18, 2);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(COLOR_HEADER_ACCENT, COLOR_HEADER_BG);
  tft.drawString("CYD-GB", SCREEN_W - 10, 18, 1);
}

static void drawFooter(const char* text) {
  tft.fillRect(0, SCREEN_H - 20, SCREEN_W, 20, COLOR_FOOTER_BG);
  tft.setTextColor(COLOR_FOOTER_TEXT, COLOR_FOOTER_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, SCREEN_W / 2, SCREEN_H - 10, 1);
}
static void drawActionButton(const char* label, int top) {
  tft.fillRoundRect(12, top, SCREEN_W - 24, 32, 6, COLOR_BUTTON_BG);
  tft.drawRoundRect(12, top, SCREEN_W - 24, 32, 6, COLOR_BUTTON_HL_BG);
  tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BUTTON_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, SCREEN_W / 2, top + 16, 2);
}

static bool pointInRect(uint16_t x, uint16_t y, int left, int top, int width, int height) {
  return x >= left && x <= left + width && y >= top && y <= top + height;
}

static void normalizeMac(char* mac) {
  char cleaned[18] = {0};
  uint8_t out = 0;

  for (uint8_t i = 0; mac[i] != 0 && out < sizeof(cleaned) - 1; ++i) {
    char c = mac[i];
    if (c == '-') {
      c = ':';
    }
    if (isxdigit(static_cast<unsigned char>(c)) || c == ':') {
      cleaned[out++] = static_cast<char>(toupper(static_cast<unsigned char>(c)));
    }
  }

  cleaned[out] = 0;
  strncpy(mac, cleaned, sizeof(cleaned));
  mac[sizeof(cleaned) - 1] = 0;
}

static void normalizeMorseWord(char* text) {
  char cleaned[16] = {0};
  uint8_t out = 0;

  for (uint8_t i = 0; text[i] != 0 && out < sizeof(cleaned) - 1; ++i) {
    char c = static_cast<char>(toupper(static_cast<unsigned char>(text[i])));
    if (isalnum(static_cast<unsigned char>(c))) {
      cleaned[out++] = c;
    }
  }

  cleaned[out] = 0;
  strncpy(text, cleaned, sizeof(cleaned));
  text[sizeof(cleaned) - 1] = 0;
}

static void normalizeCodeword(char* text) {
  char cleaned[4] = {0};
  uint8_t out = 0;

  for (uint8_t i = 0; text[i] != 0 && out < 3; ++i) {
    if (isdigit(static_cast<unsigned char>(text[i]))) {
      cleaned[out++] = text[i];
    }
  }

  strncpy(text, cleaned, sizeof(cleaned));
  text[sizeof(cleaned) - 1] = 0;
}

static const char* morseCodeForChar(char c) {
  for (const MorseCodeEntry& entry : kMorseTable) {
    if (entry.ch == c) {
      return entry.code;
    }
  }
  return nullptr;
}

static char morseCharForCode(const char* code) {
  for (const MorseCodeEntry& entry : kMorseTable) {
    if (strcmp(entry.code, code) == 0) {
      return entry.ch;
    }
  }
  return '?';
}

static bool parseMacBytes(const char* mac, uint8_t out[6]) {
  int values[6] = {0};
  int parsed = sscanf(mac, "%x:%x:%x:%x:%x:%x",
                      &values[0], &values[1], &values[2],
                      &values[3], &values[4], &values[5]);
  if (parsed != 6) {
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    if (values[i] < 0 || values[i] > 255) {
      return false;
    }
    out[i] = static_cast<uint8_t>(values[i]);
  }

  out[0] = static_cast<uint8_t>(out[0] | 0xC0);
  return true;
}

static const char* activeEditorTitle() {
  switch (senderView) {
    case VIEW_EDIT_NAME:
      return "Edit Name";
    case VIEW_EDIT_MAC:
      return "Edit MAC";
    case VIEW_EDIT_MORSE_WORD:
      return "Passwort";
    case VIEW_EDIT_CODEWORD:
      return "Codewort (3 Ziffern)";
    case VIEW_EDIT_LDR_DARK_RAW:
      return "Dunkelwert";
    case VIEW_EDIT_LDR_BRIGHT_RAW:
      return "Hellwert";
    case VIEW_EDIT_OK_MESSAGE:
      return "OK-Text";
    case VIEW_EDIT_FAIL_MESSAGE:
      return "Fail-Text";
    case VIEW_EDIT_NEXT_PASSWORD:
      return "Naechstes Passwort";
    case VIEW_EDIT_SCANNER_ID:
      return "Scanner-ID";
    case VIEW_EDIT_LDR_BUFFER_PERCENT:
      return "LDR Puffer %";
#if defined(BT_CHEST_BUILD)
    case VIEW_EDIT_SERVO_PIN:
      return "Servo Pin";
    case VIEW_EDIT_SERVO_ANGLE_OPEN:
      return "Servo Winkel offen";
    case VIEW_EDIT_SERVO_ANGLE_CLOSED:
      return "Servo Winkel zu";
#endif
    default:
      return "Editor";
  }
}

static int clampLdrRawValue(int raw) {
  return constrain(raw, LDR_RAW_MIN, LDR_RAW_MAX);
}

static void sortLdrEndpoints(int* darkRaw, int* brightRaw) {
  if (*darkRaw > *brightRaw) {
    int tmp = *darkRaw;
    *darkRaw = *brightRaw;
    *brightRaw = tmp;
  }
}

static int ldrSpan() {
  return max(1, abs(config.ldrBrightRaw - config.ldrDarkRaw));
}

static int ldrMarginRaw() {
  int span = ldrSpan();
  int margin = (span * config.ldrBufferPercent + 50) / 100;
  return max(1, margin);
}

static int ldrDarkLow() {
  int lower = min(config.ldrDarkRaw, config.ldrBrightRaw);
  return max(LDR_RAW_MIN, lower - ldrMarginRaw());
}

static int ldrDarkHigh() {
  int lower = min(config.ldrDarkRaw, config.ldrBrightRaw);
  return min(LDR_RAW_MAX, lower + ldrMarginRaw());
}

static int ldrBrightLow() {
  int upper = max(config.ldrDarkRaw, config.ldrBrightRaw);
  return max(LDR_RAW_MIN, upper - ldrMarginRaw());
}

static int ldrBrightHigh() {
  int upper = max(config.ldrDarkRaw, config.ldrBrightRaw);
  return min(LDR_RAW_MAX, upper + ldrMarginRaw());
}

static bool ldrSignalActiveForRaw(int raw, bool previousState) {
  if (raw <= ldrDarkHigh()) {
    return true;
  }
  if (raw >= ldrBrightLow()) {
    return false;
  }
  return previousState;
}

static int medianOf3(int a, int b, int c) {
  if (a > b) {
    int tmp = a;
    a = b;
    b = tmp;
  }
  if (b > c) {
    int tmp = b;
    b = c;
    c = tmp;
  }
  if (a > b) {
    int tmp = a;
    a = b;
    b = tmp;
  }
  return b;
}

static int readMorseLdrRaw() {
  int a = analogRead(LDR_PIN);
  int b = analogRead(LDR_PIN);
  int c = analogRead(LDR_PIN);
  return medianOf3(a, b, c);
}

static int filterMorseLdrRaw(int raw) {
  int targetQ8 = raw << 8;
  int delta = targetQ8 - morseRx.filteredRawQ8;
  morseRx.filteredRawQ8 += (delta * LDR_MORSE_FILTER_WEIGHT_NEW + (LDR_MORSE_FILTER_WEIGHT_TOTAL / 2)) / LDR_MORSE_FILTER_WEIGHT_TOTAL;
  return (morseRx.filteredRawQ8 + 128) >> 8;
}

static bool ldrSignalActiveForMorseRaw(int raw, bool previousState) {
  int lower = min(config.ldrDarkRaw, config.ldrBrightRaw);
  int upper = max(config.ldrDarkRaw, config.ldrBrightRaw);
  int span = max(1, upper - lower);

  int center = lower + span / 2;
  int hysteresis = max(2, span / 10);
  int activeThreshold = constrain(center - hysteresis, LDR_RAW_MIN, LDR_RAW_MAX);
  int inactiveThreshold = constrain(center + hysteresis, LDR_RAW_MIN, LDR_RAW_MAX);

  if (raw <= activeThreshold) {
    return true;
  }
  if (raw >= inactiveThreshold) {
    return false;
  }
  return previousState;
}

static void setDefaultLdrCalibration() {
  config.ldrDarkRaw = 50;
  config.ldrBrightRaw = 200;
  config.ldrBufferPercent = LDR_DEFAULT_BUFFER_PERCENT;
}

#if defined(BT_CHEST_BUILD)
static bool isValidServoPin(uint8_t pin) {
  switch (pin) {
    case 4:
    case 16:
    case 17:
    case 18:
    case 22:
    case 26:
    case 27:
      return true;
    default:
      return false;
  }
}

static uint8_t sanitizeServoPin(int pin) {
  return pin >= 0 && pin <= 33 && isValidServoPin(static_cast<uint8_t>(pin))
             ? static_cast<uint8_t>(pin)
             : kDefaultServoPin;
}
#endif

static void setDefaults() {
#if defined(BT_CHEST_BUILD)
  snprintf(config.stationName, sizeof(config.stationName), "BT Kiste");
#else
  snprintf(config.stationName, sizeof(config.stationName), "Fake Station");
#endif
  snprintf(config.mac, sizeof(config.mac), "C2:AD:BE:AC:00:01");
  snprintf(config.morseWord, sizeof(config.morseWord), "HINWEIS");
  snprintf(config.codeword, sizeof(config.codeword), "123");
#if defined(BT_CHEST_BUILD)
  snprintf(config.successMessage, sizeof(config.successMessage), "KISTE GEOEFFNET");
#else
  snprintf(config.successMessage, sizeof(config.successMessage), "RICHTIG!");
#endif
  snprintf(config.failMessage, sizeof(config.failMessage), "FALSCHES PASSWORT");
  snprintf(config.nextPassword, sizeof(config.nextPassword), "NEXT1234");
  config.allowedScannerId = 0;
  setDefaultLdrCalibration();
#if defined(BT_CHEST_BUILD)
  config.servoPin = kDefaultServoPin;
  config.servoAngleOpen = 90;
  config.servoAngleClosed = 0;
#endif
}

static void loadConfig() {
  setDefaults();
  senderPrefs.begin(
#if defined(BT_CHEST_BUILD)
  "btkiste",
#else
  "btsender",
#endif
  true);
  if (senderPrefs.getBool("valid", false)) {
    String name = senderPrefs.getString("name", config.stationName);
    String mac = senderPrefs.getString("mac", config.mac);
    String morseWord = senderPrefs.getString("code", config.morseWord);
    String codeword = senderPrefs.getString("codeword", config.codeword);
    String successMessage = senderPrefs.getString("ok", config.successMessage);
    String failMessage = senderPrefs.getString("fail", config.failMessage);
    String nextPassword = senderPrefs.getString("nextpw", config.nextPassword);
    name.trim();
    mac.trim();
    morseWord.trim();
    codeword.trim();
    successMessage.trim();
    failMessage.trim();
    nextPassword.trim();
    if (name.length() > 0) {
      strncpy(config.stationName, name.c_str(), sizeof(config.stationName) - 1);
      config.stationName[sizeof(config.stationName) - 1] = 0;
    }
    if (mac.length() > 0) {
      strncpy(config.mac, mac.c_str(), sizeof(config.mac) - 1);
      config.mac[sizeof(config.mac) - 1] = 0;
      normalizeMac(config.mac);
    }
    if (morseWord.length() > 0) {
      strncpy(config.morseWord, morseWord.c_str(), sizeof(config.morseWord) - 1);
      config.morseWord[sizeof(config.morseWord) - 1] = 0;
      normalizeMorseWord(config.morseWord);
    }
    if (codeword.length() > 0) {
      strncpy(config.codeword, codeword.c_str(), sizeof(config.codeword) - 1);
      config.codeword[sizeof(config.codeword) - 1] = 0;
      normalizeCodeword(config.codeword);
    }
    if (successMessage.length() > 0) {
      strncpy(config.successMessage, successMessage.c_str(), sizeof(config.successMessage) - 1);
      config.successMessage[sizeof(config.successMessage) - 1] = 0;
    }
    if (failMessage.length() > 0) {
      strncpy(config.failMessage, failMessage.c_str(), sizeof(config.failMessage) - 1);
      config.failMessage[sizeof(config.failMessage) - 1] = 0;
    }
    if (nextPassword.length() > 0) {
      strncpy(config.nextPassword, nextPassword.c_str(), sizeof(config.nextPassword) - 1);
      config.nextPassword[sizeof(config.nextPassword) - 1] = 0;
      config.allowedScannerId = constrain(senderPrefs.getInt("scid", 0), 0, 8);
    }

    int darkRaw = senderPrefs.getInt("ldrDark", -1);
    int brightRaw = senderPrefs.getInt("ldrBright", -1);
    int bufferPercent = senderPrefs.getInt("ldrBuffer", -1);
    if (darkRaw >= 0 && brightRaw >= 0) {
      config.ldrDarkRaw = clampLdrRawValue(darkRaw);
      config.ldrBrightRaw = clampLdrRawValue(brightRaw);
      config.ldrBufferPercent = constrain(bufferPercent >= 0 ? bufferPercent : config.ldrBufferPercent, 0, 49);
    } else {
      int legacyThreshold = senderPrefs.getInt("thr", -1);
      if (legacyThreshold >= 0) {
        config.ldrDarkRaw = clampLdrRawValue(legacyThreshold - 75);
        config.ldrBrightRaw = clampLdrRawValue(legacyThreshold + 75);
        config.ldrBufferPercent = LDR_DEFAULT_BUFFER_PERCENT;
      }
    }
#if defined(BT_CHEST_BUILD)
    config.servoPin = sanitizeServoPin(senderPrefs.getInt("servoPin", config.servoPin));
    config.servoAngleOpen = constrain(senderPrefs.getInt("servoOpen", config.servoAngleOpen), 0, 180);
    config.servoAngleClosed = constrain(senderPrefs.getInt("servoClosed", config.servoAngleClosed), 0, 180);
#endif
  }
  senderPrefs.end();
}

static void saveConfig() {
  senderPrefs.begin(
#if defined(BT_CHEST_BUILD)
      "btkiste",
#else
      "btsender",
#endif
      false);
  senderPrefs.putBool("valid", true);
  senderPrefs.putString("name", config.stationName);
  senderPrefs.putString("mac", config.mac);
  senderPrefs.putString("code", config.morseWord);
  senderPrefs.putString("codeword", config.codeword);
  senderPrefs.putString("ok", config.successMessage);
  senderPrefs.putString("fail", config.failMessage);
  senderPrefs.putString("nextpw", config.nextPassword);
    senderPrefs.putInt("scid", config.allowedScannerId);
  senderPrefs.putInt("ldrDark", config.ldrDarkRaw);
  senderPrefs.putInt("ldrBright", config.ldrBrightRaw);
  senderPrefs.putInt("ldrBuffer", config.ldrBufferPercent);
#if defined(BT_CHEST_BUILD)
  senderPrefs.putInt("servoPin", config.servoPin);
  senderPrefs.putInt("servoOpen", config.servoAngleOpen);
  senderPrefs.putInt("servoClosed", config.servoAngleClosed);
#endif
  senderPrefs.end();
}

#if defined(BT_CHEST_BUILD)
static void setChestServoAngle(uint8_t angle) {
  if (chestServo.attached() && attachedServoPin != config.servoPin) {
    chestServo.detach();
    attachedServoPin = 255;
  }
  if (!chestServo.attached()) {
    chestServo.setPeriodHertz(50);
    chestServo.attach(config.servoPin, 500, 2400);
    if (!chestServo.attached()) {
      Serial.printf("Servo attach failed: GPIO %u\n", config.servoPin);
      return;
    }
    attachedServoPin = config.servoPin;
    Serial.printf("Servo attached: GPIO %u, 50 Hz\n", config.servoPin);
  }
  chestServo.write(angle);
  Serial.printf("Servo command: GPIO %u, %u deg, %d us\n",
                config.servoPin, angle, chestServo.readMicroseconds());
}

static void closeChest() {
  setChestServoAngle(config.servoAngleClosed);
}

static void openChest() {
  setChestServoAngle(config.servoAngleOpen);
}
#endif

static void ensureBleInit() {
  if (advertising != nullptr) {
    return;
  }

  BLEDevice::init(config.stationName);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new AdvertisingServerCallbacks());
  bleService = bleServer->createService(BLEUUID(kBleServiceUuid));
  passwordCharacteristic = bleService->createCharacteristic(
      BLEUUID(kBlePasswordCharUuid),
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ);
  passwordCharacteristic->setCallbacks(new PasswordWriteCallbacks());
  passwordCharacteristic->setValue("READY");
  bleService->start();
  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLEUUID(kBleServiceUuid));
}

static void startOrRefreshAdvertising() {
  ensureBleInit();
  if (advertising == nullptr) {
    return;
  }

  advertising->stop();

  uint8_t addrBytes[6] = {0};
  if (parseMacBytes(config.mac, addrBytes)) {
    esp_bd_addr_t bleAddr;
    memcpy(bleAddr, addrBytes, sizeof(bleAddr));
    advertising->setDeviceAddress(bleAddr, BLE_ADDR_TYPE_RANDOM);
  }

  BLEBeacon beacon;
  beacon.setManufacturerId(0x004C);
  beacon.setMajor(1);
  beacon.setMinor(1);
  beacon.setSignalPower(-59);
  beacon.setProximityUUID(BLEUUID("7f4ac8ec-9b6e-46f0-b6ee-6f95a2d0a917"));

  BLEAdvertisementData advData;
  advData.setFlags(0x06);

  BLEAdvertisementData scanData;
  scanData.setName(config.stationName);

  advertising->setScanResponseData(scanData);
  advertising->setAdvertisementData(advData);
  advertising->setAdvertisementType(ADV_TYPE_IND);
  advertising->setMinInterval(0x40);
  advertising->setMaxInterval(0x80);
  advertising->start();
}

static void drawStatusScreen() {
  tft.fillScreen(COLOR_BG);
#if defined(BT_CHEST_BUILD)
  drawHeader("BT Schatzkiste");
#else
  drawHeader("BT Beacon Sender");
#endif

  tft.fillRoundRect(8, 46, SCREEN_W - 16, 154, 8, COLOR_CARD_BG);
  tft.setTextColor(COLOR_CARD_TEXT, COLOR_CARD_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Advertising: ON", 16, 62, 2);

  tft.drawString("Station Name", 16, 92, 1);
  tft.fillRoundRect(16, 100, SCREEN_W - 32, 22, 4, COLOR_VALUE_BG);
  tft.setTextColor(COLOR_VALUE_TEXT, COLOR_VALUE_BG);
  tft.drawString(config.stationName, 20, 111, 2);

  tft.setTextColor(COLOR_CARD_TEXT, COLOR_CARD_BG);
  tft.drawString("TX MAC", 16, 132, 1);
  tft.fillRoundRect(16, 140, SCREEN_W - 32, 22, 4, COLOR_VALUE_BG);
  tft.setTextColor(COLOR_VALUE_TEXT, COLOR_VALUE_BG);
  tft.drawString(config.mac, 20, 151, 2);

  tft.setTextColor(COLOR_CARD_TEXT, COLOR_CARD_BG);
  tft.drawString("BT Empfang", 16, 178, 1);
  tft.fillRoundRect(16, 186, SCREEN_W - 32, 22, 4, COLOR_VALUE_BG);
  tft.setTextColor(COLOR_VALUE_TEXT, COLOR_VALUE_BG);
  tft.drawString(config.allowedScannerId == 0 ? "Alle Scanner" : "Scanner-Filter aktiv", 20, 197, 2);

  tft.fillRoundRect(12, 230, SCREEN_W - 24, 40, 6, COLOR_BUTTON_BG);
  tft.drawRoundRect(12, 230, SCREEN_W - 24, 40, 6, COLOR_BUTTON_HL_BG);
  tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BUTTON_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("BT Empfang starten", SCREEN_W / 2, 250, 2);

  drawFooter(config.allowedScannerId == 0 ? "Bereit fur BT-Empfang" : "Bereit fur BT-Empfang mit Filter");
}

static void drawEditMenuScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader(editMenuInMorseSubmenu ? "Passwort & Codewort" :
#if defined(BT_CHEST_BUILD)
             (editMenuInChestSubmenu ? "Kisten Servo" : "Sender Settings"));
#else
             "Sender Settings");
#endif

#if defined(BT_CHEST_BUILD)
  const char* mainLabels[8] = {"Edit Name", "Edit MAC", "Passwort & Codewort", "OK-Text", "Fail-Text", "Passwort", "Kisten Servo", "Back"};
  const char* chestLabels[6] = {"Servo Pin", "Winkel offen", "Winkel zu", "Servo offen", "Servo geschlossen", "Zuruck"};
#else
  const char* mainLabels[7] = {"Edit Name", "Edit MAC", "Passwort & Codewort", "OK-Text", "Fail-Text", "Passwort", "Back"};
#endif
  const char* morseLabels[4] = {"Passwort", "Codewort", "Scanner-ID", "Zuruck"};
  const char** labels = editMenuInMorseSubmenu ? morseLabels : mainLabels;
  int count = editMenuInMorseSubmenu ? 4 : sizeof(mainLabels) / sizeof(mainLabels[0]);
#if defined(BT_CHEST_BUILD)
  if (editMenuInChestSubmenu) {
    labels = chestLabels;
    count = 6;
  }
#endif

  for (int i = 0; i < count; ++i) {
    int top = 40 + (i * 28);
    uint16_t bg = (i == selectedEditChoice) ? COLOR_BUTTON_HL_BG : COLOR_BUTTON_BG;
    uint16_t fg = (i == selectedEditChoice) ? COLOR_BUTTON_HL_TEXT : COLOR_BUTTON_TEXT;
    tft.fillRoundRect(12, top, SCREEN_W - 24, 22, 6, bg);
    tft.drawRoundRect(12, top, SCREEN_W - 24, 22, 6, fg);
    tft.setTextColor(fg, bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(labels[i], SCREEN_W / 2, top + 11, 1);
  }

  drawFooter("Touch: Select");
}

static void drawMorseReadyScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader("BT Empfang");

  tft.fillRoundRect(10, 48, SCREEN_W - 20, 118, 8, COLOR_CARD_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_CARD_TEXT, COLOR_CARD_BG);
  tft.drawString(btReceiveActive ? "Empfang ist bereit" : "Empfang inaktiv", 18, 66, 2);
  tft.drawString("Scanner waehlt diese Station.", 18, 98, 1);
  tft.drawString("Dann wird das Passwort gesendet.", 18, 122, 1);
#if defined(BT_CHEST_BUILD)
  tft.drawString("Bei Erfolg wird die Kiste geoeffnet.", 18, 146, 1);
#else
  tft.drawString("Bei Erfolg erscheint das Codewort.", 18, 146, 1);
#endif

  tft.fillRoundRect(12, 238, SCREEN_W - 24, 40, 6, COLOR_BUTTON_BG);
  tft.drawRoundRect(12, 238, SCREEN_W - 24, 40, 6, COLOR_BUTTON_HL_BG);
  tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BUTTON_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Zuruck", SCREEN_W / 2, 258, 2);

  drawFooter("Empfang per Bluetooth");
}

static void drawMorseResultScreen() {
  tft.fillScreen(COLOR_BG);
  drawHeader(morseResultSuccess ? "Passwort richtig" : "Passwort falsch");

  tft.fillRoundRect(10, 60, SCREEN_W - 20, 120, 8, morseResultSuccess ? 0x03E0 : 0x7800);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, morseResultSuccess ? 0x03E0 : 0x7800);
  tft.drawString(morseResultSuccess ? config.successMessage : config.failMessage, 18, 82, 2);
  if (morseResultSuccess) {
#if defined(BT_CHEST_BUILD)
    tft.drawString("SERVO IST GEOEFFNET", 18, 126, 2);
#else
    tft.drawString("DEIN CODEWORT", 18, 116, 2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(config.codeword, SCREEN_W / 2, 154, 4);
    tft.setTextDatum(ML_DATUM);
#endif
  } else {
    tft.drawString("Bitte noch einmal versuchen.", 18, 126, 1);
  }

  tft.fillRoundRect(12, 204, SCREEN_W - 24, 34, 6, COLOR_BUTTON_BG);
  tft.drawRoundRect(12, 204, SCREEN_W - 24, 34, 6, COLOR_BUTTON_HL_BG);
  tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BUTTON_BG);
  tft.drawString("Nochmal", SCREEN_W / 2, 221, 2);

  tft.fillRoundRect(12, 248, SCREEN_W - 24, 34, 6, COLOR_BUTTON_BG);
  tft.drawRoundRect(12, 248, SCREEN_W - 24, 34, 6, COLOR_BUTTON_HL_BG);
  tft.drawString("Zuruck", SCREEN_W / 2, 265, 2);

  drawFooter("Touch zum Weitergehen");
}

static void drawMorseListenScreen(bool force) {
  if (!force && millis() - morseRx.lastDrawMs < MORSE_UI_REFRESH_MS) {
    return;
  }

  morseRx.lastDrawMs = millis();
  if (force || !morseListenUiInitialized) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("RAW:", 10, MORSE_INFO_BASE_Y + 0, 2);
    tft.drawString("RX:", 10, MORSE_INFO_BASE_Y + 22, 2);
    tft.drawString("SCHWARZ:", 10, MORSE_INFO_BASE_Y + 44, 2);
    tft.drawString("WEISS:", 10, MORSE_INFO_BASE_Y + 66, 2);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ABBRECHEN", MORSE_STOP_TOUCH_LEFT, MORSE_STOP_TOUCH_TOP + 3, 1);

    morseListenUiInitialized = true;
    morseListenUiRaw = -1;
    morseListenUiRx[0] = 0;
  }

  tft.setTextDatum(TL_DATUM);
  if (force || morseRx.lastRaw != morseListenUiRaw) {
    tft.fillRect(100, MORSE_INFO_BASE_Y + 0, SCREEN_W - 108, 20, TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    char rawText[24];
    snprintf(rawText, sizeof(rawText), "%d", morseRx.lastRaw);
    tft.drawString(rawText, 100, MORSE_INFO_BASE_Y + 0, 2);
    morseListenUiRaw = morseRx.lastRaw;
  }

  if (force || strcmp(morseRx.receivedWord, morseListenUiRx) != 0) {
    tft.fillRect(100, MORSE_INFO_BASE_Y + 22, SCREEN_W - 108, 20, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(morseRx.receivedWord[0] ? morseRx.receivedWord : "-", 100, MORSE_INFO_BASE_Y + 22, 2);
    strncpy(morseListenUiRx, morseRx.receivedWord, sizeof(morseListenUiRx) - 1);
    morseListenUiRx[sizeof(morseListenUiRx) - 1] = 0;
  }

  if (force) {
    tft.fillRect(100, MORSE_INFO_BASE_Y + 44, SCREEN_W - 108, 20, TFT_BLACK);
    tft.fillRect(100, MORSE_INFO_BASE_Y + 66, SCREEN_W - 108, 20, TFT_BLACK);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    char darkText[20];
    snprintf(darkText, sizeof(darkText), "%d-%d", ldrDarkLow(), ldrDarkHigh());
    tft.drawString(darkText, 100, MORSE_INFO_BASE_Y + 44, 2);

    char brightText[20];
    snprintf(brightText, sizeof(brightText), "%d-%d", ldrBrightLow(), ldrBrightHigh());
    tft.drawString(brightText, 100, MORSE_INFO_BASE_Y + 66, 2);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ABBRECHEN", MORSE_STOP_TOUCH_LEFT, MORSE_STOP_TOUCH_TOP + 3, 1);
  }
}

static void initKeyboardBuffer(const char* text, uint8_t maxLen, KeyboardMode mode, uint8_t fieldIndex) {
  keyboardState.active = true;
  keyboardState.mode = mode;
  keyboardState.fieldIndex = fieldIndex;
  keyboardState.maxLen = maxLen;
  strncpy(keyboardState.buffer, text, sizeof(keyboardState.buffer) - 1);
  keyboardState.buffer[sizeof(keyboardState.buffer) - 1] = 0;
}

static void appendKeyboardChar(char c) {
  size_t len = strlen(keyboardState.buffer);
  if (len >= keyboardState.maxLen) {
    return;
  }
  keyboardState.buffer[len] = c;
  keyboardState.buffer[len + 1] = 0;
}

static void keyboardBackspace() {
  size_t len = strlen(keyboardState.buffer);
  if (len > 0) {
    keyboardState.buffer[len - 1] = 0;
  }
}

static void keyboardClear() {
  keyboardState.buffer[0] = 0;
}

static void keyboardSwitchLayout() {
  keyboardState.mode = (keyboardState.mode == KB_ALPHA) ? KB_MAC : KB_ALPHA;
}

static void saveKeyboardBuffer() {
  switch (keyboardState.fieldIndex) {
    case 0:
      strncpy(config.stationName, keyboardState.buffer, sizeof(config.stationName) - 1);
      config.stationName[sizeof(config.stationName) - 1] = 0;
      break;
    case 1:
      strncpy(config.mac, keyboardState.buffer, sizeof(config.mac) - 1);
      config.mac[sizeof(config.mac) - 1] = 0;
      normalizeMac(config.mac);
      break;
    case 2:
      strncpy(config.morseWord, keyboardState.buffer, sizeof(config.morseWord) - 1);
      config.morseWord[sizeof(config.morseWord) - 1] = 0;
      normalizeMorseWord(config.morseWord);
      break;
    case 3:
      strncpy(config.successMessage, keyboardState.buffer, sizeof(config.successMessage) - 1);
      config.successMessage[sizeof(config.successMessage) - 1] = 0;
      break;
    case 4:
      strncpy(config.failMessage, keyboardState.buffer, sizeof(config.failMessage) - 1);
      config.failMessage[sizeof(config.failMessage) - 1] = 0;
      break;
    case 5:
      strncpy(config.nextPassword, keyboardState.buffer, sizeof(config.nextPassword) - 1);
      config.nextPassword[sizeof(config.nextPassword) - 1] = 0;
      normalizeMorseWord(config.nextPassword);
      break;
    case 6: {
      int parsed = atoi(keyboardState.buffer);
      config.ldrBufferPercent = constrain(parsed, 0, 49);
      break;
    }
    case 7: {
      int parsed = clampLdrRawValue(atoi(keyboardState.buffer));
      config.ldrDarkRaw = parsed;
      break;
    }
    case 8: {
      int parsed = clampLdrRawValue(atoi(keyboardState.buffer));
      config.ldrBrightRaw = parsed;
      break;
    }
    case 9: {
      int parsed = constrain(atoi(keyboardState.buffer), 0, 8);
      config.allowedScannerId = static_cast<uint8_t>(parsed);
      break;
    }
    case 10: {
      char codeword[sizeof(config.codeword)] = {0};
      strncpy(codeword, keyboardState.buffer, sizeof(codeword) - 1);
      normalizeCodeword(codeword);
      if (strlen(codeword) == 3) {
        strncpy(config.codeword, codeword, sizeof(config.codeword));
      }
      break;
    }
#if defined(BT_CHEST_BUILD)
    case 11:
  config.servoPin = sanitizeServoPin(atoi(keyboardState.buffer));
      break;
    case 12:
      config.servoAngleOpen = constrain(atoi(keyboardState.buffer), 0, 180);
      break;
    case 13:
      config.servoAngleClosed = constrain(atoi(keyboardState.buffer), 0, 180);
      break;
#endif
  }

  saveConfig();
#if defined(BT_CHEST_BUILD)
  if (keyboardState.fieldIndex == 12) {
    openChest();
    delay(1500);
    closeChest();
  } else if (keyboardState.fieldIndex == 11 || keyboardState.fieldIndex == 13) {
    closeChest();
  }
#endif
  if (keyboardState.fieldIndex <= 1) {
    startOrRefreshAdvertising();
  }
}

static void drawLdrCalibrationScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("LDR Kalibrierung", SCREEN_W / 2, 26, 2);

  if (ldrCalibration.step == LDR_CALIBRATION_DARK) {
    char stepText[40];
    snprintf(stepText, sizeof(stepText), "1/%u: Sender dunkel machen", LDR_CALIBRATION_SAMPLES);
    tft.drawString(stepText, SCREEN_W / 2, 66, 2);
    tft.drawString("Dann Touch druecken", SCREEN_W / 2, 92, 2);
  } else if (ldrCalibration.step == LDR_CALIBRATION_BRIGHT) {
    char stepText[40];
    snprintf(stepText, sizeof(stepText), "2/%u: Sender hell machen", LDR_CALIBRATION_SAMPLES);
    tft.drawString(stepText, SCREEN_W / 2, 66, 2);
    tft.drawString("Dann unteren Button druecken", SCREEN_W / 2, 92, 2);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Gespeichert", SCREEN_W / 2, 74, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Zurueck ins Menu", SCREEN_W / 2, 102, 2);
  }

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  char rawText[40];
  snprintf(rawText, sizeof(rawText), "RAW: %d", ldrCalibration.lastRaw);
  tft.drawString(rawText, SCREEN_W / 2, 244, 2);

  drawActionButton(ldrCalibration.step == LDR_CALIBRATION_DARK ? "Dunkelwert messen" : "Hellwert messen", 276);
}

static void startLdrCalibration() {
  ldrCalibration.active = true;
  ldrCalibration.step = LDR_CALIBRATION_DARK;
  ldrCalibration.darkRaw = 0;
  ldrCalibration.brightRaw = 0;
  memset(ldrCalibration.darkSamples, 0, sizeof(ldrCalibration.darkSamples));
  memset(ldrCalibration.brightSamples, 0, sizeof(ldrCalibration.brightSamples));
  ldrCalibration.sampleIndex = 0;
  ldrCalibration.lastRaw = analogRead(LDR_PIN);
  ldrCalibration.lastDrawMs = 0;
  senderView = VIEW_LDR_CALIBRATION;
  drawLdrCalibrationScreen();
}

static int averageLdrSamples(const int* samples, uint8_t count) {
  if (count == 0) {
    return 0;
  }

  int sum = 0;
  for (uint8_t i = 0; i < count; ++i) {
    sum += samples[i];
  }
  return (sum + static_cast<int>(count / 2)) / count;
}

static void finishLdrCalibration() {
  config.ldrDarkRaw = clampLdrRawValue(averageLdrSamples(ldrCalibration.darkSamples, LDR_CALIBRATION_SAMPLES));
  config.ldrBrightRaw = clampLdrRawValue(averageLdrSamples(ldrCalibration.brightSamples, LDR_CALIBRATION_SAMPLES));
  saveConfig();
  ldrCalibration.active = false;
  ldrCalibration.step = LDR_CALIBRATION_DONE;
  ldrCalibration.lastDrawMs = millis();
  senderView = VIEW_EDIT_MENU;
  drawEditMenuScreen();
}

static void captureLdrCalibrationPoint() {
  int raw = analogRead(LDR_PIN);
  ldrCalibration.lastRaw = raw;

  if (ldrCalibration.step == LDR_CALIBRATION_DARK) {
    if (ldrCalibration.sampleIndex < LDR_CALIBRATION_SAMPLES) {
      ldrCalibration.darkSamples[ldrCalibration.sampleIndex++] = raw;
      if (ldrCalibration.sampleIndex < LDR_CALIBRATION_SAMPLES) {
        drawLdrCalibrationScreen();
        return;
      }
    }
    ldrCalibration.sampleIndex = 0;
    ldrCalibration.step = LDR_CALIBRATION_BRIGHT;
    drawLdrCalibrationScreen();
    return;
  }

  if (ldrCalibration.step == LDR_CALIBRATION_BRIGHT) {
    if (ldrCalibration.sampleIndex < LDR_CALIBRATION_SAMPLES) {
      ldrCalibration.brightSamples[ldrCalibration.sampleIndex++] = raw;
      if (ldrCalibration.sampleIndex < LDR_CALIBRATION_SAMPLES) {
        drawLdrCalibrationScreen();
        return;
      }
    }
    finishLdrCalibration();
  }
}

static void drawKeyboardRow(const KeyDef* keys, int count, int top, int gap) {
  int availableWidth = SCREEN_W - 16 - (gap * (count - 1));
  int keyWidth = availableWidth / count;
  int left = 8;
  for (int i = 0; i < count; ++i) {
    int x = left + i * (keyWidth + gap);
    tft.fillRoundRect(x, top, keyWidth, 28, 4, COLOR_BUTTON_BG);
    tft.drawRoundRect(x, top, keyWidth, 28, 4, COLOR_BUTTON_HL_BG);
    tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BUTTON_BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(keys[i].label, x + keyWidth / 2, top + 14, 1);
  }
}

static void drawKeyboardScreen(const char* title) {
  tft.fillScreen(COLOR_BG);
  drawHeader(title);

  tft.fillRoundRect(8, 44, SCREEN_W - 16, 34, 6, COLOR_VALUE_BG);
  tft.setTextColor(COLOR_VALUE_TEXT, COLOR_VALUE_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(keyboardState.buffer, 14, 61, 2);

  if (keyboardState.mode == KB_ALPHA) {
    drawKeyboardRow(alphaRow1, 10, 92, 2);
    drawKeyboardRow(alphaRow2, 9, 126, 2);
    drawKeyboardRow(alphaRow3, 7, 160, 2);
    drawKeyboardRow(alphaRow4, 5, 204, 4);
  } else {
    if (keyboardState.mode == KB_NUMERIC) {
      drawKeyboardRow(numericRow1, 3, 92, 6);
      drawKeyboardRow(numericRow2, 3, 128, 6);
      drawKeyboardRow(numericRow3, 3, 164, 6);
      drawKeyboardRow(numericRow4, 5, 208, 4);
    } else {
    drawKeyboardRow(macRow1, 10, 102, 2);
    drawKeyboardRow(macRow2, 7, 136, 2);
    drawKeyboardRow(macRow3, 5, 180, 4);
    }
  }

  drawFooter("Tap keys  ESC: zuruck  OK: speichern");
}

static bool keyboardTap(uint16_t x, uint16_t y) {
  const KeyDef* rowsAlpha[] = {alphaRow1, alphaRow2, alphaRow3, alphaRow4};
  const int rowCountsAlpha[] = {10, 9, 7, 5};
  const int rowTopAlpha[] = {92, 126, 160, 204};
  const int rowGapAlpha[] = {2, 2, 2, 4};

  const KeyDef* rowsMac[] = {macRow1, macRow2, macRow3};
  const int rowCountsMac[] = {10, 7, 5};
  const int rowTopMac[] = {102, 136, 180};
  const int rowGapMac[] = {2, 2, 4};

  const KeyDef* rowsNumeric[] = {numericRow1, numericRow2, numericRow3, numericRow4};
  const int rowCountsNumeric[] = {3, 3, 3, 5};
  const int rowTopNumeric[] = {92, 128, 164, 208};
  const int rowGapNumeric[] = {6, 6, 6, 4};

  const KeyDef** rows = nullptr;
  const int* counts = nullptr;
  const int* tops = nullptr;
  const int* gaps = nullptr;
  int rowCount = 0;
  if (keyboardState.mode == KB_ALPHA) {
    rows = rowsAlpha;
    counts = rowCountsAlpha;
    tops = rowTopAlpha;
    gaps = rowGapAlpha;
    rowCount = 4;
  } else if (keyboardState.mode == KB_NUMERIC) {
    rows = rowsNumeric;
    counts = rowCountsNumeric;
    tops = rowTopNumeric;
    gaps = rowGapNumeric;
    rowCount = 4;
  } else {
    rows = rowsMac;
    counts = rowCountsMac;
    tops = rowTopMac;
    gaps = rowGapMac;
    rowCount = 3;
  }

  for (int row = 0; row < rowCount; ++row) {
    int count = counts[row];
    int top = tops[row];
    int gap = gaps[row];
    int availableWidth = SCREEN_W - 16 - (gap * (count - 1));
    int keyWidth = availableWidth / count;
    int left = 8;

    for (int col = 0; col < count; ++col) {
      int keyLeft = left + col * (keyWidth + gap);
      int keyRight = keyLeft + keyWidth;
      int keyBottom = top + 28;
      if (x >= keyLeft && x <= keyRight && y >= top && y <= keyBottom) {
        const KeyDef& key = rows[row][col];
        switch (key.action) {
          case KEY_CHAR:
            appendKeyboardChar(key.value);
            break;
          case KEY_SPACE:
            appendKeyboardChar(' ');
            break;
          case KEY_BACKSPACE:
            keyboardBackspace();
            break;
          case KEY_SAVE:
            saveKeyboardBuffer();
            senderView = VIEW_EDIT_MENU;
            drawEditMenuScreen();
            return true;
          case KEY_SWITCH:
            keyboardSwitchLayout();
            drawKeyboardScreen(activeEditorTitle());
            return true;
          case KEY_CLEAR:
            keyboardClear();
            break;
          case KEY_CANCEL:
            senderView = VIEW_EDIT_MENU;
            drawEditMenuScreen();
            return true;
        }
        drawKeyboardScreen(activeEditorTitle());
        return true;
      }
    }
  }

  return false;
}

static void resetMorseReceiver() {
  memset(&morseRx, 0, sizeof(morseRx));
  morseRx.listening = true;
  morseRx.stateSince = millis();
  morseRx.lastDrawMs = 0;
  morseListenUiInitialized = false;
  morseListenUiRaw = -1;
  morseListenUiRx[0] = 0;
}

static void drawCurrentScreen();
static void drawLdrCalibrationScreen();
static void startLdrCalibration();
static void captureLdrCalibrationPoint();
static void finishLdrCalibration();
static void openTextEditor(uint8_t fieldIndex, const char* text, uint8_t maxLen, KeyboardMode mode, SenderView view);
static void handleEditMenuSelection(int index);

static void finishMorseReception() {
  strncpy(morseResultWord, morseRx.receivedWord, sizeof(morseResultWord) - 1);
  morseResultWord[sizeof(morseResultWord) - 1] = 0;
  normalizeMorseWord(morseResultWord);
  morseResultSuccess = strcmp(morseResultWord, config.morseWord) == 0;
  displayAwake = true;
  digitalWrite(TFT_BL, HIGH);
  lastInteractionMs = millis();
  senderView = VIEW_MORSE_RESULT;
  morseRx.listening = false;
  drawMorseResultScreen();
}

static void finalizeMorseCharacter() {
  if (morseRx.currentSymbolLen == 0) {
    return;
  }

  morseRx.currentSymbol[morseRx.currentSymbolLen] = 0;
  char decoded = morseCharForCode(morseRx.currentSymbol);
  if (morseRx.receivedLen < sizeof(morseRx.receivedWord) - 1) {
    morseRx.receivedWord[morseRx.receivedLen++] = decoded;
    morseRx.receivedWord[morseRx.receivedLen] = 0;
  }
  morseRx.currentSymbolLen = 0;
  morseRx.currentSymbol[0] = 0;
  morseRx.postambleCount = 0;
}

static void processMorseGap(unsigned long durationMs) {
  if (!morseRx.frameStarted) {
    if (durationMs >= MORSE_UNIT_MS * 4) {
      morseRx.preambleCount = 0;
      morseRx.startPatternDashCount = 0;
    }
    return;
  }

  if (morseRx.currentSymbolLen > 0 && durationMs >= MORSE_UNIT_MS * 2) {
    finalizeMorseCharacter();
  }

  if (durationMs >= MORSE_UNIT_MS * 12) {
    morseRx.frameStarted = false;
    morseRx.preambleCount = 0;
    morseRx.postambleCount = 0;
    morseRx.currentSymbolLen = 0;
    morseRx.currentSymbol[0] = 0;
    morseRx.receivedLen = 0;
    morseRx.receivedWord[0] = 0;
  }
}

static void processMorsePulse(unsigned long durationMs) {
  enum PulseKind { PULSE_DOT, PULSE_DASH, PULSE_FRAME };
  PulseKind kind = PULSE_DOT;
  if (durationMs >= MORSE_UNIT_MS * 4) {
    kind = PULSE_FRAME;
  } else if (durationMs >= MORSE_UNIT_MS * 2) {
    kind = PULSE_DASH;
  }

  if (!morseRx.frameStarted) {
    if (kind == PULSE_DOT) {
      if (morseRx.startPatternDashCount > 0) {
        morseRx.startPatternDashCount = 0;
        morseRx.preambleCount = 1;
      } else {
        morseRx.preambleCount++;
      }
      if (morseRx.preambleCount >= 5) {
        morseRx.frameStarted = true;
        morseRx.receivedLen = 0;
        morseRx.receivedWord[0] = 0;
        morseRx.currentSymbolLen = 0;
        morseRx.currentSymbol[0] = 0;
        morseRx.postambleCount = 0;
        morseRx.startPatternDashCount = 0;
      }
      return;
    }

    if (kind == PULSE_DASH) {
      if (morseRx.preambleCount == 3) {
        morseRx.startPatternDashCount++;
        if (morseRx.startPatternDashCount >= 3) {
          morseRx.frameStarted = true;
          morseRx.receivedLen = 0;
          morseRx.receivedWord[0] = 0;
          morseRx.currentSymbolLen = 0;
          morseRx.currentSymbol[0] = 0;
          morseRx.postambleCount = 0;
          morseRx.preambleCount = 0;
          morseRx.startPatternDashCount = 0;
        }
      } else {
        morseRx.preambleCount = 0;
        morseRx.startPatternDashCount = 0;
      }
    } else {
      morseRx.preambleCount = 0;
      morseRx.startPatternDashCount = 0;
    }
    return;
  }

  if (kind == PULSE_FRAME) {
    if (morseRx.currentSymbolLen == 0) {
      morseRx.postambleCount++;
      if (morseRx.postambleCount >= 5) {
        finishMorseReception();
      }
    } else {
      morseRx.currentSymbolLen = 0;
      morseRx.currentSymbol[0] = 0;
      morseRx.postambleCount = 1;
    }
    return;
  }

  morseRx.postambleCount = 0;
  if (morseRx.currentSymbolLen < sizeof(morseRx.currentSymbol) - 1) {
    morseRx.currentSymbol[morseRx.currentSymbolLen++] = (kind == PULSE_DOT) ? '.' : '-';
    morseRx.currentSymbol[morseRx.currentSymbolLen] = 0;
  }
}

static void handleMorseListening() {
  if (senderView != VIEW_MORSE_LISTEN || !morseRx.listening) {
    return;
  }

  unsigned long now = millis();
  morseRx.lastRaw = 1200 + static_cast<int>((now / 180) % 12) * 180;
  morseRx.signalActive = ((now / 400) % 2) == 0;
  morseRx.receivedLen = morseRx.receivedWord[0] != 0 ? morseRx.receivedLen : 0;

  if (senderView == VIEW_MORSE_LISTEN) {
    drawMorseListenScreen(false);
  }
}

static void drawCurrentScreen() {
  switch (senderView) {
    case VIEW_STATUS:
      drawStatusScreen();
      break;
    case VIEW_EDIT_MENU:
      drawEditMenuScreen();
      break;
    case VIEW_EDIT_NAME:
    case VIEW_EDIT_MAC:
    case VIEW_EDIT_MORSE_WORD:
    case VIEW_EDIT_LDR_DARK_RAW:
    case VIEW_EDIT_LDR_BRIGHT_RAW:
    case VIEW_EDIT_OK_MESSAGE:
    case VIEW_EDIT_FAIL_MESSAGE:
    case VIEW_EDIT_NEXT_PASSWORD:
    case VIEW_EDIT_LDR_BUFFER_PERCENT:
      drawKeyboardScreen(activeEditorTitle());
      break;
    case VIEW_LDR_CALIBRATION:
      drawLdrCalibrationScreen();
      break;
    case VIEW_MORSE_READY:
      drawMorseReadyScreen();
      break;
    case VIEW_MORSE_LISTEN:
      drawMorseListenScreen(true);
      break;
    case VIEW_MORSE_RESULT:
      drawMorseResultScreen();
      break;
  }
}

static void wakeDisplayIfNeeded() {
  if (!displayAwake) {
    digitalWrite(TFT_BL, HIGH);
    displayAwake = true;
    drawCurrentScreen();
  }
}

static void noteInteraction() {
  lastInteractionMs = millis();
}

static void enterEditMenu() {
  senderView = VIEW_EDIT_MENU;
  editMenuInMorseSubmenu = false;
#if defined(BT_CHEST_BUILD)
  editMenuInChestSubmenu = false;
#endif
  selectedEditChoice = 0;
  drawEditMenuScreen();
}

static void startMorseListening() {
  noteInteraction();
  btReceiveActive = true;
  senderView = VIEW_MORSE_READY;
  drawMorseReadyScreen();
}

static void handleEditMenuSelection(int index) {
  selectedEditChoice = index;
  if (!editMenuInMorseSubmenu) {
#if defined(BT_CHEST_BUILD)
    if (editMenuInChestSubmenu) {
      if (index == 0) {
        char value[4];
        snprintf(value, sizeof(value), "%u", config.servoPin);
        openTextEditor(11, value, sizeof(value) - 1, KB_NUMERIC, VIEW_EDIT_SERVO_PIN);
      } else if (index == 1) {
        char value[4];
        snprintf(value, sizeof(value), "%u", config.servoAngleOpen);
        openTextEditor(12, value, sizeof(value) - 1, KB_NUMERIC, VIEW_EDIT_SERVO_ANGLE_OPEN);
      } else if (index == 2) {
        char value[4];
        snprintf(value, sizeof(value), "%u", config.servoAngleClosed);
        openTextEditor(13, value, sizeof(value) - 1, KB_NUMERIC, VIEW_EDIT_SERVO_ANGLE_CLOSED);
      } else if (index == 3) {
        openChest();
        drawEditMenuScreen();
      } else if (index == 4) {
        closeChest();
        drawEditMenuScreen();
      } else {
        editMenuInChestSubmenu = false;
        selectedEditChoice = 0;
        drawEditMenuScreen();
      }
      return;
    }
#endif
    if (index == 0) {
      openTextEditor(0, config.stationName, sizeof(config.stationName) - 1, KB_ALPHA, VIEW_EDIT_NAME);
    } else if (index == 1) {
      openTextEditor(1, config.mac, sizeof(config.mac) - 1, KB_MAC, VIEW_EDIT_MAC);
    } else if (index == 2) {
      editMenuInMorseSubmenu = true;
      selectedEditChoice = 0;
      drawEditMenuScreen();
    } else if (index == 3) {
      openTextEditor(3, config.successMessage, sizeof(config.successMessage) - 1, KB_ALPHA, VIEW_EDIT_OK_MESSAGE);
    } else if (index == 4) {
      openTextEditor(4, config.failMessage, sizeof(config.failMessage) - 1, KB_ALPHA, VIEW_EDIT_FAIL_MESSAGE);
    } else if (index == 5) {
      openTextEditor(5, config.nextPassword, sizeof(config.nextPassword) - 1, KB_ALPHA, VIEW_EDIT_NEXT_PASSWORD);
#if defined(BT_CHEST_BUILD)
    } else if (index == 6) {
      editMenuInChestSubmenu = true;
      selectedEditChoice = 0;
      drawEditMenuScreen();
#endif
    } else {
      senderView = VIEW_STATUS;
      drawStatusScreen();
    }
  } else {
    if (index == 0) {
      openTextEditor(2, config.morseWord, sizeof(config.morseWord) - 1, KB_ALPHA, VIEW_EDIT_MORSE_WORD);
    } else if (index == 1) {
      openTextEditor(10, config.codeword, sizeof(config.codeword) - 1, KB_NUMERIC, VIEW_EDIT_CODEWORD);
    } else if (index == 2) {
      char bufferText[8];
      snprintf(bufferText, sizeof(bufferText), "%u", config.allowedScannerId);
      openTextEditor(9, bufferText, sizeof(bufferText) - 1, KB_NUMERIC, VIEW_EDIT_SCANNER_ID);
    } else {
      editMenuInMorseSubmenu = false;
      selectedEditChoice = 0;
      drawEditMenuScreen();
    }
  }
}

static void openTextEditor(uint8_t fieldIndex, const char* text, uint8_t maxLen, KeyboardMode mode, SenderView view) {
  senderView = view;
  initKeyboardBuffer(text, maxLen, mode, fieldIndex);
  drawKeyboardScreen(activeEditorTitle());
}

static bool touchTap(uint16_t x, uint16_t y) {
  if (senderView == VIEW_STATUS) {
    if (x >= 12 && x <= SCREEN_W - 12 && y >= 230 && y <= 270) {
      startMorseListening();
      return true;
    }
  } else if (senderView == VIEW_EDIT_MENU) {
    int count = editMenuInMorseSubmenu ? 4 :
#if defined(BT_CHEST_BUILD)
                (editMenuInChestSubmenu ? 6 : 8);
#else
                7;
#endif
    for (int i = 0; i < count; ++i) {
      int top = 40 + (i * 28);
      if (y >= top && y <= top + 22) {
        handleEditMenuSelection(i);
        return true;
      }
    }
  } else if (senderView == VIEW_LDR_CALIBRATION) {
    if (x >= 12 && x <= SCREEN_W - 12 && y >= 276 && y <= 308) {
      captureLdrCalibrationPoint();
      return true;
    }
  } else if (senderView == VIEW_MORSE_READY) {
    if (x >= 12 && x <= SCREEN_W - 12 && y >= 238 && y <= 278) {
      btReceiveActive = false;
      senderView = VIEW_STATUS;
      drawStatusScreen();
      return true;
    }
  } else if (senderView == VIEW_MORSE_LISTEN) {
    if (pointInRect(x, y, MORSE_STOP_TOUCH_LEFT, MORSE_STOP_TOUCH_TOP, MORSE_STOP_TOUCH_WIDTH, MORSE_STOP_TOUCH_HEIGHT)) {
      btReceiveActive = false;
      senderView = VIEW_MORSE_READY;
      morseRx.listening = false;
      drawMorseReadyScreen();
      return true;
    }
  } else if (senderView == VIEW_MORSE_RESULT) {
    if (x >= 12 && x <= SCREEN_W - 12 && y >= 204 && y <= 238) {
      startMorseListening();
      return true;
    }
    if (x >= 12 && x <= SCREEN_W - 12 && y >= 248 && y <= 282) {
      btReceiveActive = false;
      senderView = VIEW_STATUS;
      drawStatusScreen();
      return true;
    }
  } else {
    return keyboardTap(x, y);
  }

  return false;
}

static void handleDisplaySleep() {
  if (btReceiveActive) {
    return;
  }

  if (displayAwake && (millis() - lastInteractionMs >= DISPLAY_TIMEOUT_MS)) {
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_BL, LOW);
    displayAwake = false;
  }
}

static void handleTouch() {
  uint16_t x = 0;
  uint16_t y = 0;
  bool touching = touchReadScreen(&x, &y);

  if (touching) {
    if (!touchPressed) {
      touchPressed = true;
      touchLongHandled = false;
      touchPressX = x;
      touchPressY = y;
      touchPressStartMs = millis();

      if (!displayAwake) {
        wakeDisplayIfNeeded();
        noteInteraction();
      }
    } else if (!touchLongHandled && senderView == VIEW_STATUS && millis() - touchPressStartMs >= SETTINGS_LONGPRESS_MS) {
      touchLongHandled = true;
      enterEditMenu();
      noteInteraction();
    }
  } else {
    if (touchPressed) {
      if (displayAwake && !touchLongHandled) {
        touchTap(touchPressX, touchPressY);
        noteInteraction();
      }
    }
    touchPressed = false;
    touchLongHandled = false;
  }
}

void btSenderEnter() {
  loadConfig();
  normalizeMac(config.mac);
  normalizeMorseWord(config.morseWord);
  normalizeCodeword(config.codeword);
  normalizeMorseWord(config.nextPassword);
  pinMode(LDR_PIN, INPUT);
  analogRead(LDR_PIN);
#if defined(BT_CHEST_BUILD)
  Serial.printf("Chest servo: GPIO %u, open %u deg, closed %u deg\n",
                config.servoPin, config.servoAngleOpen, config.servoAngleClosed);
  closeChest();
#endif
  startOrRefreshAdvertising();

  senderView = VIEW_STATUS;
  editMenuInMorseSubmenu = false;
#if defined(BT_CHEST_BUILD)
  editMenuInChestSubmenu = false;
#endif
  selectedEditChoice = 0;
  displayAwake = true;
  touchPressed = false;
  touchLongHandled = false;
  lastInteractionMs = millis();
  memset(&morseRx, 0, sizeof(morseRx));
  morseResultWord[0] = 0;
  ldrCalibration = {};
  btReceiveActive = false;
  btResultPending = false;

  drawStatusScreen();
}

void btSenderLoop() {
  handleTouch();
  handleMorseListening();
  if (btResultPending) {
    btResultPending = false;
    strncpy(morseResultWord, btPendingPassword, sizeof(morseResultWord) - 1);
    morseResultWord[sizeof(morseResultWord) - 1] = 0;
    morseResultSuccess = btPendingSuccess;
#if defined(BT_CHEST_BUILD)
    if (morseResultSuccess) {
      openChest();
    }
#endif
    btReceiveActive = false;
    morseRx.listening = false;
    displayAwake = true;
    digitalWrite(TFT_BL, HIGH);
    lastInteractionMs = millis();
    senderView = VIEW_MORSE_RESULT;
    drawMorseResultScreen();
  }
  handleDisplaySleep();
}
