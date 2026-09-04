#include <Arduino.h>
#include "hw_config.h"
#include "display.h"
#include "touch_input.h"
#include "button_input.h"
#include "sd_manager.h"
#include "ui_launcher.h"
#include "emulator_bridge.h"
#include "bt_scanner.h"

static RomEntry roms[64];
static int rcnt = 0;
static char cur_path[80] = {0};
static TaskHandle_t ttask = nullptr;
static volatile bool emu_on = false, menu_req = false;
static bool show_fps_overlay = false;
static bool show_sd_save_overlay = false;
static bool has_saved_settings = false;
static constexpr uint32_t SAVE_RAM_DEBOUNCE_MS = 3000;

void input_task(void* p) {
    (void)p;
    bool prev_menu_combo = false;
    for(;;) {
        button_update();
        if (emu_on) {
            uint16_t b = button_get_buttons();
            bool menu_combo = (b & (GB_BTN_START | GB_BTN_SELECT)) == (GB_BTN_START | GB_BTN_SELECT);
            if (menu_combo && !prev_menu_combo) {
                menu_req = true;
            }
            prev_menu_combo = menu_combo;
            if (menu_combo) {
                emu_set_joypad(b & ~(GB_BTN_START | GB_BTN_SELECT));
            } else {
                emu_set_joypad(b & 0xFF);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(12));
    }
}

static void tt_start() {
    if(!ttask) xTaskCreatePinnedToCore(input_task,"t",4096,0,2,&ttask,0);
    else vTaskResume(ttask);
}
static void tt_stop() { if(ttask) vTaskSuspend(ttask); }

static void save_ram() {
    if(!cur_path[0]) return;
    uint32_t sz=0; uint8_t* r=emu_get_cart_ram(&sz);
    if(sz>0) {
        bool ok = sd_save_state(cur_path,r,sz);
        Serial.printf("[SAVE] %u bytes (%s)\n",sz, ok ? "ok" : "fail");
        if (ok) emu_clear_cart_ram_dirty();
    }
}

static void load_ram() {
    if(!cur_path[0]) return;
    uint32_t sz=0;
    uint8_t* cart_ram = emu_get_cart_ram(&sz);
    if (sz == 0) {
        Serial.println("[SAVE] Load skipped: cart RAM size is 0");
        return;
    }

    if (!cart_ram) {
        Serial.println("[SAVE] Load failed: cart RAM pointer is null");
        return;
    }

    if (sd_load_state(cur_path, cart_ram, sz)) {
        Serial.printf("[SAVE] Loaded %u bytes\n", sz);
        emu_clear_cart_ram_dirty();
    } else {
        Serial.printf("[SAVE] No load for %s\n", cur_path);
    }
}

// ─── Emulation loop ─────────────────────────────────────────────────────────
void run_emu() {
    emu_on = true; menu_req = false;
    tt_start();
    display_clear(TFT_BLACK);
    display_draw_controls();

    while(emu_on) {
        emu_run_frame();

        if (emu_cart_ram_dirty() &&
            millis() - emu_get_cart_ram_last_write_ms() >= SAVE_RAM_DEBOUNCE_MS) {
            save_ram();
        }

        if (menu_req) {
            menu_req = false;
            tt_stop();

            int c = launcher_ingame_menu();
            switch(c) {
                case 0: break;  // resume
                case 1:  // save
                    save_ram();
                    tft.fillRect(80,80,160,40,TFT_BLACK);
                    tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_GREEN);
                    tft.drawString("SAVED!",SCREEN_W/2,100,4);
                    delay(700);
                    break;
                case 2:  // load
                    load_ram(); emu_reset(); load_ram();
                    tft.fillRect(80,80,160,40,TFT_BLACK);
                    tft.setTextDatum(MC_DATUM); tft.setTextColor(0x07FF);
                    tft.drawString("LOADED!",SCREEN_W/2,100,4);
                    delay(700);
                    break;
                case 3:  // quit
                    emu_on=false; save_ram(); tt_stop(); return;
                case 5:  // settings
                    launcher_settings_menu(&show_fps_overlay, &show_sd_save_overlay); break;
            }
            display_clear(TFT_BLACK);
            display_draw_controls();
            tt_start();
        }

        taskYIELD();
    }
}

static void run_bt_scanner() {
    bt_scanner_enter();
    while (true) {
        if (bt_scanner_loop()) break;
        delay(10);
        taskYIELD();
    }
    bt_scanner_shutdown();
    display_clear(TFT_BLACK);
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200); delay(200);
    Serial.println("\n=== CYD-GB ===");
    pinMode(LED_R_PIN, OUTPUT);
    if (LED_G_PIN >= 0) pinMode(LED_G_PIN, OUTPUT);
    if (LED_B_PIN >= 0) pinMode(LED_B_PIN, OUTPUT);
    digitalWrite(LED_R_PIN, HIGH);
    if (LED_G_PIN >= 0) digitalWrite(LED_G_PIN, HIGH);
    if (LED_B_PIN >= 0) digitalWrite(LED_B_PIN, HIGH);

    display_init();
    touch_init();
    button_init();

    if(!sd_init()) {
        tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_RED); tft.drawString("SD Card Error!",SCREEN_W/2,100,4);
        tft.setTextColor(0x7BEF); tft.drawString("Insert FAT32 SD & reset",SCREEN_W/2,140,2);
        while(true) delay(1000);
    }

    // Splash
    tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM);
    tft.setTextColor(0x07E0); tft.drawString("CYD-GB",SCREEN_W/2,70,4);
    tft.setTextColor(0x7BEF); tft.drawString("Game Boy Emulator",SCREEN_W/2,110,2);
    delay(1200);

    // Load saved settings from NVS
    uint8_t s_pal, s_fs, s_bl;
    if (touch_load_settings(&s_pal, &s_fs, &s_bl, &show_fps_overlay, &show_sd_save_overlay)) {
        has_saved_settings = true;
        emu_set_palette(s_pal);
        emu_set_frame_skip(s_fs);
        display_set_backlight(s_bl);
        Serial.printf("[INIT] Loaded settings: pal=%d fs=%d bl=%d fps_ov=%d save_ov=%d\n",
                      s_pal, s_fs, s_bl, (int)show_fps_overlay, (int)show_sd_save_overlay);
    }

    Serial.printf("[INIT] Heap: %u\n",ESP.getFreeHeap());
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    rcnt = sd_scan_roms(roms, 64);
    int sel = launcher_show(roms, rcnt);
    if (sel == LAUNCHER_SEL_BT_SCANNER) {
        run_bt_scanner();
        return;
    }
    if(sel<0||sel>=rcnt) return;

    strncpy(cur_path,roms[sel].full_path,79);
    cur_path[79] = 0;

    // Loading screen
    tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM);
    tft.setTextColor(0x07E0); tft.drawString("Loading...",SCREEN_W/2,90,4);
    char nm[30]; strncpy(nm,roms[sel].filename,28); nm[28]=0;
    char* d=strrchr(nm,'.'); if(d)*d=0;
    tft.setTextColor(TFT_WHITE); tft.drawString(nm,SCREEN_W/2,130,2);

    if(!emu_open_rom(cur_path)){
        tft.setTextColor(TFT_RED); tft.drawString("Open failed!",SCREEN_W/2,170,2); delay(2000); return;
    }
    if(!emu_init(0,0)){
        tft.setTextColor(TFT_RED); tft.drawString("Init failed!",SCREEN_W/2,170,2); delay(2000); emu_shutdown(); return;
    }

    load_ram();
    if (!has_saved_settings) emu_set_frame_skip(2);
    if (LED_G_PIN >= 0) digitalWrite(LED_G_PIN, LOW);
    run_emu();
    if (LED_G_PIN >= 0) digitalWrite(LED_G_PIN, HIGH);
    emu_shutdown();
}
