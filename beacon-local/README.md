# fake-cyd-beacon-scanner

Firmware fuer den BT-Beacon-Morse-Empfaenger auf Fake-CYD.

## Architektur
- Empfaenger-Logik in `src/sender/apps/beacon_sender.*`
- Entrypoint in `src/sender/main.cpp`
- Gemeinsame GUI/Touch-Basis aus `fake-cyd-gui-kit`

## Empfaenger-Features
- Startet direkt in die Beacon-Empfaenger-Firmware
- BLE Advertising dauerhaft aktiv
- Einstellbar:
	- Stationsname (wird als BLE Name in Scan Response gesendet)
	- Beacon-MAC (Random BLE Address fuer Advertising)
	- Morse-Wort, LDR-Kalibrierung und LDR-Puffer
- Display geht nach 20s ohne Interaktion aus
- Touch weckt das Display sofort wieder auf

## Build
```bash
python3 -m pip install -U platformio
python3 -m platformio run -e esp32dev_sender
python3 -m platformio run -e esp32dev_sender --target upload
```

## Binary-Namen
- Empfaenger: `btsend-YYYYMMDD-HHMMSS.bin`

## Abhaengigkeiten
- TFT_eSPI
- PCF8574
- ESP32 BLE Arduino
- fake-cyd-gui-kit
