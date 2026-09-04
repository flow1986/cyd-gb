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

# Schatzkiste mit Servo
python3 -m platformio run -e btkiste_timestamp
python3 -m platformio run -e btkiste_timestamp --target upload
```

Die Schatzkisten-Firmware verwendet denselben BLE-Password-Receiver wie der Beacon.
Ein langer Touch auf dem Statusbildschirm oeffnet das versteckte Einstellungsmenue;
unter `Kisten Servo` lassen sich Servo-Pin sowie die Winkel fuer offen und geschlossen
setzen. Die Standardwerte sind GPIO 27, offen 90 Grad und geschlossen 0 Grad. Beim
Start faehrt die Kiste in die geschlossene Position. Ein korrektes BLE-Passwort oeffnet
den Servo und zeigt kein Codewort an.

## Binary-Namen
- Empfaenger: `btsend-YYYYMMDD-HHMMSS.bin`
- Schatzkiste: `btkiste-YYYYMMDD-HHMMSS.bin`

## Abhaengigkeiten
- TFT_eSPI
- PCF8574
- ESP32 BLE Arduino
- fake-cyd-gui-kit
