#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BEACON_DIR="$ROOT_DIR/beacon-local"

echo "[1/4] Build GB scanner (env: cyd)"
pio run -d "$ROOT_DIR" -e cyd

echo "[2/4] Build BT logger (env: btlogger)"
pio run -d "$ROOT_DIR" -e btlogger

echo "[3/4] Build BLE beacon (env: esp32dev_sender)"
pio run -d "$BEACON_DIR" -e esp32dev_sender

echo "[4/4] Build BLE treasure chest (env: btkiste_timestamp)"
pio run -d "$BEACON_DIR" -e btkiste_timestamp

GB_BIN="$(ls -1t "$ROOT_DIR"/builds/gbscanner-*.bin | head -n 1)"
LOGGER_BIN="$(ls -1t "$ROOT_DIR"/builds/btlogger-*.bin | head -n 1)"
BEACON_BIN="$(ls -1t "$BEACON_DIR"/btsend-*.bin | head -n 1)"
CHEST_BIN="$(ls -1t "$BEACON_DIR"/btkiste-*.bin | head -n 1)"

cp "$GB_BIN" "$ROOT_DIR/"
cp "$LOGGER_BIN" "$ROOT_DIR/"
cp "$BEACON_BIN" "$ROOT_DIR/"
cp "$CHEST_BIN" "$ROOT_DIR/"

echo "[OK] Both builds done."
echo "[OUT] $(basename "$GB_BIN")"
echo "[OUT] $(basename "$LOGGER_BIN")"
echo "[OUT] $(basename "$BEACON_BIN")"
echo "[OUT] $(basename "$CHEST_BIN")"
