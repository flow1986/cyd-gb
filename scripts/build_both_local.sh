#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BEACON_DIR="$ROOT_DIR/beacon-local"

echo "[1/3] Build GB scanner (env: cyd)"
pio run -d "$ROOT_DIR" -e cyd

echo "[2/3] Build BLE beacon (env: esp32dev_sender)"
pio run -d "$BEACON_DIR" -e esp32dev_sender

echo "[3/3] Build BLE treasure chest (env: btkiste_timestamp)"
pio run -d "$BEACON_DIR" -e btkiste_timestamp

GB_BIN="$(ls -1t "$ROOT_DIR"/builds/gbscanner-*.bin | head -n 1)"
BEACON_BIN="$(ls -1t "$BEACON_DIR"/btsend-*.bin | head -n 1)"
CHEST_BIN="$(ls -1t "$BEACON_DIR"/btkiste-*.bin | head -n 1)"

cp "$GB_BIN" "$ROOT_DIR/"
cp "$BEACON_BIN" "$ROOT_DIR/"
cp "$CHEST_BIN" "$ROOT_DIR/"

echo "[OK] Both builds done."
echo "[OUT] $(basename "$GB_BIN")"
echo "[OUT] $(basename "$BEACON_BIN")"
echo "[OUT] $(basename "$CHEST_BIN")"
