#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BEACON_DIR="$ROOT_DIR/beacon-local"

echo "[1/2] Build GB scanner (env: cyd)"
pio run -d "$ROOT_DIR" -e cyd

echo "[2/2] Build BLE beacon (env: esp32dev_sender)"
pio run -d "$BEACON_DIR" -e esp32dev_sender

GB_BIN="$(ls -1t "$ROOT_DIR"/builds/gbscanner-*.bin | head -n 1)"
BEACON_BIN="$(ls -1t "$BEACON_DIR"/btsend-*.bin | head -n 1)"

cp "$GB_BIN" "$ROOT_DIR/"
cp "$BEACON_BIN" "$ROOT_DIR/"

echo "[OK] Both builds done."
echo "[OUT] $(basename "$GB_BIN")"
echo "[OUT] $(basename "$BEACON_BIN")"
