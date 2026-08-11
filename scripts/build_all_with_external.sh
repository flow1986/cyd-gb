#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTERNAL_DIR="$ROOT_DIR/beacon-local"

if [[ ! -f "$EXTERNAL_DIR/platformio.ini" ]]; then
    echo "[ERROR] Local beacon project not found: $EXTERNAL_DIR" >&2
    exit 1
fi

echo "[1/2] Build main project (env: cyd)"
pio run -d "$ROOT_DIR" -e cyd

echo "[2/2] Build external beacon project (env: esp32dev_sender)"
pio run -d "$EXTERNAL_DIR" -e esp32dev_sender

GB_BIN="$(ls -1t "$ROOT_DIR"/builds/gbscanner-*.bin | head -n 1)"
BEACON_BIN="$(ls -1t "$EXTERNAL_DIR"/btsend-*.bin | head -n 1)"

cp "$GB_BIN" "$ROOT_DIR/"
cp "$BEACON_BIN" "$ROOT_DIR/"

echo "[OK] All builds finished successfully."
echo "[OUT] $(basename "$GB_BIN") -> $ROOT_DIR"
echo "[OUT] $(basename "$BEACON_BIN") -> $ROOT_DIR"
