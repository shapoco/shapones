#!/usr/bin/env bash
#
#   ./upload.sh [PORT] [BAUD]
#
# PORT は省略すると $PORT または /dev/ttyACM0 (Windows では COM3 など)、
# BAUD は省略すると $BAUD または 921600。esptool が必要 (pip install esptool)。
#
# イメージはブートローダ、パーティションテーブル、アプリを結合したもので、0x0 に一度に書く。
# ポートが現れないときは、リセットボタンを緑の LED が点くまで (約 3 秒) 長押ししてダウンロードモードにする。

set -eu

PORT="${1:-${PORT:-/dev/ttyACM0}}"
BAUD="${2:-${BAUD:-921600}}"
BIN="$(cd "$(dirname "$0")" && pwd)/shapones_for_xiamocon_esp32s3.factory.bin"

[ -f "$BIN" ] || { echo "firmware not found: $BIN" >&2; exit 1; }

# esptool は esptool、esptool.py、python のモジュールのいずれかで入っている
if command -v esptool >/dev/null 2>&1; then
  ESPTOOL="esptool"
elif command -v esptool.py >/dev/null 2>&1; then
  ESPTOOL="esptool.py"
elif python3 -m esptool version >/dev/null 2>&1; then
  ESPTOOL="python3 -m esptool"
else
  echo "esptool not found. Install it with: pip install esptool" >&2
  exit 1
fi

# esptool 5 でサブコマンド名が変わった (write_flash -> write-flash)
MAJOR="$($ESPTOOL version 2>/dev/null | grep -oE '[0-9]+' | head -1 || true)"
if [ "${MAJOR:-4}" -ge 5 ]; then WRITE=write-flash; else WRITE=write_flash; fi

set -x
$ESPTOOL --chip esp32s3 --port "$PORT" --baud "$BAUD" "$WRITE" 0x0 "$BIN"
