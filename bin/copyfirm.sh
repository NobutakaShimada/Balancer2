#!/bin/bash
set -euo pipefail

DEVICE=/dev/sdb
MNT=/media/balancer2
FILE=firmware.bin

# 引数チェック
if [ $# -ne 1 ]; then
  echo "Usage: $0 <firmware.bin>" >&2
  exit 1
fi
FW="$1"

# マウント済フラグ
MOUNTED=0

# エラー時の後片付け
cleanup() {
  if [ "$MOUNTED" -eq 1 ]; then
    echo "エラー発生 → アンマウント実行: ${MNT}" >&2
    umount "$MNT" || echo "アンマウント失敗…" >&2
  fi
}
trap cleanup ERR

# マウント
echo "マウント: ${DEVICE} → ${MNT}"
mount -t msdos "$DEVICE" "$MNT"
MOUNTED=1
echo "  → OK"

# dd書き込み
echo "dd書き込み: ${FW} → ${MNT}"
dd if="$FW" of="$MNT/$FILE" \
   bs=1024 \
   conv=notrunc,fsync \
   oflag=direct \
   status=progress
echo "  → dd OK"

# 正常終了前のアンマウント
echo "アンマウント: ${MNT}"
umount "$MNT"
MOUNTED=0
echo "  → アンマウント完了"

echo "firmware ${FW} copied into Balancer2."
