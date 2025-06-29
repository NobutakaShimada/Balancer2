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

# 既存のマウント状態をチェック
echo "マウント状態をチェック中..."
if mount | grep -q "^${DEVICE} "; then
  # mountコマンドの出力から空白を含むパスを正しく抽出
  CURRENT_MOUNT=$(mount | grep "^${DEVICE} " | sed 's/^[^ ]* on \(.*\) type.*$/\1/')
  # エスケープされた空白を元に戻す（\040 → 空白）
  CURRENT_MOUNT=$(echo "$CURRENT_MOUNT" | sed 's/\\040/ /g')
  echo "  → ${DEVICE} は既に '${CURRENT_MOUNT}' にマウント済み"
  echo "  → アンマウント実行: '${CURRENT_MOUNT}'"
  umount "$CURRENT_MOUNT"
  echo "  → アンマウント完了"
else
  echo "  → ${DEVICE} はマウントされていません"
fi

# マウント
echo "マウント: ${DEVICE} → ${MNT}"
mount -t msdos "$DEVICE" "$MNT"
MOUNTED=1
echo "  → OK"

# dd書き込み
echo "dd ファームウェア書き込み: ${FW} → ${MNT}"
dd if="$FW" of="$MNT/$FILE" \
   bs=1024 \
   conv=notrunc,fsync \
   oflag=direct \
   status=progress
echo "  → ddファームウェア書き込み OK"

# 正常終了前のアンマウント
echo "アンマウント: ${MNT}"
umount "$MNT"
MOUNTED=0
echo "  → アンマウント完了"

echo "firmware ${FW} copied into Balancer2."

