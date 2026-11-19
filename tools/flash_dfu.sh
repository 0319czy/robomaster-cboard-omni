#!/usr/bin/env bash
# 历史可选DFU入口，本轮未验收；此脚本不做独立回读校验。
# 通过 STM32 ROM DFU 烧写（绕过 SWD/DAP）：micro USB + BOOT0=1
# 前提: C 板 BOOT0 跳线置 1、按过复位、micro USB 连到电脑
# 用法: tools/flash_dfu.sh [firmware.bin]
set -uo pipefail
cd "$(dirname "$0")/.."
IMG=${1:-.pio/build/cboard_chassis/firmware.bin}

echo "===== 1) 查找 STM32 BOOTLOADER (0483:df11) ====="
if ! lsusb | grep -qi "0483:df11"; then
    echo "未发现 DFU 设备。请确认："
    echo "  1. C 板 BOOT 配置接口的 BOOT0 置 1（跳线帽换位或短接）"
    echo "  2. 按过复位键（或断电重上电）"
    echo "  3. micro USB 数据线已连到电脑（不是只能充电的线）"
    lsusb | grep -iE "0483|stm" || true
    exit 1
fi
lsusb | grep -i "0483:df11"

echo
echo "===== 2) DFU 设备详情 ====="
dfu-util -l 2>&1 | tail -6

echo
echo "===== 3) 烧写 $IMG ====="
ls -l "$IMG"
dfu-util -a 0 -s 0x08000000:leave -D "$IMG" 2>&1 | tail -12
rc=$?

echo
if [ $rc -eq 0 ]; then
    echo "烧写完成。请把 BOOT0 跳线恢复为 0，然后按一次复位（或断电重上电）。"
else
    echo "烧写失败（退出码 $rc），检查上面输出。"
fi
exit $rc
