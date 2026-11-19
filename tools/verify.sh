#!/usr/bin/env bash
# 工程自检入口：主机端单元测试 → 固件构建 → (可选)板上固件与本地镜像比对
# 用法: tools/verify.sh [--with-board]
set -uo pipefail
cd "$(dirname "$0")/.."
if [[ -z "${PIO:-}" ]]; then
    if command -v pio >/dev/null 2>&1; then
        PIO=$(command -v pio)
    else
        PIO="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}/penv/bin/pio"
    fi
fi
if [[ ! -x "$PIO" ]]; then
    echo "未找到PlatformIO，请将pio加入PATH或设置PIO路径。" >&2
    exit 1
fi
fail=0

echo "==================== [1/3] 主机端单元测试 (gcc) ===================="
if gcc -std=c11 -Wall -Wextra -Werror -Iinclude \
        src/chassis_control.c src/omni_x45.c src/rc_drive.c tests/control_test.c \
        -lm -o /tmp/chassis_control_test; then
    echo "编译: 通过"
    if /tmp/chassis_control_test; then echo "单元测试: 通过"; else echo "单元测试: 失败"; fail=1; fi
else
    echo "编译: 失败"; fail=1
fi

if gcc -std=c11 -Wall -Wextra -Werror -Iinclude src/drive_trace.c tests/drive_trace_test.c -o /tmp/drive_trace_test && /tmp/drive_trace_test; then
    echo "高速记录测试: 通过"
else
    echo "高速记录测试: 失败"; fail=1
fi

echo
echo "==================== [2/3] 固件构建 (PlatformIO) ===================="
if "$PIO" run -e cboard_chassis 2>&1 | tail -5; then
    echo "构建: 通过"
else
    echo "构建: 失败"; fail=1
fi
python3 - <<'PY'
d=open('.pio/build/cboard_chassis/firmware.bin','rb').read()
sp=int.from_bytes(d[:4],'little'); ep=int.from_bytes(d[4:8],'little')
ok = 0x20000000 < sp <= 0x20020000 and 0x08000000 < ep < 0x08040000 and ep & 1
print(f"镜像: {len(d)} 字节  向量: SP=0x{sp:08X} 入口=0x{ep:08X} -> {'有效' if ok else '无效'}")
raise SystemExit(0 if ok else 1)
PY
[ $? -ne 0 ] && fail=1

echo
echo "==================== [3/3] 板上固件与本地镜像比对 ===================="
if [ "${1:-}" = "--with-board" ]; then
    python3 - <<'PY'
import sys, hashlib
sys.path.insert(0, 'tools')
from common import ROOT, probe_lock, openocd, tcl_path
from pathlib import Path

image = (ROOT/'.pio/build/cboard_chassis/firmware.bin').read_bytes()
# 不复位、不 halt，避免校验操作打断或重新解锁遥控。首次读取可能不稳定，
# 因此同一会话连读三次：丢弃第 1 次，要求第 2、3 次互相一致，再与镜像比对。
outs = [Path(f'/tmp/board_readback_{i}.bin') for i in (1, 2, 3)]
cmds = ['init'] + [f'dump_image {tcl_path(o)} 0x08000000 {len(image)}' for o in outs] + \
       ['shutdown']
with probe_lock():
    openocd(cmds, ROOT/'logs'/'verify_readback.log')
reads = [o.read_bytes() for o in outs]
stable = reads[1] == reads[2]
same = reads[2] == image
print(f"本地镜像 {len(image)} 字节  SHA256 {hashlib.sha256(image).hexdigest()}")
for i, r in enumerate(reads, 1):
    note = "  <- 首次读取，可能含脏数据，按约定丢弃" if i == 1 else ""
    print(f"第{i}次读回 {len(r)} 字节  SHA256 {hashlib.sha256(r).hexdigest()}{note}")
print("后两次读回互相一致:", "是 ✔" if stable else "否 ✘（读取不稳定，本轮结论不可信）")
print("板上固件 == 本地镜像:", "是 ✔（板上运行的就是本次构建）" if same else "否 ✘")
raise SystemExit(0 if (stable and same) else 1)
PY
    [ $? -ne 0 ] && fail=1
else
    echo "跳过（加 --with-board 可读回板上固件逐字节比对，需要下载器空闲）"
fi

echo
if [ $fail -eq 0 ]; then echo "结论: 全部通过"; else echo "结论: 有失败项"; fi
exit $fail
