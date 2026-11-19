"""全向底盘限时 RAM 动作试验。遥控器启用时由遥控优先；RAM 动作需释放遥控占用。

动作预设（车体系，yaw=0）：
  fwd/back/left/right  = 平移 ±500 mm/s
  ccw/cw               = 自旋 ±1400 mrad/s（逆/顺时针，俯视）
  mix-ccw-fwd          = 前进 + 逆时针自旋叠加

示例：
  python3 tools/run_motion.py --status
  python3 tools/run_motion.py --motion fwd --duration 4000
  python3 tools/run_motion.py --vx 120 --vy 0 --omega 300 --duration 5000
  python3 tools/run_motion.py --stop
"""

import argparse, datetime, json, re, subprocess, sys, time
from pathlib import Path
from common import ROOT, probe_lock, openocd, tcl_path
from hardware_config import ARM_NM

STATUS_WORDS = 64
MAGIC = 0x43485331
PRESETS = {
    "fwd": dict(vx=500, vy=0, omega=0),  # 0.5 m/s
    "back": dict(vx=-500, vy=0, omega=0),
    "left": dict(vx=0, vy=500, omega=0),
    "right": dict(vx=0, vy=-500, omega=0),
    "ccw": dict(vx=0, vy=0, omega=1400),  # 1.4 rad/s
    "cw": dict(vx=0, vy=0, omega=-1400),
    "mix-ccw-fwd": dict(vx=400, vy=0, omega=1200),
    "slow-fwd": dict(vx=100, vy=0, omega=0),
    "slow-ccw": dict(vx=0, vy=0, omega=300),
}


def symbols(names=("g_chassis_status", "g_cmd", "g_stop_request"), elf=None):
    nm = ARM_NM
    elf = Path(elf) if elf else ROOT / ".pio/build/cboard_chassis/firmware.elf"
    text = subprocess.check_output([str(nm), "-S", str(elf)], text=True)
    found = {}
    for name in names:
        match = re.search(r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\w\s+" + name + r"$", text, re.M)
        if not match:
            raise RuntimeError("ELF 缺少符号 " + name)
        found[name] = int(match[1], 16)
    return found


def signed(v):
    return v if v < 0x80000000 else v - 0x100000000


def decode(w):
    return {
        "uptime_ms": w[3],
        "state": w[4],
        "fault": w[5],
        "fault_channel": w[6],
        "elapsed_ms": w[7],
        "duration_ms": w[8],
        "ack_token": w[9],
        "done_token": w[10],
        "target_rotor_rpm": [signed(v) / 1000.0 for v in w[11:15]],
        "rotor_rpm": [signed(v) for v in w[15:19]],
        "command_raw": [signed(v) for v in w[19:23]],
        "temperature_c": list(w[23:27]),
        "feedback_age_ms": list(w[27:31]),
        "can_error": w[31],
        "rx_total": w[32],
        "tx_success": w[33],
        "tx_fail": w[34],
        "other_controller": w[35],
        "peak_rotor_rpm": w[36],
        "peak_command_raw": w[37],
        "cmd": {"vx_mm_s": signed(w[38]), "vy_mm_s": signed(w[39]), "omega_mrad_s": signed(w[40])},
        "wheel_ticks": [signed(v) for v in w[41:45]],
        "filtered_rotor_rpm": [signed(v) / 1000.0 for v in w[45:49]],
    }


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--motion", choices=sorted(PRESETS))
    p.add_argument("--vx", type=int, help="车体系前进速度 mm/s，正=前")
    p.add_argument("--vy", type=int, help="车体系左移速度 mm/s，正=左")
    p.add_argument("--omega", type=int, help="自旋 mrad/s，正=逆时针")
    p.add_argument("--duration", type=int, default=4000, help="动作时长 ms（200~20000）")
    p.add_argument("--status", action="store_true")
    p.add_argument("--stop", action="store_true")
    p.add_argument(
        "--reset",
        action="store_true",
        help="复位清故障；请先遥控全回中，上电 500 ms 中位解锁后仍由遥控器控制",
    )
    args = p.parse_args()

    manual = any(v is not None for v in (args.vx, args.vy, args.omega))
    if sum([bool(args.motion or manual), args.status, args.stop, args.reset]) != 1:
        p.error("选择 --motion / (--vx/--vy/--omega) / --status / --stop 之一")
    if args.motion:
        vx, vy, omega = (
            PRESETS[args.motion]["vx"],
            PRESETS[args.motion]["vy"],
            PRESETS[args.motion]["omega"],
        )
    else:
        vx, vy, omega = args.vx or 0, args.vy or 0, args.omega or 0
    if not (200 <= args.duration <= 20000):
        p.error("--duration 必须 200~20000 ms")
    if max(abs(vx), abs(vy)) > 1200 or abs(omega) > 4000:
        p.error("超出固件限幅：|vx|,|vy| ≤ 1200 mm/s，|omega| ≤ 4000 mrad/s")

    syms = symbols()
    status_addr = syms["g_chassis_status"]
    cmd_addr = syms["g_cmd"]
    stop_addr = syms["g_stop_request"]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    name = args.motion or ("manual" if manual else ("stop" if args.stop else "status"))
    log = ROOT / "logs" / f"motion_{name}_{stamp}.log"

    # 快照：先读序号(偶数=一致)，读 64 字，再读序号确认；不 halt、不 reset。
    snapshot = (
        "proc snapshot {} {"
        + f" set addr {status_addr}; set seq {status_addr + 8};"
        + " for {set n 0} {$n<30} {incr n} {"
        + " set a [lindex [read_memory $seq 32 1] 0]; if {$a&1} {sleep 2;continue};"
        + f" set w [read_memory $addr 32 {STATUS_WORDS}]; set b [lindex [read_memory $seq 32 1] 0];"
        + f' if {{$a==$b}} {{if {{[lindex $w 0]!={MAGIC} || [lindex $w 1]!=1}} {{error "固件不匹配"}};return $w}} }};'
        'error "状态一致性读取失败" }'
    )
    commands = ["init", snapshot]

    if args.reset:
        commands = ["init", "reset run", "sleep 500", snapshot, 'echo "SAMPLE [join [snapshot] ,]"']
    elif args.stop:
        commands += ["mww %d 1" % stop_addr, "sleep 500", 'echo "SAMPLE [join [snapshot] ,]"']
    elif args.status:
        commands += ["sleep 200", 'echo "SAMPLE [join [snapshot] ,]"']
    else:
        token = int(time.time() * 1000) & 0x7FFFFFFF or 1
        pre = (
            "set w [snapshot]; "
            'if {[lindex $w 4]!=0 && [lindex $w 4]!=3} {error "当前非待命或完成状态"}; '
            'if {[lindex $w 5]!=0} {error "存在锁定故障,需复位"}; '
            "for {set i 0} {$i<4} {incr i} {"
            'if {[lindex $w [expr {27+$i}]]>30} {error "电机反馈不新鲜"}; '
            "set r [lindex $w [expr {15+$i}]]; if {$r&0x80000000} {set r [expr {$r-0x100000000}]}; "
            'if {abs($r)>8} {error "电机未停稳"}}; '
        )
        commands += [
            pre,
            f"mww {cmd_addr} 0x43484331",  # g_cmd[0] magic 'CHC1'
            f"mww {cmd_addr + 4} 1",  # g_cmd[1] version
            f"mww {cmd_addr + 8} {vx}",  # g_cmd[2]
            f"mww {cmd_addr + 12} {vy}",  # g_cmd[3]
            f"mww {cmd_addr + 16} {omega}",  # g_cmd[4]
            f"mww {cmd_addr + 20} {args.duration}",  # g_cmd[5]
            f"mww {cmd_addr + 24} {token}",  # g_cmd[6] 最后提交
            "set finished 0; for {set step 0} {$step<120} {incr step} {"
            ' sleep 250; set w [snapshot]; echo "SAMPLE [join $w ,]";'
            f' if {{[lindex $w 9]!={token}}} {{error "触发未确认或 MCU 复位"}};'
            ' if {[lindex $w 4]==4 || [lindex $w 5]!=0} {error "保护触发"};'
            f" if {{[lindex $w 4]==3 && [lindex $w 10]=={token}}} {{set finished 1;break}} }};"
            f' if {{!$finished}} {{mww {stop_addr} 1;error "动作未在期限内完成"}}',
        ]
    commands += ["shutdown"]

    error = None
    text = ""
    with probe_lock():
        if not args.status:
            # 另一个智能体可能刚改过固件布局。写 RAM 或复位前必须确认本地镜像
            # 正是板上版本，防止错误 ELF 地址把停车标志写到其它变量上。
            image = (ROOT / ".pio/build/cboard_chassis/firmware.bin").read_bytes()
            reads = [log.with_suffix(f".preflight{i}.bin") for i in range(3)]
            openocd(
                ["init"]
                + [f"dump_image {tcl_path(p)} 0x08000000 {len(image)}" for p in reads]
                + ["shutdown"],
                log.with_suffix(".preflight.log"),
                timeout=30,
            )
            if any(p.read_bytes() != image for p in reads[1:]):
                raise RuntimeError(
                    "板上镜像与本地 ELF 配套 BIN 不符；拒绝写 RAM/复位，请先核对部署版本"
                )
        try:
            text = openocd(commands, log, timeout=60)
        except Exception as exc:
            error = exc
            if log.is_file():
                text = log.read_text()
            elif isinstance(exc, subprocess.TimeoutExpired):
                parts = [part for part in (exc.stdout, exc.stderr) if part]
                text = "\n".join(
                    p.decode(errors="replace") if isinstance(p, bytes) else p for p in parts
                )
                log.write_text(text)
            if args.motion or manual:
                # 触发后出错，补一次带校验的停机，避免电机继续转。
                guarded = (
                    "set w [snapshot]; "
                    f"if {{[lindex $w 9]=={token}}} {{mww {stop_addr} 1; sleep 150}}; "
                    'echo "SAMPLE [join [snapshot] ,]"'
                )
                try:
                    text += "\n" + openocd(
                        ["init", snapshot, guarded, "shutdown"],
                        log.with_suffix(".stop.log"),
                        timeout=10,
                    )
                except Exception as rescue_error:
                    print("停止复核失败：", rescue_error, file=sys.stderr)

    rows = []
    for sample in (text or "").splitlines():
        if sample.startswith("SAMPLE "):
            values = [int(v, 0) for v in sample[7:].split(",")]
            if len(values) == STATUS_WORDS and values[:2] == [MAGIC, 1]:
                rows.append(decode(values))
    if not rows and error is None:
        error = RuntimeError("没有有效遥测")
    result = {
        "motion": name,
        "command": {"vx": vx, "vy": vy, "omega": omega, "duration_ms": args.duration},
        "success": error is None,
        "snapshots": rows,
    }
    if error:
        result["error"] = str(error)
    result["first_fault"] = next((row for row in rows if row["fault"]), None)
    log.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    print("记录：", log.with_suffix(".json"))
    if error:
        raise error


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
