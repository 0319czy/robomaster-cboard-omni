"""板内同步高速记录：只配置观察缓冲区，不复位、不halt、不发送电机动作。
先校验板上Flash与本地镜像一致；只在冻结后导出，后两次RAM读回必须一致。
"""

import argparse, datetime, hashlib, json, re, struct, time
from pathlib import Path
from common import ROOT, probe_lock, openocd, tcl_path
from run_motion import symbols

FRAME = struct.Struct("<IHH33h7HII")
MAGIC = 0x54524331


def decode_frame(data):
    v = FRAME.unpack(data)
    r = dict(
        uptime_ms=v[0],
        step_dt_ms=v[1],
        imu_age_ms=v[2],
        target_rotor_rpm=list(v[3:7]),
        rotor_rpm=list(v[7:11]),
        filtered_rotor_rpm=list(v[11:15]),
        command_raw=list(v[15:19]),
        queued_current_raw=list(v[19:23]),
        feedback_current_raw=list(v[23:27]),
        gyro_x_mrad_s=v[27],
        gyro_y_mrad_s=v[28],
        gyro_z_mrad_s=v[29],
        gyro_bias_mrad_s=0,
        yaw_mrad=v[30],
        control_reference_mrad=v[31],
        body_relative_reference_mrad=v[32],
        cmd=dict(vx_mm_s=v[33], vy_mm_s=v[34], omega_mrad_s=v[35]),
        combined_turn_mrad_s=v[35],
        state=v[36],
        fault=v[37],
        feedback_age_ms=list(v[38:42]),
        loop_work_ms=v[42],
        sequence=v[43],
        tx_fail=v[44],
    )
    return r


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interval-ms", type=int, default=10, choices=range(4, 21))
    p.add_argument("--cancel", action="store_true", help="只冻结采集，绝不停止或驱动电机")
    p.add_argument(
        "--immediate", action="store_true", help="立即记录静止基线；默认等合成自旋指令>1rad/s"
    )
    p.add_argument("--timeout", type=int, default=180)
    p.add_argument("--ready-file", type=Path)
    args = p.parse_args()
    if not 10 <= args.timeout <= 600:
        p.error("timeout须为10..600秒")
    elf = ROOT / ".pio/build/cboard_chassis/firmware.elf"
    image = elf.with_suffix(".bin").read_bytes()
    syms = symbols(("g_trace_meta", "g_trace_frames"), elf)
    meta, frames = syms["g_trace_meta"], syms["g_trace_frames"]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    base = ROOT / "logs" / f"fast_capture_{stamp}"
    flashes = [base.with_suffix(f".flash{i}.bin") for i in range(3)]
    dumps = [base.with_suffix(f".ram{i}.bin") for i in range(3)]
    ready = args.ready_file or base.with_suffix(".ready")
    if ready.exists():
        raise RuntimeError("ready-file已经存在，请使用本次的新路径")
    token = int(time.time() * 1000) & 0x3FFFFFFF or 1
    with probe_lock():
        # RAM写入之前必须完成校验；不能等采完再发现使用了错误的符号地址。
        openocd(
            ["init"]
            + [f"dump_image {tcl_path(f)} 0x08000000 {len(image)}" for f in flashes]
            + ["shutdown"],
            base.with_suffix(".preflight.log"),
        )
        if any(f.read_bytes() != image for f in flashes[1:]):
            raise RuntimeError("板上固件与ELF不匹配，没有写入采集令牌")
        if args.cancel:
            openocd(
                [
                    "init",
                    f'set m [read_memory {meta} 32 16]; if {{[lindex $m 0]!={MAGIC}}} {{error "记录器结构不匹配"}}',
                    f"mww {meta+52} 2",
                    f"mww {meta+48} {token}",
                    "sleep 30",
                    f'echo "FASTCANCEL [join [read_memory {meta} 32 16] ,]"',
                    "shutdown",
                ],
                base.with_suffix(".cancel.log"),
            )
            print("采集已冻结；未向电机发送停止或动作命令", flush=True)
            return
        mode = 0 if args.immediate else 1
        verify = (
            f"set m [read_memory {meta} 32 16]; "
            f"if {{[lindex $m 0]!={MAGIC} || [lindex $m 1]!=1 || [lindex $m 2]!={FRAME.size} || [lindex $m 3]!=1000}} "
            '{error "记录器结构不匹配"}; '
            'if {[lindex $m 4]==1 || [lindex $m 4]==2} {error "记录器已被占用"}'
        )
        arm = [
            f"mww {meta+52} {mode}",
            f"mww {meta+56} {args.interval_ms}",
            f"mww {meta+48} {token}",
            "sleep 30",
            f'set m [read_memory {meta} 32 16]; if {{[lindex $m 6]!={token} || ([lindex $m 4]!=1 && [lindex $m 4]!=2)}} {{error "采集布置失败"}}',
            f"set f [open {tcl_path(ready)} w]; puts $f READY; close $f",
        ]
        wait = (
            "set deadline [expr {[clock milliseconds]+" + str(args.timeout * 1000) + "}]; "
            f"while {{[clock milliseconds]<$deadline}} {{set m [read_memory {meta} 32 16]; "
            "if {[lindex $m 4]==3} {break}; sleep 100}; "
            "if {[lindex $m 4]!=3} {"
            f"mww {meta+52} 2; mww {meta+48} {token+1}; sleep 30; set m [read_memory {meta} 32 16]"
            '}; if {[lindex $m 4]!=3} {error "未冻结，拒绝读出活动缓冲"}; '
            'set count [lindex $m 5]; if {$count<=0 || $count>1000} {error "没有触发或样本数无效"}; '
            f'set bytes [expr {{$count*{FRAME.size}}}]; echo "FASTMETA [join $m ,]"'
        )
        commands = (
            ["init", verify]
            + arm
            + [wait]
            + [f"dump_image {tcl_path(f)} {frames} $bytes" for f in dumps]
            + [f'echo "FASTEND [join [read_memory {meta} 32 16] ,]"', "shutdown"]
        )
        print("等待板内记录触发；就绪标记：" + str(ready), flush=True)
        output = openocd(commands, base.with_suffix(".log"), timeout=args.timeout + 90)
    a, b = dumps[1].read_bytes(), dumps[2].read_bytes()
    if a != b or not a or len(a) % FRAME.size:
        raise RuntimeError("冻结RAM读回不一致，拒绝生成有效采样")

    def line_meta(name):
        line = next(l for l in output.splitlines() if l.startswith(name + " "))
        return [int(x, 0) for x in line.split(" ", 1)[1].split(",")]

    start, end = line_meta("FASTMETA"), line_meta("FASTEND")
    if start != end or start[4] != 3 or start[5] != len(a) // FRAME.size:
        raise RuntimeError("导出期间记录状态发生变化")
    rows = [decode_frame(a[i : i + FRAME.size]) for i in range(0, len(a), FRAME.size)]
    if any(rows[i]["uptime_ms"] <= rows[i - 1]["uptime_ms"] for i in range(1, len(rows))):
        raise RuntimeError("记录时间戳不是严格递增")
    result = dict(
        sample_rate_limit_hz=1000 / args.interval_ms,
        elf=str(elf),
        firmware_sha256=hashlib.sha256(image).hexdigest(),
        trace_meta=start,
        fault_snapshot=None,
        notes=[
            "板内同周期快照；CAN反馈与IMU仍有各自物理采样时刻，陈旧时间单独保存。",
            "gyro_z_mrad_s已经扣零偏，gyro_bias_mrad_s固定为0以兼容旧分析程序。",
            "未单独记录SB/右摇杆输入；combined_turn_mrad_s为合成旋转指令。",
            "feedback_current_raw为C620反馈的转矩电流原始值，不能当作直接相电流测量。",
            "command_raw为本轮PI输出，queued_current_raw为最近提交CAN的输出。",
        ],
        samples=rows,
    )
    path = base.with_suffix(".json")
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    duration = (rows[-1]["uptime_ms"] - rows[0]["uptime_ms"]) / 1000
    print("采集：" + str(path), flush=True)
    print(
        json.dumps(
            dict(
                samples=len(rows),
                duration_s=duration,
                effective_hz=(len(rows) - 1) / duration if duration else 0,
                max_step_dt_ms=max(r["step_dt_ms"] for r in rows),
                max_loop_work_ms=max(r["loop_work_ms"] for r in rows),
                final=rows[-1],
            ),
            ensure_ascii=False,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
