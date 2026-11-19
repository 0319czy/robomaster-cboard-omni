"""只读采集底盘低频波形：不复位、不 halt、不写 RAM、不发送动作。

状态发布频率为 10 Hz，因此此工具适合观察回中、自旋和慢振荡，不能用于分析电机高频纹波。
--elf 必须指向板上对应镜像；采集后逐字节核验 Flash，不匹配时拒绝生成有效数据。
"""

import argparse, datetime, json, re
from pathlib import Path
from common import ROOT, probe_lock, openocd, tcl_path
from run_motion import symbols, signed, decode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seconds", type=float, default=10)
    p.add_argument("--stop-file", type=Path, help="创建此文件即可正常结束后台采集")
    p.add_argument("--elf", type=Path, default=ROOT / ".pio/build/cboard_chassis/firmware.elf")
    args = p.parse_args()
    if not 0.2 <= args.seconds <= 900:
        p.error("采集时间须为 0.2..900 秒")
    image = args.elf.with_suffix(".bin").read_bytes()
    syms = symbols(("g_chassis_status", "g_sbus", "g_imu"), args.elf)
    try:
        fault_addr = symbols(("g_teleop_fault",), args.elf)["g_teleop_fault"]
    except RuntimeError:
        fault_addr = None
    try:
        frame_addr = symbols(("g_rc_frame",), args.elf)["g_rc_frame"]
    except RuntimeError:
        frame_addr = None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    base = ROOT / "logs" / f"drive_capture_{stamp}"
    copies = [base.with_suffix(f".flash{i}.bin") for i in range(3)]
    addr = syms["g_chassis_status"]
    frame_read = (
        f" set frame [read_memory {frame_addr} 32 8];"
        if frame_addr is not None
        else " set frame {};"
    )
    snapshot = (
        "proc drive_sample {} {"
        + f" set addr {addr}; set seq {addr+8};"
        + " for {set n 0} {$n<30} {incr n} {"
        + " set a [lindex [read_memory $seq 32 1] 0]; if {$a&1} {sleep 2;continue};"
        + " set w [read_memory $addr 32 64];"
        + f' set rc [read_memory {syms["g_sbus"]} 32 20];'
        + f' set imu [read_memory {syms["g_imu"]} 32 12];'
        + frame_read
        + " set b [lindex [read_memory $seq 32 1] 0];"
        + ' if {$a==$b} {return [concat $w $rc $imu $frame]} }; error "状态读取不一致" }'
    )
    commands = ["init"] + [f"dump_image {tcl_path(f)} 0x08000000 {len(image)}" for f in copies]
    stop_condition = (
        (" && ![file exists " + tcl_path(args.stop_file.resolve()) + "]") if args.stop_file else ""
    )
    commands += [
        snapshot,
        "set until [expr {[clock milliseconds]+"
        + str(int(args.seconds * 1000))
        + "}];"
        + "while {[clock milliseconds]<$until"
        + stop_condition
        + '} {echo "DRIVE [join [drive_sample] ,]";sleep 100}',
        "shutdown",
    ]
    if fault_addr is not None:
        commands.insert(-1, f'echo "TFAULT [join [read_memory {fault_addr} 32 32] ,]"')
    with probe_lock():
        out = openocd(commands, base.with_suffix(".log"), timeout=int(args.seconds) + 40)
    if any(c.read_bytes() != image for c in copies[1:]):
        raise RuntimeError("板上 Flash 与指定 ELF 的配套 BIN 不一致；拒绝使用错误符号解释数据")
    samples = []
    for line in out.splitlines():
        if not line.startswith("DRIVE "):
            continue
        w = [int(v, 0) for v in line[6:].split(",")]
        if len(w) != (104 if frame_addr is not None else 96) or w[0] != 0x43485331 or w[1] != 1:
            raise RuntimeError("状态结构不匹配")
        row = decode(w[:64])
        row.update(
            rc_channels=w[66:82],
            rc_flags=w[82],
            rc_frames=w[65],
            rc_armed=w[61],
            rc_errors=w[83],
            gyro_z_mrad_s=signed(w[88]),
            yaw_mrad=signed(w[90]),
            gyro_bias_mrad_s=signed(w[91]),
            gyro_cal_samples=w[92],
            heading_mrad=signed(w[93]),
            heading_correction_mrad_s=signed(w[94]),
            sensor_raw_gyro_z_mrad_s=signed(w[95]),
        )
        if frame_addr is not None:
            if w[96] != 0x52434631:
                raise RuntimeError("参考航向诊断结构不匹配")
            row.update(
                control_reference_mrad=signed(w[97]),
                body_relative_reference_mrad=signed(w[98]),
                right_turn_mrad_s=signed(w[99]),
                sb_spin_mrad_s=signed(w[100]),
                combined_turn_mrad_s=signed(w[101]),
            )
        samples.append(row)
    fault = None
    for line in out.splitlines():
        if line.startswith("TFAULT "):
            f = [int(v, 0) for v in line[7:].split(",")]
            if f[0] == 0x54464C54 and f[2] and not f[2] & 1:
                fault = dict(
                    time_ms=f[3],
                    dt_ms=f[4],
                    trip_rpm=signed(f[5]) / 1000,
                    channel=f[6],
                    hard_limit=bool(f[7]),
                    target_rpm=[signed(v) / 1000 for v in f[8:12]],
                    raw_rpm=[signed(v) for v in f[12:16]],
                    filtered_rpm=[signed(v) / 1000 for v in f[16:20]],
                    integral_raw=[signed(v) / 1000 for v in f[20:24]],
                    overspeed_since_ms=f[24:28],
                )
    path = base.with_suffix(".json")
    path.write_text(
        json.dumps(
            dict(sample_rate_limit_hz=10, elf=str(args.elf), fault_snapshot=fault, samples=samples),
            ensure_ascii=False,
            indent=2,
        )
    )
    print("采集：", path)
    print(json.dumps(samples[-1] if samples else {}, ensure_ascii=False))


if __name__ == "__main__":
    main()
