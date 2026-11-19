"""分析只读采集中的恒定自旋段与回中停车；不执行任何设备操作。"""

import argparse, json, math, statistics
from pathlib import Path

K = 0.7071067812 * 9.549296586 * 19.203208556 / 0.075


def mean(v):
    return statistics.mean(v) if v else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    args = p.parse_args()
    rows = json.loads(args.capture.read_text())["samples"]
    rows = list({r["uptime_ms"]: r for r in rows}.values())
    groups = []
    for r in rows:
        om = r["cmd"]["omega_mrad_s"]
        if not groups or groups[-1][0] != om:
            groups.append([om, []])
        groups[-1][1].append(r)
    segments = []
    for idx, (om, rs) in enumerate(groups):
        if not om:
            continue
        steady = [
            r
            for r in rs
            if r["uptime_ms"] - rs[0]["uptime_ms"] >= 800
            and r["state"] == 5
            and not r["fault"]
            and any(r["target_rotor_rpm"])
        ]
        gyro = [(r["gyro_z_mrad_s"] - r["gyro_bias_mrad_s"]) / 1000 for r in steady]
        wheel = [-sum(r["rotor_rpm"]) / (4 * K * 0.31) for r in steady]
        x = [
            (-r["rotor_rpm"][0] - r["rotor_rpm"][1] + r["rotor_rpm"][2] + r["rotor_rpm"][3])
            / (4 * K)
            for r in steady
        ]
        y = [
            (-r["rotor_rpm"][0] + r["rotor_rpm"][1] + r["rotor_rpm"][2] - r["rotor_rpm"][3])
            / (4 * K)
            for r in steady
        ]
        stopped = None
        if idx + 1 < len(groups) and groups[idx + 1][0] == 0:
            stop_rows = groups[idx + 1][1]
            for r in stop_rows:
                if max(map(abs, r["rotor_rpm"])) <= 8 and not any(r["command_raw"]):
                    stopped = r["uptime_ms"] - stop_rows[0]["uptime_ms"]
                    break
        segments.append(
            dict(
                command_omega_mrad_s=om,
                recorded_duration_ms=rs[-1]["uptime_ms"] - rs[0]["uptime_ms"],
                steady_samples=len(steady),
                mean_rotor_rpm=[mean([r["rotor_rpm"][i] for r in steady]) for i in range(4)],
                mean_gyro_omega_rad_s=mean(gyro),
                mean_wheel_omega_rad_s=mean(wheel),
                gyro_std_rad_s=statistics.pstdev(gyro) if gyro else None,
                wheel_inverse_body_vx_mm_s=1000 * mean(x) if x else None,
                wheel_inverse_body_vy_mm_s=1000 * mean(y) if y else None,
                recorded_stop_delay_ms=stopped,
            )
        )
    result = dict(
        segments=segments,
        states=sorted(set(r["state"] for r in rows)),
        faults=sorted(set(r["fault"] for r in rows)),
        max_current_command_raw=max(abs(v) for r in rows for v in r["command_raw"]),
        max_heading_correction_mrad_s=max(abs(r["heading_correction_mrad_s"]) for r in rows),
        tx_fail_delta=rows[-1]["tx_fail"] - rows[0]["tx_fail"],
        final=rows[-1],
        note="10 Hz 状态发布；停车延迟为采样分辨率内的估计。轮速逆解不是地面轨迹或轮胎打滑测量。",
    )
    out = args.capture.with_suffix(".analysis.json")
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
