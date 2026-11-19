"""画出右摇杆转向、SB自旋与平移参考航向；只读已采集的 JSON。"""

import argparse, json, math
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def unwrap(v):
    out = []
    last = None
    total = 0
    for x in v:
        if last is None:
            total = x
        else:
            total += math.remainder(x - last, 2 * math.pi)
        out.append(total)
        last = x
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    args = p.parse_args()
    d = json.loads(args.capture.read_text())
    rows = [r for r in d["samples"] if "control_reference_mrad" in r]
    if not rows:
        raise RuntimeError("文件没有参考航向数据")
    t = [(r["uptime_ms"] - rows[0]["uptime_ms"]) / 1000 for r in rows]
    ref = unwrap([r["control_reference_mrad"] / 1000 for r in rows])
    yaw = unwrap([r["yaw_mrad"] / 1000 for r in rows])
    fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    ax[0].plot(t, [x * 180 / math.pi for x in yaw], label="Body yaw")
    ax[0].plot(t, [x * 180 / math.pi for x in ref], label="Translation reference", lw=2)
    ax[0].set_ylabel("Heading (deg)")
    ax[1].plot(t, [r["right_turn_mrad_s"] / 1000 for r in rows], label="Right stick turn")
    ax[1].plot(t, [r["sb_spin_mrad_s"] / 1000 for r in rows], label="SB spin")
    ax[1].plot(t, [r["combined_turn_mrad_s"] / 1000 for r in rows], "k--", label="Combined command")
    ax[1].plot(
        t,
        [(r["gyro_z_mrad_s"] - r["gyro_bias_mrad_s"]) / 1000 for r in rows],
        alpha=0.5,
        label="Gyro",
    )
    ax[1].set_ylabel("Angular rate (rad/s)")
    ax[2].plot(t, [r["cmd"]["vx_mm_s"] for r in rows], label="Forward command in reference")
    ax[2].plot(t, [r["cmd"]["vy_mm_s"] for r in rows], label="Left command in reference")
    ax[2].set_ylabel("Translation (mm/s)")
    for i in range(4):
        ax[3].plot(t, [r["command_raw"][i] for r in rows], label=f"ID{i+1}")
    ax[3].set_ylabel("Current command (raw)")
    ax[3].set_xlabel("Firmware time (s)")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(ncol=4, fontsize=8)
    fig.suptitle("Right stick changes reference; SB spin preserves reference | status <= 10 Hz")
    fig.tight_layout()
    out = args.capture.with_suffix(".reference.png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
