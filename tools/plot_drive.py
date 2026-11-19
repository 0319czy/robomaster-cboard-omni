"""将 capture_drive.py 的只读采集结果画成低频控制波形。"""

import argparse, json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    rows = json.loads(args.capture.read_text())["samples"]
    if not rows:
        raise RuntimeError("采集文件无数据")
    # 以固件时间戳作横轴，重复读取同一发布帧时只保留一次。
    rows = list({r["uptime_ms"]: r for r in rows}.values())
    t = [(r["uptime_ms"] - rows[0]["uptime_ms"]) / 1000 for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    colors = ["C0", "C1", "C2", "C3"]
    for i in range(4):
        axes[0].plot(
            t,
            [r["rotor_rpm"][i] / 19.203208556 for r in rows],
            color=colors[i],
            label=f"ID{i+1} measured",
        )
        axes[0].plot(
            t,
            [r["target_rotor_rpm"][i] / 19.203208556 for r in rows],
            "--",
            alpha=0.5,
            color=colors[i],
            label=f"ID{i+1} target",
        )
        axes[1].plot(t, [r["command_raw"][i] for r in rows], color=colors[i], label=f"ID{i+1}")
    axes[0].set_ylabel("Wheel speed (rpm)")
    axes[1].set_ylabel("Current command (raw)")
    axes[2].plot(t, [r["cmd"]["omega_mrad_s"] / 1000 for r in rows], label="Command omega")
    axes[2].plot(
        t, [(r["gyro_z_mrad_s"] - r["gyro_bias_mrad_s"]) / 1000 for r in rows], label="Gyro omega"
    )
    axes[2].plot(
        t, [r["heading_correction_mrad_s"] / 1000 for r in rows], label="Heading correction"
    )
    axes[2].set_ylabel("Angular rate (rad/s)")
    axes[3].plot(t, [r["cmd"]["vx_mm_s"] for r in rows], label="vx command")
    axes[3].plot(t, [r["cmd"]["vy_mm_s"] for r in rows], label="vy command")
    axes[3].set_ylabel("Translation (mm/s)")
    axes[3].set_xlabel("Firmware time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=4, fontsize=8)
    fig.suptitle("Chassis control capture (status publication <= 10 Hz)")
    fig.tight_layout()
    out = args.out or args.capture.with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
