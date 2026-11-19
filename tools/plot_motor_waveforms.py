"""将四个电机的转速/电流指令独立展开，并放大指定匀速段。"""

import argparse, json, statistics
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from plot_style import configure_plot_font

configure_plot_font()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    p.add_argument("--start-ms", type=int, required=True)
    p.add_argument("--end-ms", type=int, required=True)
    p.add_argument("--steady-from-ms", type=int, required=True)
    p.add_argument("--steady-to-ms", type=int, required=True)
    p.add_argument("--out-prefix", type=Path, required=True)
    a = p.parse_args()
    all_rows = json.loads(a.capture.read_text())["samples"]
    rows = list(
        {r["uptime_ms"]: r for r in all_rows if a.start_ms <= r["uptime_ms"] <= a.end_ms}.values()
    )
    steady = [r for r in rows if a.steady_from_ms <= r["uptime_ms"] <= a.steady_to_ms]
    if len(rows) < 3 or len(steady) < 3:
        raise RuntimeError("有效样本不足")
    ts = [(r["uptime_ms"] - a.start_ms) / 1000 for r in rows]
    dts = [(rows[i]["uptime_ms"] - rows[i - 1]["uptime_ms"]) / 1000 for i in range(1, len(rows))]
    hz = 1 / statistics.median(dts)
    names = ["1号 左后", "2号 左前", "3号 右前", "4号 右后"]
    fig, axs = plt.subplots(4, 2, figsize=(15, 12), sharex=True)
    metrics = []
    for i in range(4):
        ax = axs[i, 0]
        ax.plot(ts, [r["target_rotor_rpm"][i] for r in rows], "k--", label="目标")
        ax.plot(ts, [r["rotor_rpm"][i] for r in rows], color="0.6", lw=1, label="原始反馈")
        ax.plot(
            ts,
            [r["filtered_rotor_rpm"][i] for r in rows],
            color=f"C{i}",
            lw=1.4,
            label="8ms滤波反馈",
        )
        ax.set_ylabel("转子转速 / rpm")
        ax.set_title(names[i])
        ax = axs[i, 1]
        ax.plot(ts, [r["command_raw"][i] for r in rows], color=f"C{i}", label="电流指令")
        ax.axhline(2457, color="red", ls=":", alpha=0.6)
        ax.axhline(-2457, color="red", ls=":", alpha=0.6)
        ax.set_ylim(-2700, 2700)
        ax.set_ylabel("电流指令 / 原始单位")
        err = [r["rotor_rpm"][i] - r["target_rotor_rpm"][i] for r in steady]
        ferr = [r["filtered_rotor_rpm"][i] - r["target_rotor_rpm"][i] for r in steady]
        metrics.append(
            dict(
                id=i + 1,
                steady_samples=len(steady),
                target_mean=statistics.mean(r["target_rotor_rpm"][i] for r in steady),
                raw_rpm_mean=statistics.mean(r["rotor_rpm"][i] for r in steady),
                filtered_rpm_mean=statistics.mean(r["filtered_rotor_rpm"][i] for r in steady),
                raw_error_sd=statistics.pstdev(err),
                raw_error_rms=(statistics.mean(e * e for e in err)) ** 0.5,
                filtered_error_sd=statistics.pstdev(ferr),
                current_command_mean=statistics.mean(r["command_raw"][i] for r in steady),
                current_command_sd=statistics.pstdev(r["command_raw"][i] for r in steady),
                steady_saturated_samples=sum(abs(r["command_raw"][i]) >= 2457 for r in steady),
            )
        )
        for ax in axs[i]:
            ax.axvspan(
                (a.steady_from_ms - a.start_ms) / 1000,
                (a.steady_to_ms - a.start_ms) / 1000,
                color="green",
                alpha=0.06,
            )
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8, loc="best")
    for ax in axs[-1]:
        ax.set_xlabel("时间 / 秒")
    fig.suptitle(f"纯自旋：四电机转速与电流指令（绿色背景为匀速分析段，实际采样约{hz:.1f} Hz）")
    fig.tight_layout()
    fig.savefig(str(a.out_prefix) + "_全段.png", dpi=150)
    fig2, axs2 = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    tt = [(r["uptime_ms"] - a.steady_from_ms) / 1000 for r in steady]
    lim = max(
        50,
        max(abs(r["rotor_rpm"][i] - r["target_rotor_rpm"][i]) for r in steady for i in range(4))
        * 1.15,
    )
    for i, ax in enumerate(axs2):
        ax.plot(
            tt,
            [r["rotor_rpm"][i] - r["target_rotor_rpm"][i] for r in steady],
            color="0.6",
            label="原始误差",
        )
        ax.plot(
            tt,
            [r["filtered_rotor_rpm"][i] - r["target_rotor_rpm"][i] for r in steady],
            color=f"C{i}",
            label="滤波误差",
        )
        ax.axhline(0, color="black", ls="--")
        ax.set_ylim(-lim, lim)
        ax.set_title(f"{names[i]}：原始误差标准差 {metrics[i]['raw_error_sd']:.1f} rpm")
        ax.set_ylabel("实际−目标 / rpm")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, ncol=2)
    axs2[-1].set_xlabel("匀速段时间 / 秒")
    fig2.suptitle("匀速段放大：这里只能看低频误差，不能排除高频纹波")
    fig2.tight_layout()
    fig2.savefig(str(a.out_prefix) + "_匀速误差.png", dpi=150)
    result = dict(
        source=str(a.capture),
        effective_sample_hz=hz,
        metrics=metrics,
        note="电流为控制指令。该采样频率不足以分析高频振荡；编码器不能独立测量地面滑移。",
    )
    Path(str(a.out_prefix) + "_统计.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
