"""离线分析自旋叠加平移；仅读取已有记录，不访问或驱动硬件。"""

import argparse, csv, json, math, re
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from plot_style import configure_plot_font

configure_plot_font()
K = 0.7071067812 * 9.549296586 * 19.203208556 / 0.075
L = 0.31
NAMES = ["ID1 左后", "ID2 左前", "ID3 右前", "ID4 右后"]


def rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else None


def arr(rows, key):
    return np.asarray([r[key] for r in rows], dtype=float)


def signed(x):
    return x if x < 0x80000000 else x - 0x100000000


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    p.add_argument("--out-prefix", type=Path, required=True)
    a = p.parse_args()
    original = json.loads(a.capture.read_text())
    rows = sorted(
        {r["uptime_ms"]: dict(r) for r in original["samples"]}.values(),
        key=lambda r: r["uptime_ms"],
    )
    # 原日志已经包含整组 IMU 字，离线补出 X/Y 轴；不改变原始 JSON。
    log = a.capture.with_suffix(".log")
    xyz = {}
    if log.exists():
        for line in log.read_text().splitlines():
            if line.startswith("DRIVE "):
                w = [int(x, 0) for x in line[6:].split(",")]
                if len(w) >= 96:
                    xyz[w[3]] = (signed(w[86]), signed(w[87]))
    for r in rows:
        if r["uptime_ms"] in xyz:
            r["gyro_x_mrad_s"], r["gyro_y_mrad_s"] = xyz[r["uptime_ms"]]
    first_active = next(
        r["uptime_ms"]
        for r in rows
        if max(map(abs, r["target_rotor_rpm"])) > 1 or max(map(abs, r["rotor_rpm"])) > 20
    )
    t0 = first_active - 2000
    for r in rows:
        r["time_s"] = (r["uptime_ms"] - t0) / 1000
    ts = arr(rows, "time_s")
    actual = arr(rows, "rotor_rpm")
    target = arr(rows, "target_rotor_rpm")
    filt = arr(rows, "filtered_rotor_rpm")
    current = arr(rows, "command_raw")
    cmd = np.asarray([[r["cmd"]["vx_mm_s"], r["cmd"]["vy_mm_s"]] for r in rows], float)
    omega = arr(rows, "combined_turn_mrad_s") / 1000
    separate_sb = all("sb_spin_mrad_s" in r for r in rows)
    sb = arr(rows, "sb_spin_mrad_s") / 1000 if separate_sb else omega.copy()
    gyro = (arr(rows, "gyro_z_mrad_s") - arr(rows, "gyro_bias_mrad_s")) / 1000
    yaw = arr(rows, "yaw_mrad") / 1000
    # 两圈每秒时，偶发300ms采样间隔的转角超过π；普通unwrap会误判整圈。
    # 仅在展开角度时用陀螺预估选择整圈数，原始回绕航向仍单独绘图/导出。
    continuous_yaw = [yaw[0]]
    for i in range(1, len(rows)):
        delta = (yaw[i] - yaw[i - 1] + math.pi) % (2 * math.pi) - math.pi
        predicted = 0.5 * (gyro[i] + gyro[i - 1]) * (ts[i] - ts[i - 1])
        delta += round((predicted - delta) / (2 * math.pi)) * 2 * math.pi
        continuous_yaw.append(continuous_yaw[-1] + delta)
    reference = arr(rows, "control_reference_mrad") / 1000
    relative = arr(rows, "body_relative_reference_mrad") / 1000

    # 严格复用固件混控矩阵的逆解，并转回当前虚拟参考系。
    # 这是编码器运动学估计，不是独立地面速度。
    def inverse(v):
        x = (-v[:, 0] - v[:, 1] + v[:, 2] + v[:, 3]) / (4 * K)
        y = (-v[:, 0] + v[:, 1] + v[:, 2] - v[:, 3]) / (4 * K)
        return np.column_stack(
            (
                np.cos(relative) * x - np.sin(relative) * y,
                np.sin(relative) * x + np.cos(relative) * y,
            )
        ) * 1000, -np.sum(v, axis=1) / (4 * K * L)

    estimated, wheel_omega = inverse(actual)
    expected, expected_omega = inverse(target)
    active = (np.max(abs(target), axis=1) > 1) | (np.max(abs(actual), axis=1) > 20)
    ix = np.flatnonzero(active)
    if len(ix) == 0:
        raise RuntimeError("没有运动数据")
    window = (ts >= ts[ix[0]] - 2) & (ts <= ts[ix[-1]] + 2)
    lo, hi = ts[window][0], ts[window][-1]
    dt = np.diff(ts)
    count = int(window.sum())
    effective = (count - 1) / (hi - lo)
    fullspin = abs(sb) > 12.0
    # 只比较全速自旋，剔除自旋爬升初段；平移输入边沿仍保留，便于看跟踪压力。
    steady = fullspin.copy()
    onset = None
    for i in range(len(rows)):
        if not fullspin[i] or (i and np.sign(sb[i]) != np.sign(sb[i - 1])):
            onset = None
        if fullspin[i] and onset is None:
            onset = rows[i]["uptime_ms"]
        if onset is None or rows[i]["uptime_ms"] - onset < 800:
            steady[i] = False
    moving = np.linalg.norm(cmd, axis=1) > 20
    result = dict(
        source=str(a.capture),
        sample_count=len(rows),
        motion_window_start_ms=rows[np.flatnonzero(window)[0]]["uptime_ms"],
        motion_window_end_ms=rows[np.flatnonzero(window)[-1]]["uptime_ms"],
        effective_motion_sample_hz=effective,
        median_interval_s=float(np.median(dt)),
        largest_gap_s=float(max(dt)),
        faults=sorted(set(r["fault"] for r in rows)),
        tx_fail_delta=rows[-1]["tx_fail"] - rows[0]["tx_fail"],
        reference_change_mrad=int(np.ptp(arr(rows, "control_reference_mrad"))),
        right_turn_max_mrad_s=(
            int(max(abs(arr(rows, "right_turn_mrad_s"))))
            if all("right_turn_mrad_s" in r for r in rows)
            else None
        ),
        spin_stage_signal="SB输入" if separate_sb else "合成旋转指令（未单独记录SB/右杆）",
        gyro_bias_mrad_s=float(np.mean(arr(rows, "gyro_bias_mrad_s"))),
        sections=[],
        stop_events=[],
        final=rows[-1],
        limitations=[
            f'名义上限{original.get("sample_rate_limit_hz",10):g}Hz，实际频率见effective_motion_sample_hz；不分析超过采样带宽的振荡。',
            "总览电流为控制指令；独立反馈图含C620上报转矩电流，均非直接相电流测量。",
            "平移估计由轮速、近似几何与陀螺航向换算，不能证明地面轨迹或排除轮胎滑移。",
            "各传感器仍有自身物理采样时刻；高速记录为同周期快照，旧记录可能有数毫秒额外读取差。",
            "平移段包含多次杆量变化，误差含正常动态滞后；不是恒速精度验收。",
        ],
    )
    for direction, sgn in [("逆时针", 1), ("顺时针", -1)]:
        for mode, mask in [("纯自旋", ~moving), ("自旋叠加平移", moving)]:
            sel = steady & (sb * sgn > 0) & mask
            n = int(sel.sum())
            if not n:
                continue
            er = actual[sel] - target[sel]
            ve = estimated[sel] - expected[sel]
            stats = dict(
                direction=direction,
                mode=mode,
                n=n,
                start_ms=rows[np.flatnonzero(sel)[0]]["uptime_ms"],
                end_ms=rows[np.flatnonzero(sel)[-1]]["uptime_ms"],
                gyro_mean_rad_s=float(np.mean(gyro[sel])),
                gyro_sd_rad_s=float(np.std(gyro[sel])),
                gyro_mean_rev_s=float(np.mean(gyro[sel]) / (2 * math.pi)),
                gyro_tracking_rms_rad_s=rms(gyro[sel] - omega[sel]),
                wheel_omega_mean_rad_s=float(np.mean(wheel_omega[sel])),
                wheel_gyro_difference_rms_rad_s=rms(wheel_omega[sel] - gyro[sel]),
                command_translation_peak_mm_s=float(max(np.linalg.norm(cmd[sel], axis=1))),
                encoder_reference_velocity_error_rms_mm_s=float(
                    np.sqrt(np.mean(np.sum(ve * ve, axis=1)))
                ),
                encoder_vs_requested_velocity_rms_mm_s=float(
                    np.sqrt(np.mean(np.sum((estimated[sel] - cmd[sel]) ** 2, axis=1)))
                ),
                target_rotor_peak_rpm=float(np.max(abs(target[sel]))),
                actual_rotor_peak_rpm=float(np.max(abs(actual[sel]))),
                motors=[],
            )
            for i in range(4):
                stats["motors"].append(
                    dict(
                        id=i + 1,
                        rpm_error_mean=float(np.mean(er[:, i])),
                        rpm_error_rms=rms(er[:, i]),
                        rpm_error_peak=float(max(abs(er[:, i]))),
                        current_mean_raw=float(np.mean(current[sel, i])),
                        mean_abs_current_raw=float(np.mean(abs(current[sel, i]))),
                        current_peak_raw=float(max(abs(current[sel, i]))),
                        saturation_samples=int(np.sum(abs(current[sel, i]) >= 2457)),
                        saturation_percent=float(100 * np.mean(abs(current[sel, i]) >= 2457)),
                    )
                )
            fit = sel & (abs(cmd[:, 0]) > 100) & (abs(cmd[:, 1]) <= 20)
            if np.any(fit):
                x = cmd[fit, 0]
                den = float(x @ x)
                forward = float(x @ estimated[fit, 0] / den)
                lateral = float(x @ estimated[fit, 1] / den)
                stats["encoder_apparent_translation_fit"] = dict(
                    n=int(fit.sum()),
                    forward_gain=forward,
                    lateral_gain=lateral,
                    apparent_direction_deg=math.degrees(math.atan2(lateral, forward)),
                    note="包含动态输入的轮速逆解拟合，不能作为地面轨迹方向测量或精确滞后辨识。",
                )
            if all("gyro_x_mrad_s" in r and "gyro_y_mrad_s" in r for r in rows):
                stats["gyro_xy_rms_rad_s"] = [
                    rms(arr(rows, k)[sel] / 1000) for k in ("gyro_x_mrad_s", "gyro_y_mrad_s")
                ]
            result["sections"].append(stats)
    # 从原始 SB 档回中计时，避免把斜坡降到零的时刻误当操作起点。
    rawsb = np.asarray([r["rc_channels"][2] if "rc_channels" in r else 1024 for r in rows])
    neutral = (rawsb > 800) & (rawsb < 1200)
    for i in range(1, len(rows)):
        if neutral[i] and not neutral[i - 1] and abs(sb[i - 1]) > 1:
            end = None
            for j in range(i, len(rows)):
                if not neutral[j]:
                    break
                if (
                    max(abs(actual[j])) <= 20
                    and max(abs(current[j])) == 0
                    and rows[j]["state"] == 0
                ):
                    end = j
                    break
            if end is not None:
                result["stop_events"].append(
                    dict(
                        sb_mid_ms=rows[i]["uptime_ms"],
                        stopped_ms=rows[end]["uptime_ms"],
                        sampled_delay_ms=rows[end]["uptime_ms"] - rows[i]["uptime_ms"],
                        translation_was_active=bool(np.linalg.norm(cmd[i]) > 20),
                    )
                )

    def style(axes):
        for ax in np.asarray(axes).flat:
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8, loc="upper right", ncol=3)
            ax.set_xlim(lo, hi)
            # 淡蓝色表示全速自旋同时输入平移。
            ax.fill_between(
                ts,
                0,
                1,
                where=fullspin & moving,
                color="C0",
                alpha=0.06,
                transform=ax.get_xaxis_transform(),
            )
        np.asarray(axes).flat[-1].set_xlabel("相对运动窗口开始 / 秒")

    fig, ax = plt.subplots(5, 1, figsize=(15, 16), sharex=True)
    for i in range(2):
        ax[0].plot(ts, cmd[:, i], label=["前进指令", "左移指令"][i])
    ax[0].set_ylabel("平移指令 / mm/s")
    ax[1].plot(ts, omega, "k--", label="合成角速度指令")
    ax[1].plot(ts, gyro, label="陀螺Z扣零偏")
    ax[1].plot(ts, wheel_omega, alpha=0.6, label="四轮逆解角速度")
    ax[1].set_ylabel("角速度 / rad/s")
    ax[2].plot(ts, np.asarray(continuous_yaw) * 180 / math.pi, label="固件航向展开（陀螺辅助判圈）")
    ax[2].plot(ts, reference * 180 / math.pi, label="平移参考航向")
    ax[2].set_ylabel("航向 / 度")
    for i in range(2):
        ax[3].plot(ts, cmd[:, i], "--", color=f"C{i}", label=["请求前向", "请求左向"][i])
        ax[3].plot(
            ts,
            expected[:, i],
            ":",
            color=f"C{i}",
            alpha=0.65,
            label=["下发目标逆解前向", "下发目标逆解左向"][i],
        )
        ax[3].plot(
            ts, estimated[:, i], color=f"C{i}", label=["轮速反馈估计前向", "轮速反馈估计左向"][i]
        )
    ax[3].set_ylabel("参考系估计速度 / mm/s")
    for i in range(4):
        ax[4].plot(ts, current[:, i], label=NAMES[i])
    ax[4].axhline(2457, color="r", ls=":")
    ax[4].axhline(-2457, color="r", ls=":")
    ax[4].set_ylabel("电流指令 / raw")
    style(ax)
    fig.suptitle(f"自旋中移动：指令、陀螺与运动学估计（实际约{effective:.1f}Hz；浅蓝为叠加平移）")
    fig.tight_layout()
    fig.savefig(str(a.out_prefix) + "_总览.png", dpi=145)
    plt.close(fig)
    fig, axes = plt.subplots(4, 2, figsize=(16, 13), sharex=True)
    for i in range(4):
        axes[i, 0].plot(ts, target[:, i], "k--", label="目标")
        axes[i, 0].plot(ts, actual[:, i], color=f"C{i}", label="原始反馈")
        axes[i, 0].plot(ts, filt[:, i], color="gray", alpha=0.6, label="8ms滤波反馈")
        axes[i, 0].set_title(NAMES[i])
        axes[i, 0].set_ylabel("转子 rpm")
        axes[i, 1].plot(ts, actual[:, i] - target[:, i], color=f"C{i}", label="实际−目标")
        axes[i, 1].axhline(0, color="k", lw=0.5)
        axes[i, 1].set_ylabel("跟踪误差 / rpm")
    style(axes)
    fig.suptitle("四个电机独立波形：自旋叠加平移时，目标轮速本来就随航向变化")
    fig.tight_layout()
    fig.savefig(str(a.out_prefix) + "_四轮.png", dpi=145)
    plt.close(fig)
    fig, ax = plt.subplots(4, 1, figsize=(15, 13), sharex=True)
    for key, label in [("gyro_x_mrad_s", "陀螺X"), ("gyro_y_mrad_s", "陀螺Y")]:
        if all(key in r for r in rows):
            ax[0].plot(ts, arr(rows, key) / 1000, label=label)
    ax[0].set_ylabel("X/Y角速度 / rad/s")
    ax[1].plot(ts, omega, "k--", label="角速度指令")
    ax[1].plot(ts, gyro, label="陀螺Z扣零偏")
    ax[1].set_ylabel("Z角速度 / rad/s")
    ax[2].plot(ts, gyro - omega, label="陀螺−角速度指令")
    ax[2].plot(ts, wheel_omega - gyro, label="四轮逆解−陀螺")
    ax[2].set_ylabel("角速度差 / rad/s")
    ax[3].plot(ts, arr(rows, "yaw_mrad") / 1000, label="车体航向（±π回绕）")
    ax[3].plot(ts, reference, label="平移参考航向")
    ax[3].plot(ts, relative, label="车体相对参考航向")
    ax[3].set_ylabel("航向 / rad")
    style(ax)
    fig.suptitle("陀螺仪与参考航向：航向在±π处跳变是角度回绕")
    fig.tight_layout()
    fig.savefig(str(a.out_prefix) + "_陀螺.png", dpi=145)
    plt.close(fig)
    if all("feedback_current_raw" in r for r in rows):
        feedback = arr(rows, "feedback_current_raw")
        queued = arr(rows, "queued_current_raw")
        fig, axs = plt.subplots(4, 1, figsize=(15, 12), sharex=True)
        for i, ax in enumerate(axs):
            ax.plot(ts, current[:, i], "--", label="本轮PI输出")
            ax.plot(ts, queued[:, i], label="最近提交CAN的指令")
            ax.plot(ts, feedback[:, i], label="C620反馈转矩电流")
            ax.set_title(NAMES[i])
            ax.set_ylabel("电流 / raw")
        style(axs)
        fig.suptitle("电流指令与C620反馈（反馈不是直接相电流测量）")
        fig.tight_layout()
        fig.savefig(str(a.out_prefix) + "_电流反馈.png", dpi=145)
        plt.close(fig)
        result["timing"] = dict(
            max_control_dt_ms=int(max(arr(rows, "step_dt_ms"))),
            max_loop_work_ms=int(max(arr(rows, "loop_work_ms"))),
            max_imu_age_ms=int(max(arr(rows, "imu_age_ms"))),
            max_feedback_age_ms=int(np.max(arr(rows, "feedback_age_ms"))),
        )
    # 导出全部已采样数据与推导值，保留单位及原始时间戳。
    flat = []
    for i, r in enumerate(rows):
        d = dict(
            time_s=r["time_s"],
            uptime_ms=r["uptime_ms"],
            state=r["state"],
            fault=r["fault"],
            cmd_vx_mm_s=cmd[i, 0],
            cmd_vy_mm_s=cmd[i, 1],
            cmd_omega_rad_s=omega[i],
            gyro_z_corrected_rad_s=gyro[i],
            yaw_rad=yaw[i],
            reference_rad=reference[i],
            encoder_reference_vx_mm_s=estimated[i, 0],
            encoder_reference_vy_mm_s=estimated[i, 1],
            wheel_inverse_omega_rad_s=wheel_omega[i],
        )
        for key in ("gyro_x_mrad_s", "gyro_y_mrad_s", "gyro_z_mrad_s", "gyro_bias_mrad_s"):
            d[key] = r.get(key)
        for j in range(4):
            d[f"id{j+1}_target_rotor_rpm"] = target[i, j]
            d[f"id{j+1}_actual_rotor_rpm"] = actual[i, j]
            d[f"id{j+1}_current_command_raw"] = current[i, j]
        for key in ("step_dt_ms", "imu_age_ms", "loop_work_ms", "sequence"):
            if key in r:
                d[key] = r[key]
        if "feedback_current_raw" in r:
            for j in range(4):
                d[f"id{j+1}_feedback_current_raw"] = r["feedback_current_raw"][j]
                d[f"id{j+1}_queued_current_raw"] = r["queued_current_raw"][j]
                d[f"id{j+1}_feedback_age_ms"] = r["feedback_age_ms"][j]
        flat.append(d)
    with Path(str(a.out_prefix) + "_数据.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    Path(str(a.out_prefix) + "_统计.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
