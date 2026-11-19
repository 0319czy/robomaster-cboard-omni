"""从板内同步记录估计平移通道的等效动态滞后；不修改固件或控制参数。"""

import argparse, json, math
from pathlib import Path
import numpy as np

K = 0.7071067812 * 9.549296586 * 19.203208556 / 0.075


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", type=Path)
    a = p.parse_args()
    source = json.loads(a.capture.read_text())
    rows = source["samples"]
    t = np.asarray([r["uptime_ms"] for r in rows], float) / 1000
    t -= t[0]
    target = np.asarray([r["target_rotor_rpm"] for r in rows], float)
    actual = np.asarray([r["rotor_rpm"] for r in rows], float)
    cmd = np.asarray([[r["cmd"]["vx_mm_s"], r["cmd"]["vy_mm_s"]] for r in rows], float)
    omega = np.asarray([r["combined_turn_mrad_s"] for r in rows], float) / 1000

    def planar(v):
        return (
            np.column_stack(
                (-v[:, 0] - v[:, 1] + v[:, 2] + v[:, 3], -v[:, 0] + v[:, 1] + v[:, 2] - v[:, 3])
            )
            / (4 * K)
            * 1000
        )

    u = planar(target)
    y = planar(actual)
    good = (abs(omega) > 12) & (np.linalg.norm(cmd, axis=1) > 80)
    # 只用局部约200ms内杆量变化很小的样本，减轻油门推入/撤销瞬态影响。
    stable = good.copy()
    for i in np.flatnonzero(good):
        window = abs(t - t[i]) <= 0.1
        if np.max(np.linalg.norm(cmd[window] - cmd[i], axis=1)) > 20:
            stable[i] = False
    if stable.sum() < 30:
        raise RuntimeError("固定杆量全速自旋样本不足30个，无法可靠辨识")
    z = u[:, 0] + 1j * u[:, 1]
    v = y[:, 0] + 1j * y[:, 1]
    h = np.vdot(z[stable], v[stable]) / np.vdot(z[stable], z[stable])
    om = float(np.mean(omega[stable]))
    relative = np.asarray([r["body_relative_reference_mrad"] for r in rows], float) / 1000
    world = (y[:, 0] + 1j * y[:, 1]) * np.exp(1j * relative)
    desired = cmd[:, 0] + 1j * cmd[:, 1]
    requested_h = np.vdot(desired[stable], world[stable]) / np.vdot(
        desired[stable], desired[stable]
    )
    requested_error = float(np.sqrt(np.mean(abs(world[stable] - desired[stable]) ** 2)))
    fit = []
    for delay in np.arange(0, 0.15001, 0.005):
        shifted = np.column_stack([np.interp(t - delay, t, u[:, j]) for j in range(2)])
        gain = float(np.sum(shifted[stable] * y[stable]) / np.sum(shifted[stable] ** 2))
        error = float(np.sqrt(np.mean(np.sum((y[stable] - gain * shifted[stable]) ** 2, axis=1))))
        fit.append(dict(delay_ms=round(delay * 1000), gain=gain, residual_rms_mm_s=error))
    best = min(fit, key=lambda x: x["residual_rms_mm_s"])
    result = dict(
        source=str(a.capture),
        stable_samples=int(stable.sum()),
        selected_time_ranges_ms=[
            [int(rows[i]["uptime_ms"]), int(rows[j]["uptime_ms"])] for i, j in ranges(stable)
        ],
        mean_omega_rad_s=om,
        complex_response_gain=float(abs(h)),
        requested_response_gain=float(abs(requested_h)),
        requested_direction_error_deg=math.degrees(float(np.angle(requested_h))),
        requested_velocity_error_rms_mm_s=requested_error,
        command_rms_speed_mm_s=float(np.sqrt(np.mean(abs(desired[stable]) ** 2))),
        requested_velocity_normalized_rms_error=requested_error
        / float(np.sqrt(np.mean(abs(desired[stable]) ** 2))),
        phase_deg=math.degrees(float(np.angle(h))),
        phase_equivalent_delay_ms=1000 * float(np.angle(h)) / om,
        first_order_equivalent_tau_ms=1000 * math.tan(float(np.angle(h))) / om,
        simple_delay_model=best,
        delay_fit_curve=fit,
        assumptions=[
            "选择全速且局部杆量稳定的样本；轮速矩阵取当前固件参数。",
            "这是闭环平移通道的等效模型，含轮速控制、反馈时延、轮地耦合；不是单独传感器延迟。",
            "采样时刻同步，但轮速来自最近CAN反馈；真实地面滑移仍未测量。",
        ],
    )
    path = a.capture.with_suffix(".phase.json")
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


def ranges(mask):
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return zip(starts, ends)


if __name__ == "__main__":
    main()
