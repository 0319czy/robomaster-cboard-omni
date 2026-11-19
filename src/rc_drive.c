#include "rc_drive.h"
#include <math.h>
#include <string.h>

#define DEADBAND 150
#define SPAN 700
static int32_t Axis(uint16_t value, int32_t center)
{
    int32_t d = (int32_t)value - center;
    if (d > DEADBAND)
        d -= DEADBAND;
    else if (d < -DEADBAND)
        d += DEADBAND;
    else
        return 0;
    if (d > SPAN)
        d = SPAN;
    if (d < -SPAN)
        d = -SPAN;
    return d;
}

int RcDrive_Map(const uint16_t channels[16], uint8_t flags, RcDriveCommand *cmd)
{
    memset(cmd, 0, sizeof(*cmd));
    /* bit2 丢帧、bit3 失控；即使接收机继续发帧，也不能沿用其保持的油门。 */
    if (flags & 0x0cU)
        return 0;
    const int indices[4] = {0, 1, 3, 2};
    for (int i = 0; i < 4; i++)
        if (channels[indices[i]] < 100U || channels[indices[i]] > 1900U)
            return 0;
    cmd->vx_mm_s = -Axis(channels[1], 820) * RC_TRANSLATION_MAX_MM_S / SPAN;
    cmd->vy_mm_s = Axis(channels[3], 1024) * RC_TRANSLATION_MAX_MM_S / SPAN;
    /* 实测上档=460=逆时针，统一 omega 正值；下档=1506=顺时针。 */
    cmd->spin_mrad_s =
        channels[2] < 800U ? RC_SB_SPIN_MRAD_S : (channels[2] > 1250U ? -RC_SB_SPIN_MRAD_S : 0);
    /* 已有通道标定：右推 CH1 下降，因此负值对应顺时针（右转）。
     * SB 为约 ±12.566 rad/s、右摇杆最大 ±1 rad/s，合成值受独立上限约束。 */
    cmd->turn_mrad_s = Axis(channels[0], 1024) * RC_RIGHT_TURN_MRAD_S / SPAN;
    cmd->omega_mrad_s = cmd->spin_mrad_s + cmd->turn_mrad_s;
    return 1;
}

int RcDrive_Receive(RcDriveReceiver *rx, const uint16_t channels[16], uint8_t flags,
                    uint32_t frame_ms, uint32_t now, int envelope_valid, RcDriveCommand *cmd)
{
    RcDriveCommand next;
    memset(cmd, 0, sizeof(*cmd));
    if (flags & 0x08U)
    {
        rx->valid = 0;
        return 0;
    }
    int32_t age = (int32_t)(now - frame_ms);
    if (envelope_valid && age >= 0 && age <= 200 && RcDrive_Map(channels, flags, &next))
    {
        rx->last = next;
        rx->last_ms = frame_ms;
        rx->valid = 1;
    }
    if (!rx->valid || (uint32_t)(now - rx->last_ms) > 200U)
    {
        rx->valid = 0;
        return 0;
    }
    *cmd = rx->last;
    return 1;
}

void RcDrive_SpinStep(RcDriveSpinRamp *ramp, const RcDriveCommand *requested, uint32_t dt_ms,
                      int enabled, RcDriveCommand *applied)
{
    *applied = *requested;
    if (!enabled || dt_ms == 0U || dt_ms > 50U)
    {
        ramp->spin_mrad_s = 0.0f;
    }
    else
    {
        float target = (float)requested->spin_mrad_s;
        float current = ramp->spin_mrad_s;
        int reversing = target * current < 0.0f;
        /* 高速反向不能从 +12566 直接跳到 -12566：先以减速斜率到零。 */
        if (reversing)
            target = 0.0f;
        float rate = (reversing || fabsf(target) < fabsf(current)) ? RC_SPIN_DECEL_MRAD_S2
                                                                   : RC_SPIN_ACCEL_MRAD_S2;
        float step = rate * (float)dt_ms / 1000.0f;
        float delta = target - current;
        if (delta > step)
            delta = step;
        if (delta < -step)
            delta = -step;
        ramp->spin_mrad_s = current + delta;
    }
    applied->spin_mrad_s = (int32_t)ramp->spin_mrad_s;
    applied->omega_mrad_s = applied->spin_mrad_s + applied->turn_mrad_s;
    if (applied->omega_mrad_s > RC_TOTAL_OMEGA_MRAD_S)
        applied->omega_mrad_s = RC_TOTAL_OMEGA_MRAD_S;
    if (applied->omega_mrad_s < -RC_TOTAL_OMEGA_MRAD_S)
        applied->omega_mrad_s = -RC_TOTAL_OMEGA_MRAD_S;
}

int RcDrive_IsNeutral(const RcDriveCommand *cmd)
{
    /* 两路角速度相抵时也不是回中，不能据此在上电时解锁。 */
    return !cmd->vx_mm_s && !cmd->vy_mm_s && !cmd->omega_mrad_s && !cmd->spin_mrad_s &&
           !cmd->turn_mrad_s;
}

float RcDrive_FrameStep(RcDriveFrame *frame, const RcDriveCommand *cmd, float body_yaw,
                        uint32_t dt_ms, int enabled)
{
    if (enabled && dt_ms > 0U && dt_ms <= 50U)
    {
        /* 只积累操作者要求的转向量。即使 SB 同时自旋，也不会把它的旋转量
         * 混进参考航向。右摇杆回中后保留此角度，不吸附到正在自旋的车头。 */
        frame->heading_rad = remainderf(
            frame->heading_rad + (float)cmd->turn_mrad_s * (float)dt_ms / 1000000.0f, 6.283185307f);
    }
    return remainderf(body_yaw - frame->heading_rad, 6.283185307f);
}

float RcDrive_Omega(const RcDriveCommand *cmd, float yaw, float gyro_rate, int heading_hold,
                    float *heading, float *correction)
{
    *correction = 0.0f;
    /* 全回中：不保持旧姿态，不输出静止修正。主动自旋时目标随当前车头更新。 */
    if ((!cmd->vx_mm_s && !cmd->vy_mm_s) || cmd->omega_mrad_s || cmd->spin_mrad_s ||
        cmd->turn_mrad_s || !heading_hold)
    {
        *heading = yaw;
    }
    else
    {
        float err = remainderf(*heading - yaw, 6.283185307f);
        float corr = 3.0f * err - 2.0f * gyro_rate;
        if (corr > 1.5f)
            corr = 1.5f;
        if (corr < -1.5f)
            corr = -1.5f;
        *correction = corr;
    }
    float omega = (float)cmd->omega_mrad_s / 1000.0f + *correction;
    const float limit = RC_TOTAL_OMEGA_MRAD_S / 1000.0f;
    if (omega > limit)
        omega = limit;
    if (omega < -limit)
        omega = -limit;
    return omega;
}

float RcDrive_MixingYaw(const RcDriveCommand *cmd, float relative_yaw, float gyro_rate)
{
    /* 平移目标在车体系内随自旋持续旋转；速度闭环约35ms的等效滞后会造成
     * 地面方向偏转。使用实测角速度预测30ms后的坐标变换角，而不修改参考航向。
     * 无平移时不补偿；实际停转后预测量自然归零，避免留下固定航向偏置。
     * 预测角限幅保证异常角速度不会制造任意大的平移方向变化。 */
    if (!cmd || (!cmd->vx_mm_s && !cmd->vy_mm_s) || !isfinite(relative_yaw) || !isfinite(gyro_rate))
        return relative_yaw;
    float lead = gyro_rate * (float)RC_TRANSLATION_LEAD_MS / 1000.0f;
    if (lead > RC_TRANSLATION_LEAD_MAX_RAD)
        lead = RC_TRANSLATION_LEAD_MAX_RAD;
    if (lead < -RC_TRANSLATION_LEAD_MAX_RAD)
        lead = -RC_TRANSLATION_LEAD_MAX_RAD;
    return remainderf(relative_yaw + lead, 6.283185307f);
}
