#include "chassis.h"
#include "uart_teleop.h"
#include <math.h>
#include <string.h>

/* 超速时在清零状态前保存现场；锁存故障期间不再覆盖。单位见 RecordOverspeed。 */
volatile uint32_t g_teleop_fault[32] = {0x54464c54U, 1U};
static void RecordOverspeed(const Chassis *c, uint32_t now, uint32_t dt, const MotorFeedback fb[4],
                            const float target[4], uint32_t channel, int hard)
{
    g_teleop_fault[2]++;
    g_teleop_fault[3] = now;
    g_teleop_fault[4] = dt;
    g_teleop_fault[5] = (uint32_t)(int32_t)(c->overspeed_trip * 1000.0f);
    g_teleop_fault[6] = channel;
    g_teleop_fault[7] = (uint32_t)hard;
    for (int i = 0; i < 4; i++)
    {
        g_teleop_fault[8 + i] = (uint32_t)(int32_t)(target[i] * 1000.0f);
        g_teleop_fault[12 + i] = (uint32_t)fb[i].rpm;
        g_teleop_fault[16 + i] = (uint32_t)(int32_t)(c->filtered_rpm[i] * 1000.0f);
        g_teleop_fault[20 + i] = (uint32_t)(int32_t)(c->integral[i] * 1000.0f);
        g_teleop_fault[24 + i] = c->overspeed_since[i];
    }
    g_teleop_fault[28] = c->state;
    g_teleop_fault[2]++;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
static int32_t abs32(int32_t v)
{
    return v < 0 ? -v : v;
}
static void Zero(Chassis *c)
{
    for (int i = 0; i < 4; i++)
        c->output[i] = 0;
}

void Chassis_Init(Chassis *c)
{
    memset(c, 0, sizeof(*c));
}

void Chassis_Stop(Chassis *c, uint32_t reason)
{
    if (c->state != FAULT)
    {
        c->fault = reason;
        c->fault_channel = 0;
    }
    c->state = FAULT;
    for (int i = 0; i < 4; i++)
    {
        c->target_rpm[i] = 0.0f;
        c->integral[i] = 0.0f;
    }
    Zero(c);
}

/* 只登记首次引发保护的通道；主循环是 Chassis 的唯一写入者。 */
static void WheelFault(Chassis *c, uint32_t reason, uint32_t channel)
{
    if (c->state == FAULT)
        return;
    Chassis_Stop(c, reason);
    c->fault_channel = channel;
}

static int AllStill(uint32_t now, const MotorFeedback fb[4])
{
    for (int i = 0; i < 4; i++)
        if (!fb[i].seen || (uint32_t)(now - fb[i].last_ms) > FEEDBACK_TIMEOUT_MS ||
            abs32(fb[i].rpm) > STOP_STILL_RPM)
            return 0;
    return 1;
}

void Chassis_Request(Chassis *c, const float wheel_rpm[4], uint32_t duration_ms, uint32_t token,
                     uint32_t now, const MotorFeedback fb[4])
{
    if (!token || token == c->ack_token)
        return; /* 重复令牌不重播 */
    c->ack_token = token;
    if (c->state != IDLE && c->state != COMPLETE)
    {
        WheelFault(c, F_NOT_READY, 0);
        return;
    }
    if (duration_ms < RUN_TIME_MIN_MS || duration_ms > RUN_TIME_MAX_MS)
    {
        WheelFault(c, F_BAD_CMD, 0);
        return;
    }
    if (now < 1000U || !AllStill(now, fb))
    {
        WheelFault(c, F_NOT_READY, 0);
        return;
    }
    float largest = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        if (!isfinite(wheel_rpm[i]))
        {
            WheelFault(c, F_BAD_CMD, (uint32_t)i + 1);
            return;
        }
        float magnitude = fabsf(wheel_rpm[i]);
        if (magnitude > largest)
            largest = magnitude;
    }
    /* 全零指令不启动：避免"看起来在跑其实没动"的假成功。 */
    if (largest <= 0.5f)
    {
        WheelFault(c, F_BAD_CMD, 0);
        return;
    }
    /* 增益调度：已验证参数(Kp=2.5, Ki=8)对应 576 转子 rpm 工况。低速时同样
     * 的 Ki 会让积分在斜坡内顶到限幅并造成 1.4~1.5 倍超调，因此按幅值缩放。 */
    c->gain_scale = largest / 576.0f;
    if (c->gain_scale > 1.0f)
        c->gain_scale = 1.0f;
    if (c->gain_scale < 0.15f)
        c->gain_scale = 0.15f;
    c->state = RUNNING;
    c->start_ms = now;
    c->duration_ms = duration_ms;
    c->fault = 0;
    c->fault_channel = 0;
    c->done_token = 0;
    c->peak_rpm = 0;
    c->peak_current = 0;
    c->still_since = 0;
    for (int i = 0; i < 4; i++)
    {
        c->target_rpm[i] = wheel_rpm[i];
        c->integral[i] = 0.0f;
        c->filtered_rpm[i] = (float)fb[i].rpm;
        c->last_angle[i] = (int32_t)fb[i].angle;
        c->ticks[i] = 0;
        c->reverse_since[i] = 0;
    }
    Zero(c);
}

void Chassis_Update(Chassis *c, uint32_t now, uint32_t dt_ms, const MotorFeedback fb[4],
                    uint32_t can_fault, uint32_t manual_stop)
{
    Zero(c);
    if (manual_stop)
    {
        Chassis_Stop(c, F_MANUAL);
        return;
    }
    if (can_fault)
    {
        Chassis_Stop(c, can_fault == 2U ? F_OTHER_CONTROLLER : F_CAN);
        return;
    }
    if (c->state != RUNNING && c->state != SETTLING)
        return;

    for (int i = 0; i < 4; i++)
    {
        if (!fb[i].seen || (uint32_t)(now - fb[i].last_ms) > FEEDBACK_TIMEOUT_MS)
        {
            WheelFault(c, F_FEEDBACK, (uint32_t)i + 1);
            return;
        }
        if (fb[i].temp >= 60U)
        {
            WheelFault(c, F_CAN, (uint32_t)i + 1);
            return;
        }
        if ((uint32_t)abs32(fb[i].rpm) > c->peak_rpm)
            c->peak_rpm = (uint32_t)abs32(fb[i].rpm);
    }
    /* 超速只在动作运行阶段判定：SETTLING 时目标已清零，滑行中的电气纹波
     * (M3508 实测 ±35~50 转子 rpm) 会被误判为超速。地板取 120 转子 rpm,
     * 仍远低于目标 1.5 倍的安全阈值。 */
    if (c->state == RUNNING)
    {
        for (int i = 0; i < 4; i++)
        {
            float trip = fabsf(c->target_rpm[i]) * 1.5f;
            if (trip < 120.0f)
                trip = 120.0f;
            if ((float)abs32(fb[i].rpm) > trip)
            {
                WheelFault(c, F_OVERSPEED, (uint32_t)i + 1);
                return;
            }
        }
    }

    if (c->state == SETTLING)
    {
        /* 结束后全零电流，需四台连续静止 500 ms 才判定完成。 */
        if (AllStill(now, fb))
        {
            if (!c->still_since)
                c->still_since = now;
        }
        else
            c->still_since = 0;
        if (c->still_since && (uint32_t)(now - c->still_since) >= 500U)
        {
            c->state = COMPLETE;
            c->done_token = c->ack_token;
        }
        else if ((uint32_t)(now - c->stop_ms) > 3000U)
        {
            WheelFault(c, F_STOP_TIMEOUT, 0);
        }
        return;
    }

    uint32_t elapsed = (uint32_t)(now - c->start_ms);
    /* 期限到就清零，不依赖主机继续通信。 */
    if (elapsed >= c->duration_ms)
    {
        c->state = SETTLING;
        c->stop_ms = now;
        c->still_since = 0;
        for (int i = 0; i < 4; i++)
        {
            c->target_rpm[i] = 0.0f;
            c->integral[i] = 0.0f;
        }
        return;
    }

    float ramp = 1.0f;
    if (elapsed < RAMP_MS)
        ramp = (float)elapsed / (float)RAMP_MS;
    else if (elapsed > c->duration_ms - RAMP_MS)
        ramp = (float)(c->duration_ms - elapsed) / (float)RAMP_MS;
    if (ramp < 0.0f)
        ramp = 0.0f;

    int32_t total_ticks = 0;
    for (int i = 0; i < 4; i++)
    {
        float target = c->target_rpm[i] * ramp;
        int32_t delta = (int32_t)fb[i].angle - c->last_angle[i];
        if (delta > 4096)
            delta -= 8192;
        else if (delta < -4096)
            delta += 8192;
        c->ticks[i] += delta;
        total_ticks += abs32(c->ticks[i]);
        c->last_angle[i] = (int32_t)fb[i].angle;

        /* 命令方向与实测方向相反且持续 100 ms，视为接线/符号异常。 */
        if (c->target_rpm[i] > 30.0f && fb[i].rpm < -30)
        {
            if (!c->reverse_since[i])
                c->reverse_since[i] = now;
        }
        else if (c->target_rpm[i] < -30.0f && fb[i].rpm > 30)
        {
            if (!c->reverse_since[i])
                c->reverse_since[i] = now;
        }
        else
        {
            c->reverse_since[i] = 0;
        }
        if (c->reverse_since[i] && (uint32_t)(now - c->reverse_since[i]) >= 100U)
        {
            WheelFault(c, F_UNEXPECTED_MOTION, (uint32_t)i + 1);
            return;
        }

        /* 与已验证的 M3508 闭环一致：8 ms 一阶滤波 + 连续 PI + 条件积分。 */
        float alpha = (float)dt_ms / (FILTER_MS + (float)dt_ms);
        c->filtered_rpm[i] += alpha * ((float)fb[i].rpm - c->filtered_rpm[i]);
        float error = target - c->filtered_rpm[i];
        /* 斜坡加速期间只做比例项，避免目标爬升造成的积分累积直接转成超调。 */
        float integrate = (elapsed >= RAMP_MS) ? 1.0f : 0.0f;
        float candidate = clampf(c->integral[i] + integrate * error * KI * c->gain_scale *
                                                      ((float)dt_ms / 1000.0f),
                                 -CURRENT_LIMIT, CURRENT_LIMIT);
        float demand = KP * error + candidate;
        if ((demand >= -CURRENT_LIMIT && demand <= CURRENT_LIMIT) ||
            (demand > CURRENT_LIMIT && error < 0.0f) || (demand < -CURRENT_LIMIT && error > 0.0f))
            c->integral[i] = candidate;
        c->output[i] = (int32_t)clampf(KP * error + c->integral[i], -CURRENT_LIMIT, CURRENT_LIMIT);
        if ((uint32_t)abs32(c->output[i]) > c->peak_current)
            c->peak_current = (uint32_t)abs32(c->output[i]);
    }

    /* 命令非零但四轮 4 秒内总行程不足，判定未起转。 */
    if (elapsed > 4000U && total_ticks < 100)
    {
        WheelFault(c, F_NO_MOTION, 0);
        return;
    }
}

/* ------------------------------------------------------------------ *
 * 串口遥控通道
 * 设计取舍：与令牌路径共用同一组 PI 参数与限幅，但独立实现，避免改动
 * 已被实测验证的 20 s 令牌流程。丢命令时不锁存故障，只做制止动后断电流。
 * ------------------------------------------------------------------ */
#define TELEOP_BRAKE_MS 1200U /* 达到零速目标后继续按反馈制动，避免高速回中提前滑行 */
#define TELEOP_STILL_RPM 20
#define SYNC_GAIN 0.0f /* 纯自旋同步增益；实测 0.4 会与 PI 形成慢振荡，先关掉 */

void Chassis_Teleop(Chassis *c, uint32_t now, uint32_t dt_ms, const float wheel_rpm[4], int active,
                    const MotorFeedback fb[4], uint32_t can_fault)
{
    Zero(c);
    if (c->state == FAULT)
        return; /* 任何遥控帧均不能覆盖已锁存的故障。 */
    if (can_fault)
    {
        Chassis_Stop(c, can_fault == 2U ? F_OTHER_CONTROLLER : F_CAN);
        return;
    }

    /* 任一反馈过期，四轮一起停车；不可只清过期的那一路。 */
    for (int i = 0; i < 4; i++)
        if (!fb[i].seen || (uint32_t)(now - fb[i].last_ms) > FEEDBACK_TIMEOUT_MS)
        {
            Chassis_Stop(c, F_FEEDBACK);
            c->fault_channel = (uint32_t)i + 1;
            return;
        }

    for (int i = 0; i < 4; i++)
    {
        if (fb[i].temp >= 60U)
        {
            WheelFault(c, F_CAN, (uint32_t)i + 1);
            return;
        }
        if (active && (!isfinite(wheel_rpm[i]) || fabsf(wheel_rpm[i]) > TELEOP_MAX_TARGET_RPM))
        {
            WheelFault(c, F_BAD_CMD, (uint32_t)i + 1);
            return;
        }
    }
    /* 保护与 PI 共用 8 ms 反馈滤波；硬限另看原始转速，不受滤波延迟影响。 */
    for (int i = 0; i < 4; i++)
    {
        float alpha = (float)dt_ms / (FILTER_MS + (float)dt_ms);
        c->filtered_rpm[i] += alpha * ((float)fb[i].rpm - c->filtered_rpm[i]);
    }
    /* 有效的全零命令也代表停车，不能继续积累 PI 并输出残余电流。 */
    float requested = 0.0f;
    for (int i = 0; i < 4; i++)
        requested = fmaxf(requested, fabsf(wheel_rpm[i]));
    if (requested <= 0.5f)
        active = 0;
    float target[4];
    float largest = 0.0f;
    if (active)
    {
        for (int i = 0; i < 4; i++)
        {
            target[i] = isfinite(wheel_rpm[i]) ? wheel_rpm[i] : 0.0f;
            if (fabsf(target[i]) > largest)
                largest = fabsf(target[i]);
        }
        c->state = TELEOP;
        c->brake_since = 0;
        c->gain_scale = largest / 576.0f;
        if (c->gain_scale > 1.0f)
            c->gain_scale = 1.0f;
        if (c->gain_scale < 0.15f)
            c->gain_scale = 0.15f;
        /* 混控时单轮目标会过零，用本次四轮最大目标构成共同速度包络，
         * 给仍在减速的轮子留出惯性空间；继续保留 2 倍阈值和硬上限。
         * 不可拿某一瞬间接近零的目标直接判定该轮真实超速。 */
        float trip = fmaxf(largest * 2.0f, 200.0f);
        if (trip > TELEOP_HARD_OVERSPEED_RPM)
            trip = TELEOP_HARD_OVERSPEED_RPM;
        /* 实测回油门 100 ms 内目标大幅下降，轮子尚有惯性。阈值上升立即生效，
         * 下降每秒最多收紧 2000 转子 rpm；持续不减速仍会被收紧后的阈值锁停。 */
        c->overspeed_trip = fmaxf(trip, c->overspeed_trip - 2000.0f * (float)dt_ms / 1000.0f);
        if (c->overspeed_trip > TELEOP_HARD_OVERSPEED_RPM)
            c->overspeed_trip = TELEOP_HARD_OVERSPEED_RPM;
        for (int i = 0; i < 4; i++)
        {
            int hard = (float)abs32(fb[i].rpm) > TELEOP_HARD_OVERSPEED_RPM;
            if (fabsf(c->filtered_rpm[i]) > c->overspeed_trip)
            {
                if (!c->overspeed_since[i])
                    c->overspeed_since[i] = now;
            }
            else
                c->overspeed_since[i] = 0;
            if (hard || (c->overspeed_since[i] && (uint32_t)(now - c->overspeed_since[i]) >= 100U))
            {
                RecordOverspeed(c, now, dt_ms, fb, target, (uint32_t)i + 1, hard);
                WheelFault(c, F_OVERSPEED, (uint32_t)i + 1);
                return;
            }
        }
    }
    else
    {
        /* 松手后先按零速闭环制动，直到四轮基本静止或超过制动窗口，再彻底断电流。 */
        for (int i = 0; i < 4; i++)
        {
            c->target_rpm[i] = 0.0f;
            c->overspeed_since[i] = 0;
        }
        if (c->state != TELEOP)
        {
            c->overspeed_trip = 0.0f;
            for (int i = 0; i < 4; i++)
            {
                c->integral[i] = 0.0f;
                c->filtered_rpm[i] = (float)fb[i].rpm;
            }
            return; /* 制动窗口结束后不因下一帧零命令而重新开始制动。 */
        }
        int moving = 0;
        for (int i = 0; i < 4; i++)
            if (abs32(fb[i].rpm) > TELEOP_STILL_RPM)
                moving = 1;
        if (!moving)
        {
            c->state = IDLE;
            c->brake_since = 0;
            c->overspeed_trip = 0.0f;
            for (int i = 0; i < 4; i++)
            {
                c->integral[i] = 0.0f;
                c->filtered_rpm[i] = (float)fb[i].rpm;
            }
            return;
        }
        if (!c->brake_since)
        {
            c->brake_since = now;
            for (int i = 0; i < 4; i++)
            {
                c->integral[i] = 0.0f;
                c->filtered_rpm[i] = (float)fb[i].rpm;
            }
        }
        if ((uint32_t)(now - c->brake_since) > TELEOP_BRAKE_MS)
        {
            /* 已持续制动但反馈仍显示运动，锁存超时，不能当作正常待命重启。 */
            Chassis_Stop(c, F_STOP_TIMEOUT);
            c->brake_since = 0;
            return;
        }
        for (int i = 0; i < 4; i++)
            target[i] = 0.0f;
        c->state = TELEOP;
        if (c->gain_scale <= 0.0f)
            c->gain_scale = 0.3f;
    }

    /* 纯自旋同步目前关闭，保持已验证的独立四轮 PI。 */
    {
        float tmin = c->filtered_rpm[0], tmax = c->filtered_rpm[0];
        for (int i = 1; i < 4; i++)
        {
            if (target[i] < tmin)
                tmin = target[i];
            if (target[i] > tmax)
                tmax = target[i];
        }
        /* 四轮目标几乎相同 = 纯自旋：把偏快/偏慢的轮子拉向共同速度，
         * 抵消电机/摩擦差异造成的绕心漂移（1.9% 的速度差≈4 mm/s 平移）。 */
        if ((tmax - tmin) < 30.0f && fabsf(tmax) > 100.0f)
        {
            float avg = 0.0f;
            for (int i = 0; i < 4; i++)
                avg += c->filtered_rpm[i];
            avg *= 0.25f;
            for (int i = 0; i < 4; i++)
                target[i] -= SYNC_GAIN * (c->filtered_rpm[i] - avg);
        }
    }
    for (int i = 0; i < 4; i++)
    {
        float error = target[i] - c->filtered_rpm[i];
        float candidate =
            clampf(c->integral[i] + error * KI * c->gain_scale * ((float)dt_ms / 1000.0f),
                   -TELEOP_CURRENT_LIMIT, TELEOP_CURRENT_LIMIT);
        float demand = KP * error + candidate;
        if ((demand >= -TELEOP_CURRENT_LIMIT && demand <= TELEOP_CURRENT_LIMIT) ||
            (demand > TELEOP_CURRENT_LIMIT && error < 0.0f) ||
            (demand < -TELEOP_CURRENT_LIMIT && error > 0.0f))
            c->integral[i] = candidate;
        c->output[i] = (int32_t)clampf(KP * error + c->integral[i], -TELEOP_CURRENT_LIMIT,
                                       TELEOP_CURRENT_LIMIT);
        c->target_rpm[i] = target[i];
        if ((uint32_t)abs32(c->output[i]) > c->peak_current)
            c->peak_current = (uint32_t)abs32(c->output[i]);
        if ((uint32_t)abs32(fb[i].rpm) > c->peak_rpm)
            c->peak_rpm = (uint32_t)abs32(fb[i].rpm);
    }
}
