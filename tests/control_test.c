/* 主机端单元测试：混控矩阵 + 状态机/保护逻辑。gcc 编译，不依赖 HAL。 */
#include "chassis.h"
#include "omni_x45.h"
#include "rc_drive.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const OmniX45Config OMNI = {
    .half_length_m = 0.150f,
    .half_width_m = 0.160f,
    .wheel_radius_m = 0.075f,
    .motor_turns_per_wheel_turn = 19.203208556f,
    .max_motor_rpm = 576.0f,
    .sign = {1, 1, 1, 1},
};

static MotorFeedback fb[4];
static uint32_t t;

static void reset_fb(uint32_t now)
{
    for (int i = 0; i < 4; i++)
    {
        fb[i].rpm = 0;
        fb[i].angle = 0;
        fb[i].last_ms = now;
        fb[i].seen = 1;
        fb[i].temp = 25;
    }
}

/* 理想跟随：反馈转速直接跟到（应用斜坡后的）目标，角度按转速累计。 */
static void ideal_step(Chassis *c)
{
    for (int i = 0; i < 4; i++)
    {
        uint32_t ramp_ok = (c->state == RUNNING);
        fb[i].rpm = ramp_ok ? (int32_t)c->target_rpm[i] : 0;
        fb[i].last_ms = t;
        /* 角度：转子 rpm → 每 2 ms 计数, 8192/圈 */
        fb[i].angle += (uint32_t)(fb[i].rpm * 8192 / 60 / 500);
        fb[i].angle %= 8192;
    }
}

static int near(float a, float b)
{
    return fabsf(a - b) < 0.05f * fabsf(b) + 1.0f;
}

static void test_mixing(void)
{
    float rpm[4];
    /* 前进 0.1 m/s：1/2 负、3/4 正，幅值相同 */
    assert(omni_x45_rpm(&OMNI, 0.1f, 0.0f, 0.0f, 0.0f, rpm));
    assert(near(rpm[0], -0.1f * WHEEL_RPM_PER_MPS));
    assert(near(rpm[1], rpm[0]) && near(rpm[2], -rpm[0]) && near(rpm[3], -rpm[0]));
    printf("混控 前进: %.1f %.1f %.1f %.1f\n", rpm[0], rpm[1], rpm[2], rpm[3]);

    /* 左移 0.1 m/s：1/4 负、2/3 正 */
    assert(omni_x45_rpm(&OMNI, 0.0f, 0.1f, 0.0f, 0.0f, rpm));
    assert(near(rpm[0], -0.1f * WHEEL_RPM_PER_MPS) && near(rpm[1], -rpm[0]));
    assert(near(rpm[2], -rpm[0]) && near(rpm[3], rpm[0]));
    printf("混控 左移: %.1f %.1f %.1f %.1f\n", rpm[0], rpm[1], rpm[2], rpm[3]);

    /* 逆时针自旋：实车四轮同为负号 */
    assert(omni_x45_rpm(&OMNI, 0.0f, 0.0f, 0.5f, 0.0f, rpm));
    assert(near(rpm[0], rpm[1]) && near(rpm[1], rpm[2]) && near(rpm[2], rpm[3]) && rpm[0] < 0);
    float expect_spin = -(OMNI.half_length_m + OMNI.half_width_m) * 0.5f * WHEEL_RPM_PER_MPS;
    assert(near(rpm[0], expect_spin));
    printf("混控 逆时针自旋: %.1f %.1f %.1f %.1f (期望 %.1f)\n", rpm[0], rpm[1], rpm[2], rpm[3],
           expect_spin);

    /* 顺时针自旋 = 逆时针取反 */
    assert(omni_x45_rpm(&OMNI, 0.0f, 0.0f, -0.5f, 0.0f, rpm));
    assert(near(rpm[0], -expect_spin));

    /* 限速按比例保留方向：超大字命令应等幅缩放到 max_motor_rpm */
    assert(omni_x45_rpm(&OMNI, 5.0f, 0.0f, 1.0f, 0.0f, rpm));
    float largest = 0;
    for (int i = 0; i < 4; i++)
        largest = fmaxf(largest, fabsf(rpm[i]));
    assert(near(largest, OMNI.max_motor_rpm));
    printf("限速保持比例: 最大 %.1f rpm\n", largest);

    /* 非法参数返回 false 并清零 */
    rpm[0] = 1;
    assert(!omni_x45_rpm(&OMNI, NAN, 0, 0, 0, rpm) && rpm[0] == 0);
    OmniX45Config bad = OMNI;
    bad.sign[2] = 0;
    assert(!omni_x45_rpm(&bad, 0.1f, 0, 0, 0, rpm));
}

static void test_reject_not_ready(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    fb[2].rpm = 50; /* 有电机在转 */
    float target[4] = {100, 100, -100, -100};
    Chassis_Request(&c, target, 5000, 11, t, fb);
    assert(c.state == FAULT && c.fault == F_NOT_READY);
    printf("拒绝：电机未停稳 ok\n");

    Chassis_Init(&c);
    reset_fb(t);
    fb[1].last_ms = t - 100; /* 反馈过期 */
    Chassis_Request(&c, target, 5000, 12, t, fb);
    assert(c.state == FAULT && c.fault == F_NOT_READY);
    printf("拒绝：反馈过期 ok\n");
}

static void test_reject_bad_cmd(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float zero[4] = {0, 0, 0, 0};
    Chassis_Request(&c, zero, 5000, 21, t, fb);
    assert(c.state == FAULT && c.fault == F_BAD_CMD);
    printf("拒绝：全零指令 ok\n");

    Chassis_Init(&c);
    reset_fb(t);
    float target[4] = {100, 100, -100, -100};
    Chassis_Request(&c, target, RUN_TIME_MAX_MS + 1, 22, t, fb);
    assert(c.state == FAULT && c.fault == F_BAD_CMD);
    printf("拒绝：时长越界 ok\n");

    Chassis_Init(&c);
    reset_fb(t);
    float nan_target[4] = {NAN, 100, -100, -100};
    Chassis_Request(&c, nan_target, 5000, 23, t, fb);
    assert(c.state == FAULT && c.fault == F_BAD_CMD);
    printf("拒绝：目标非有限值 ok\n");
}

static void test_complete_run(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float wheel[4];
    assert(omni_x45_rpm(&OMNI, 0.1f, 0, 0, 0, wheel));
    Chassis_Request(&c, wheel, 4000, 31, t, fb);
    assert(c.state == RUNNING);
    uint32_t guard = 0;
    while (c.state != COMPLETE && c.state != FAULT && guard++ < 5000)
    {
        t += 2;
        ideal_step(&c);
        Chassis_Update(&c, t, 2, fb, 0, 0);
    }
    assert(c.state == COMPLETE);
    assert(c.done_token == 31);
    for (int i = 0; i < 4; i++)
        assert(c.output[i] == 0);
    printf("完整流程 4s 动作：%u ms 后 COMPLETE，输出已清零 ok\n", t - 5000);
}

static void test_overspeed(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float target[4] = {100, 100, -100, -100};
    Chassis_Request(&c, target, 5000, 41, t, fb);
    t += 2;
    fb[1].rpm = 300;
    fb[1].last_ms = t; /* 目标 100 的 3 倍 */
    Chassis_Update(&c, t, 2, fb, 0, 0);
    assert(c.state == FAULT && c.fault == F_OVERSPEED && c.fault_channel == 2);
    printf("保护：超速锁存 通道%u ok\n", c.fault_channel);
}

static void test_no_motion(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float target[4] = {100, 100, -100, -100};
    Chassis_Request(&c, target, 8000, 51, t, fb);
    uint32_t guard = 0;
    while (c.state == RUNNING && guard++ < 5000)
    { /* 反馈恒为 0 */
        t += 2;
        for (int i = 0; i < 4; i++)
        {
            fb[i].rpm = 0;
            fb[i].last_ms = t;
        }
        Chassis_Update(&c, t, 2, fb, 0, 0);
    }
    assert(c.state == FAULT && c.fault == F_NO_MOTION);
    printf("保护：未起转 %u ms 锁定 ok\n", t - 5000);
}

static void test_reverse_motion(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float target[4] = {200, 200, -200, -200};
    Chassis_Request(&c, target, 5000, 61, t, fb);
    for (int step = 0; step < 100 && c.state == RUNNING; step++)
    {
        t += 2;
        fb[0].rpm = -150;
        fb[0].last_ms = t; /* 命令正向，实际反向 */
        for (int i = 1; i < 4; i++)
        {
            fb[i].rpm = 0;
            fb[i].last_ms = t;
        }
        Chassis_Update(&c, t, 2, fb, 0, 0);
    }
    assert(c.state == FAULT && c.fault == F_UNEXPECTED_MOTION && c.fault_channel == 1);
    printf("保护：反向运动锁存 ok\n");
}

static void test_manual_stop_and_replay(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float target[4] = {100, 100, -100, -100};
    Chassis_Request(&c, target, 5000, 71, t, fb);
    assert(c.state == RUNNING);
    Chassis_Update(&c, t, 2, fb, 0, 1); /* 主机手动停止 */
    assert(c.state == FAULT && c.fault == F_MANUAL);
    printf("保护：手动停止 ok\n");

    Chassis_Init(&c);
    reset_fb(t);
    Chassis_Request(&c, target, 5000, 72, t, fb);
    assert(c.state == RUNNING);
    uint32_t ack = c.ack_token;
    Chassis_Request(&c, target, 5000, 72, t, fb); /* 重复令牌不得重播 */
    assert(c.state == RUNNING && c.ack_token == ack);
    printf("拒绝：重复令牌不重播 ok\n");
}

static void test_teleop(void)
{
    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float wheel[4];
    assert(omni_x45_rpm(&OMNI, 0.3f, 0, 0, 0, wheel));

    /* 有命令：进入 TELEOP，输出方向与目标的符号一致 */
    Chassis_Teleop(&c, t, 2, wheel, 1, fb, 0);
    assert(c.state == TELEOP);
    assert(c.output[0] < 0 && c.output[2] > 0);
    printf("遥控 有命令: state=TELEOP 输出 %d %d %d %d\n", c.output[0], c.output[1], c.output[2],
           c.output[3]);

    /* 失联：目标为 0，电机仍在转时进入制动，只输出反力矩 */
    for (int i = 0; i < 4; i++)
    {
        fb[i].rpm = (int32_t)(wheel[i] * 0.8f);
        fb[i].last_ms = t;
    }
    Chassis_Teleop(&c, t, 2, wheel, 0, fb, 0);
    assert(c.state == TELEOP);
    assert(c.output[0] > 0); /* 目标 0、实际为负 → 正力矩制动 */
    printf("遥控 失联制动: 输出 %d %d %d %d\n", c.output[0], c.output[1], c.output[2], c.output[3]);

    /* 四轮停住后断电流、回到 IDLE */
    for (int i = 0; i < 4; i++)
    {
        fb[i].rpm = 0;
        fb[i].last_ms = t;
    }
    Chassis_Teleop(&c, t, 2, wheel, 0, fb, 0);
    assert(c.state == IDLE);
    for (int i = 0; i < 4; i++)
        assert(c.output[i] == 0);
    printf("遥控 失联且静止: 电流归零 ok\n");

    /* 反馈过期：立即断电流 */
    Chassis_Init(&c);
    reset_fb(t);
    fb[2].last_ms = t - 100;
    Chassis_Teleop(&c, t, 2, wheel, 1, fb, 0);
    for (int i = 0; i < 4; i++)
        assert(c.output[i] == 0);
    printf("遥控 反馈过期: 断电流 ok\n");

    /* 自旋叠加平移：一轮目标过零，但实测仍有惯性；不可因瞬时目标低而误停。 */
    Chassis_Init(&c);
    reset_fb(t);
    float crossing[4] = {0, 536, 900, 536};
    for (int i = 0; i < 4; i++)
        fb[i].rpm = 500;
    Chassis_Teleop(&c, t, 2, crossing, 1, fb, 0);
    assert(c.state == TELEOP && c.fault == F_NONE);
    /* 真正超过当前整车轮速包络仍锁存，下一帧不能自行解除。 */
    fb[0].rpm = 1900;
    Chassis_Teleop(&c, t, 2, crossing, 1, fb, 0);
    assert(c.state == TELEOP); /* 一个瞬时尖峰不锁存。 */
    for (int n = 0; n < 200 && c.state == TELEOP; n++)
    {
        t += 2;
        for (int i = 0; i < 4; i++)
            fb[i].last_ms = t;
        Chassis_Teleop(&c, t, 2, crossing, 1, fb, 0);
    }
    assert(c.state == FAULT && c.fault == F_OVERSPEED && c.fault_channel == 1);
    assert(g_teleop_fault[6] == 1 && g_teleop_fault[7] == 0);
    assert((int32_t)g_teleop_fault[12] == 1900);
    Chassis_Init(&c);
    reset_fb(t);
    fb[0].rpm = (int32_t)TELEOP_HARD_OVERSPEED_RPM + 100;
    Chassis_Teleop(&c, t, 2, crossing, 1, fb, 0);
    assert(c.state == FAULT && g_teleop_fault[7] == 1); /* 硬限仍立即保护。 */
    assert(g_teleop_fault[8] == 0 && g_teleop_fault[10] == 900000);
    printf("遥控 目标过零不误判、真实超速仍锁存 ok\n");

    /* 复现本次波形：回油门时目标骤降，惯性转速不能瞬间当成超速。 */
    Chassis_Init(&c);
    reset_fb(t);
    float cruise[4] = {500, 500, 500, 500}, crawl[4] = {50, 50, 50, 50};
    for (int i = 0; i < 4; i++)
        fb[i].rpm = 500;
    Chassis_Teleop(&c, t, 2, cruise, 1, fb, 0);
    Chassis_Teleop(&c, t + 2, 2, crawl, 1, fb, 0);
    assert(c.state == TELEOP && c.fault == F_NONE);
    /* 若转速迟迟不下降，过渡阈值须收紧并最终锁停，不能永久沿用旧高阈值。 */
    for (int n = 0; n < 300 && c.state == TELEOP; n++)
    {
        t += 2;
        for (int i = 0; i < 4; i++)
            fb[i].last_ms = t;
        Chassis_Teleop(&c, t, 2, crawl, 1, fb, 0);
    }
    assert(c.state == FAULT && c.fault == F_OVERSPEED);
    printf("遥控 减速惯性不误判、持续异常转速仍锁存 ok\n");

    /* CAN 错误：锁存停机 */
    Chassis_Init(&c);
    reset_fb(t);
    Chassis_Teleop(&c, t, 2, wheel, 1, fb, 1);
    assert(c.state == FAULT && c.fault == F_CAN);
    printf("遥控 CAN错误: 锁存 ok\n");
}

/* 复现本次现场问题：中位仍纠偏、零命令仍留积分、FAULT 被下一帧遥控覆盖。 */
static void test_rc_stop_regressions(void)
{
    uint16_t ch[16];
    for (int i = 0; i < 16; i++)
        ch[i] = 1024;
    ch[1] = 838;
    RcDriveCommand cmd;
    assert(RcDrive_Map(ch, 0, &cmd));
    assert(!cmd.vx_mm_s && !cmd.vy_mm_s && !cmd.omega_mrad_s);
    float heading = -2.0f, correction = 1.0f;
    assert(RcDrive_Omega(&cmd, 1.0f, 0.8f, 1, &heading, &correction) == 0.0f);
    assert(heading == 1.0f && correction == 0.0f);
    cmd.vx_mm_s = 100;
    heading = -2.0f;
    assert(RcDrive_Omega(&cmd, 1.0f, 0.8f, 0, &heading, &correction) == 0.0f);
    assert(correction == 0.0f); /* 关闭开关必须真正关闭纠偏。 */
    ch[2] = 1506;
    assert(RcDrive_Map(ch, 0, &cmd) && cmd.omega_mrad_s == -RC_SB_SPIN_MRAD_S);
    ch[2] = 460;
    assert(RcDrive_Map(ch, 0, &cmd) && cmd.omega_mrad_s == RC_SB_SPIN_MRAD_S);
    assert(!RcDrive_Map(ch, 0x08, &cmd) && !cmd.omega_mrad_s);
    assert(!RcDrive_Map(ch, 0x04, &cmd) && !cmd.omega_mrad_s);
    RcDriveReceiver rx = {0};
    assert(RcDrive_Receive(&rx, ch, 0, 1000, 1000, 1, &cmd) &&
           cmd.omega_mrad_s == RC_SB_SPIN_MRAD_S);
    assert(RcDrive_Receive(&rx, ch, 4, 1100, 1100, 1, &cmd) &&
           cmd.omega_mrad_s == RC_SB_SPIN_MRAD_S);
    assert(!RcDrive_Receive(&rx, ch, 4, 1201, 1201, 1, &cmd) && !cmd.omega_mrad_s);
    assert(RcDrive_Receive(&rx, ch, 0, 1300, 1300, 1, &cmd));
    assert(!RcDrive_Receive(&rx, ch, 8, 1301, 1301, 1, &cmd) && !cmd.omega_mrad_s);
    assert(!RcDrive_Receive(&rx, ch, 4, 1302, 1302, 1, &cmd));
    ch[1] = 2047;
    assert(!RcDrive_Map(ch, 0, &cmd));

    Chassis c;
    Chassis_Init(&c);
    t = 5000;
    reset_fb(t);
    float moving[4] = {500, 500, 500, 500}, zero[4] = {0};
    Chassis_Teleop(&c, t, 2, moving, 1, fb, 0);
    for (int i = 0; i < 4; i++)
        c.integral[i] = 300.0f;
    Chassis_Teleop(&c, t, 2, zero, 1, fb, 0);
    assert(c.state == IDLE);
    for (int i = 0; i < 4; i++)
    {
        assert(c.output[i] == 0 && c.integral[i] == 0 && c.target_rpm[i] == 0);
    }
    /* 回中时正在转：先反力矩制动，最长 1200 ms 后零电流，后续零命令不重复制动。 */
    Chassis_Teleop(&c, t, 2, moving, 1, fb, 0);
    for (int i = 0; i < 4; i++)
    {
        fb[i].rpm = 400;
        c.integral[i] = 1000;
    }
    Chassis_Teleop(&c, t, 2, zero, 1, fb, 0);
    assert(c.output[0] < 0 && c.target_rpm[0] == 0);
    for (int n = 0; n < 250; n++)
    {
        t += 2;
        for (int i = 0; i < 4; i++)
            fb[i].last_ms = t;
        Chassis_Teleop(&c, t, 2, zero, 1, fb, 0);
    }
    assert(c.state == TELEOP && c.output[0] < 0); /* 500ms仍运动，继续主动制动。 */
    for (int n = 0; n < 351; n++)
    {
        t += 2;
        for (int i = 0; i < 4; i++)
            fb[i].last_ms = t;
        Chassis_Teleop(&c, t, 2, zero, 1, fb, 0);
    }
    assert(c.state == FAULT && c.fault == F_STOP_TIMEOUT && c.output[0] == 0);
    t += 2;
    for (int i = 0; i < 4; i++)
        fb[i].last_ms = t;
    Chassis_Teleop(&c, t, 2, zero, 1, fb, 0);
    assert(c.state == FAULT && c.output[0] == 0);
    /* 人工停车、超速、CAN 故障均须保留首个故障，不能被后续遥控解除。 */
    const unsigned faults[] = {F_MANUAL, F_OVERSPEED, F_CAN, F_OTHER_CONTROLLER};
    for (unsigned i = 0; i < sizeof(faults) / sizeof(faults[0]); i++)
    {
        Chassis_Init(&c);
        Chassis_Stop(&c, faults[i]);
        Chassis_Teleop(&c, t, 2, moving, 1, fb, 0);
        assert(c.state == FAULT && c.fault == faults[i] && c.output[0] == 0);
    }
    Chassis_Init(&c);
    reset_fb(t);
    fb[2].last_ms = t - 100;
    for (int i = 0; i < 4; i++)
    {
        c.target_rpm[i] = 500;
        c.integral[i] = 200;
    }
    Chassis_Teleop(&c, t, 2, moving, 1, fb, 0);
    assert(c.state == FAULT && c.fault == F_FEEDBACK && c.fault_channel == 3);
    for (int i = 0; i < 4; i++)
        assert(!c.output[i] && !c.target_rpm[i] && !c.integral[i]);
    printf("回归：SB 回中/航向开关/失控标志/零速停车/制动期限/故障锁存通过\n");
}

static void test_field_translation(void)
{
    /* 用本次实测的两个旋转方向回归安装符号：原始 Z 与车体逆时针角速度同号。
     * 每步把场地命令混控再逆解，检验完整一圈后场地方向始终不随车头转。 */
    for (int dir = -1; dir <= 1; dir += 2)
    {
        float yaw = 0;
        float sensor = (float)dir * 0.985f;
        OmniX45Config cfg = OMNI;
        cfg.max_motor_rpm = 2880;
        for (int n = 0; n < 320; n++)
        {
            float physical_yaw = (float)dir * 0.985f * (float)(n + 1) * 0.02f;
            yaw = remainderf(yaw + CHASSIS_GYRO_Z_SIGN * sensor * 0.02f, 6.283185307f);
            float rpm[4];
            assert(omni_x45_rpm(&cfg, 0.2f, 0, (float)dir, yaw, rpm));
            float x = (-rpm[0] - rpm[1] + rpm[2] + rpm[3]) / (4.0f * WHEEL_RPM_PER_MPS);
            float y = (-rpm[0] + rpm[1] + rpm[2] - rpm[3]) / (4.0f * WHEEL_RPM_PER_MPS);
            assert(fabsf(cosf(physical_yaw) * x - sinf(physical_yaw) * y - 0.2f) < 0.0001f);
            assert(fabsf(sinf(physical_yaw) * x + cosf(physical_yaw) * y) < 0.0001f);
        }
    }
    /* 自旋同时前进：任意车头角，把轮速逆解回场地后仍应沿固定场地方向。
     * 检查两种自旋方向；这只验证数学一致性，不能替代实车安装方向验证。 */
    OmniX45Config cfg = OMNI;
    cfg.max_motor_rpm = 2880;
    for (int direction = -1; direction <= 1; direction += 2)
    {
        for (int step = -36; step <= 36; step++)
        {
            float yaw = step * 0.1745329252f, rpm[4];
            assert(omni_x45_rpm(&cfg, 0.2f, 0.1f, (float)direction, yaw, rpm));
            float k = WHEEL_RPM_PER_MPS;
            float x = (-rpm[0] - rpm[1] + rpm[2] + rpm[3]) / (4.0f * k);
            float y = (-rpm[0] + rpm[1] + rpm[2] - rpm[3]) / (4.0f * k);
            float om = -(rpm[0] + rpm[1] + rpm[2] + rpm[3]) / (4.0f * k * 0.31f);
            assert(fabsf(cosf(yaw) * x - sinf(yaw) * y - 0.2f) < 0.0001f);
            assert(fabsf(sinf(yaw) * x + cosf(yaw) * y - 0.1f) < 0.0001f);
            assert(fabsf(om - (float)direction) < 0.0001f);
        }
    }
    printf("场地变换：顺/逆自旋叠加平移，航向 -360..360 度通过\n");
}

static void test_right_stick_reference(void)
{
    uint16_t ch[16];
    for (int i = 0; i < 16; i++)
        ch[i] = 1024;
    ch[1] = 820;
    RcDriveCommand cmd;
    RcDriveFrame frame = {0};
    assert(RcDrive_Map(ch, 0, &cmd) && RcDrive_IsNeutral(&cmd));
    ch[0] = 460;
    assert(RcDrive_Map(ch, 0, &cmd));
    assert(cmd.turn_mrad_s < 0 && cmd.spin_mrad_s == 0 && cmd.omega_mrad_s == cmd.turn_mrad_s);
    ch[0] = 1506;
    assert(RcDrive_Map(ch, 0, &cmd) && cmd.turn_mrad_s > 0);
    /* SB 一直自旋：真实车头可以绕很多圈，参考航向必须精确不变。 */
    ch[0] = 1024;
    ch[2] = 460;
    assert(RcDrive_Map(ch, 0, &cmd) && cmd.spin_mrad_s == RC_SB_SPIN_MRAD_S);
    for (int n = 0; n < 1000; n++)
    {
        float yaw = (float)n * .02f;
        RcDrive_FrameStep(&frame, &cmd, yaw, 20, 1);
        assert(frame.heading_rad == 0.0f);
    }
    /* 右摇杆单独转向：新参考方向随指令旋转，松手后保留。 */
    ch[2] = 1024;
    ch[0] = 1506;
    assert(RcDrive_Map(ch, 0, &cmd));
    float expected = (float)cmd.turn_mrad_s / 1000;
    for (int n = 0; n < 50; n++)
        RcDrive_FrameStep(&frame, &cmd, expected * (float)(n + 1) / 50, 20, 1);
    assert(fabsf(frame.heading_rad - expected) < .00001f);
    ch[0] = 1024;
    assert(RcDrive_Map(ch, 0, &cmd));
    float rel = RcDrive_FrameStep(&frame, &cmd, expected, 20, 1);
    assert(fabsf(rel) < .00001f);
    for (int n = 0; n < 100; n++)
        RcDrive_FrameStep(&frame, &cmd, expected + .02f * n, 20, 1);
    assert(fabsf(frame.heading_rad - expected) < .00001f);
    /* 未解锁或异常周期不改参考方向。 */
    cmd.turn_mrad_s = 1000;
    cmd.omega_mrad_s = 1000;
    RcDrive_FrameStep(&frame, &cmd, expected, 20, 0);
    RcDrive_FrameStep(&frame, &cmd, expected, 1000, 1);
    assert(fabsf(frame.heading_rad - expected) < .00001f);
    /* 上档自旋与右转抵消时，仍有两个非中位输入，禁止上电解锁。 */
    ch[0] = 100;
    ch[2] = 460;
    assert(RcDrive_Map(ch, 0, &cmd) && cmd.spin_mrad_s == RC_SB_SPIN_MRAD_S &&
           cmd.turn_mrad_s == -RC_RIGHT_TURN_MRAD_S);
    cmd = (RcDriveCommand){.spin_mrad_s = 1000, .turn_mrad_s = -1000};
    assert(cmd.omega_mrad_s == 0 && !RcDrive_IsNeutral(&cmd));
    /* 当前 SB 速度叠加右摇杆：车体角随总指令，参考只积分右摇杆。 */
    frame.heading_rad = 0;
    cmd = (RcDriveCommand){.vx_mm_s = 100,
                           .omega_mrad_s = RC_SB_SPIN_MRAD_S + RC_RIGHT_TURN_MRAD_S,
                           .spin_mrad_s = RC_SB_SPIN_MRAD_S,
                           .turn_mrad_s = 1000};
    OmniX45Config cfg = OMNI;
    cfg.max_motor_rpm = TELEOP_MAX_TARGET_RPM;
    for (int n = 0; n < 50; n++)
    {
        float yaw = (float)cmd.omega_mrad_s / 1000.0f * (float)(n + 1) * .02f, rpm[4];
        float relative = RcDrive_FrameStep(&frame, &cmd, yaw, 20, 1);
        assert(omni_x45_rpm(&cfg, .1f, 0, (float)cmd.omega_mrad_s / 1000.0f, relative, rpm));
        float x = (-rpm[0] - rpm[1] + rpm[2] + rpm[3]) / (4 * WHEEL_RPM_PER_MPS);
        float y = (-rpm[0] + rpm[1] + rpm[2] - rpm[3]) / (4 * WHEEL_RPM_PER_MPS);
        assert(fabsf(cosf(yaw) * x - sinf(yaw) * y - .1f * cosf(frame.heading_rad)) < .00001f);
        assert(fabsf(sinf(yaw) * x + cosf(yaw) * y - .1f * sinf(frame.heading_rad)) < .00001f);
    }
    assert(fabsf(frame.heading_rad - 1.0f) < .00001f);
    cmd.turn_mrad_s = 0;
    cmd.omega_mrad_s = RC_SB_SPIN_MRAD_S;
    for (int n = 0; n < 500; n++)
        RcDrive_FrameStep(&frame, &cmd, 2.0f + .02f * n, 20, 1);
    assert(fabsf(frame.heading_rad - 1.0f) < .00001f);
    printf("右摇杆改参考航向、SB保持参考、两路叠加及相抵解锁保护通过\n");
}

static void test_two_rev_per_second(void)
{
    uint16_t ch[16];
    for (int i = 0; i < 16; i++)
        ch[i] = 1024;
    ch[1] = 820;
    ch[2] = 460;
    RcDriveCommand raw, out;
    RcDriveSpinRamp ramp = {0};
    RcDriveFrame frame = {0};
    assert(RcDrive_Map(ch, 0, &raw) && raw.spin_mrad_s == 12566);
    assert(!RcDrive_IsNeutral(&raw));
    const int up_step = RC_SPIN_ACCEL_MRAD_S2 * 20 / 1000;
    const int down_step = RC_SPIN_DECEL_MRAD_S2 * 20 / 1000;
    const int accel_steps = (RC_SB_SPIN_MRAD_S + up_step - 1) / up_step;
    const int decel_steps = (RC_SB_SPIN_MRAD_S + down_step - 1) / down_step;
    float last = 0;
    for (int n = 0; n < accel_steps; n++)
    {
        RcDrive_SpinStep(&ramp, &raw, 20, 1, &out);
        assert(ramp.spin_mrad_s >= last && ramp.spin_mrad_s - last <= (float)up_step + .01f);
        last = ramp.spin_mrad_s;
        RcDrive_FrameStep(&frame, &raw, .02f * n, 20, 1);
        assert(frame.heading_rad == 0);
    }
    assert(out.spin_mrad_s == 12566 && out.omega_mrad_s == 12566);
    float heading = 0, corr = 0;
    assert(fabsf(RcDrive_Omega(&out, 0, 0, 0, &heading, &corr) - 12.566f) < .00001f);
    OmniX45Config cfg = OMNI;
    cfg.max_motor_rpm = TELEOP_MAX_TARGET_RPM;
    float rpm[4];
    assert(omni_x45_rpm(&cfg, 0, 0, 12.566f, 0, rpm));
    for (int i = 0; i < 4; i++)
        assert(rpm[i] < -6700 && rpm[i] > -6800);
    assert(omni_x45_rpm(&cfg, .3f, .3f, 12.566f, 0, rpm));
    float spin_est = -(rpm[0] + rpm[1] + rpm[2] + rpm[3]) / (4 * WHEEL_RPM_PER_MPS * .31f);
    assert(fabsf(spin_est - 12.566f) < .001f);
    RcDriveCommand zero = {0};
    for (int n = 0; n < decel_steps; n++)
    {
        float before = ramp.spin_mrad_s;
        RcDrive_SpinStep(&ramp, &zero, 20, 1, &out);
        assert(ramp.spin_mrad_s <= before && before - ramp.spin_mrad_s <= (float)down_step + .01f);
    }
    assert(out.spin_mrad_s == 0);
    ramp.spin_mrad_s = RC_SB_SPIN_MRAD_S;
    raw.spin_mrad_s = -RC_SB_SPIN_MRAD_S;
    for (int n = 0; n < decel_steps; n++)
    {
        RcDrive_SpinStep(&ramp, &raw, 20, 1, &out);
        assert(out.spin_mrad_s >= 0);
    }
    RcDrive_SpinStep(&ramp, &raw, 20, 1, &out);
    assert(out.spin_mrad_s < 0 && out.spin_mrad_s >= -up_step - 1);
    ramp.spin_mrad_s = RC_SB_SPIN_MRAD_S;
    RcDrive_SpinStep(&ramp, &raw, 20, 0, &out);
    assert(out.spin_mrad_s == 0);
    ramp.spin_mrad_s = RC_SB_SPIN_MRAD_S;
    RcDrive_SpinStep(&ramp, &raw, 1000, 1, &out);
    assert(out.spin_mrad_s == 0);
    printf("每秒两圈轮速余量、SB渐加减速、先减速再反向和失联清零通过\n");
}

static void test_translation_prediction(void)
{
    RcDriveCommand cmd = {.vx_mm_s = 200};
    RcDriveFrame frame = {.heading_rad = .4f};
    for (int dir = -1; dir <= 1; dir += 2)
    {
        float gyro = (float)dir * 12.566f;
        float yaw = .7f;
        float predicted = RcDrive_MixingYaw(&cmd, yaw, gyro);
        float delta = remainderf(predicted - yaw, 6.283185307f);
        assert(fabsf(delta - gyro * .03f) < .00001f);
        /* 用35ms纯滞后模型检查补偿方向：两种旋向都应减小角误差，
         * 不允许因补偿符号错误把约25度扩大为约47度。 */
        float old_error = gyro * .035f;
        float compensated_error = old_error - delta;
        assert(fabsf(compensated_error) < fabsf(old_error) * .2f);
        RcDriveCommand spin = {.spin_mrad_s = dir * 12566, .omega_mrad_s = dir * 12566};
        RcDrive_FrameStep(&frame, &spin, 1.0f, 2, 1);
        assert(fabsf(frame.heading_rad - .4f) < .00001f);
    }
    assert(RcDrive_MixingYaw(&cmd, .7f, 0) == .7f);
    RcDriveCommand zero = {0};
    assert(RcDrive_MixingYaw(&zero, .7f, 12.566f) == .7f);
    assert(RcDrive_MixingYaw(&cmd, .7f, NAN) == .7f);
    float limited = RcDrive_MixingYaw(&cmd, 0, 100);
    assert(fabsf(limited - RC_TRANSLATION_LEAD_MAX_RAD) < .00001f);
    assert(isnan(RcDrive_MixingYaw(&cmd, NAN, 10)));
    printf("平移预测：双旋向补偿符号、静止归零、参考不变及异常限幅通过\n");
}

int main(void)
{
    test_translation_prediction();
    test_mixing();
    test_right_stick_reference();
    test_two_rev_per_second();
    test_field_translation();
    test_rc_stop_regressions();
    test_reject_not_ready();
    test_reject_bad_cmd();
    test_complete_run();
    test_overspeed();
    test_no_motion();
    test_reverse_motion();
    test_manual_stop_and_replay();
    test_teleop();
    printf("\n全部测试通过\n");
    return 0;
}
