#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

#define MOTOR_COUNT 4
/* 同步实车标定：SB 上档 460、整车逆时针、BMI088 原始 Z 为正。 */
#define CHASSIS_GYRO_Z_SIGN (+1.0f)
#define TELEOP_MAX_TARGET_RPM 8000.0f
#define TELEOP_HARD_OVERSPEED_RPM 9000.0f
#define GEAR_RATIO (3591.0f / 187.0f)
#define WHEEL_RPM_PER_MPS (0.7071067812f * 9.549296586f * GEAR_RATIO / 0.075f)

/* 限时RAM动作与遥控共用的控制边界；遥控目标/硬超速上限见上方独立常量。 */
#define RUN_TIME_MIN_MS 200U
#define RUN_TIME_MAX_MS 20000U
#define RAMP_MS 1000U
#define CURRENT_LIMIT 3276 /* C620 原始单位，约 4 A 转矩电流指令 */
#define FEEDBACK_TIMEOUT_MS 30U
#define STOP_STILL_RPM 8
#define FILTER_MS 8.0f
#define KP 2.5f
#define KI 8.0f
#define CMD_VXY_LIMIT_MM_S 1200
#define CMD_OMEGA_LIMIT_MRAD_S 4000

enum
{
    IDLE = 0,
    RUNNING = 1,
    SETTLING = 2,
    COMPLETE = 3,
    FAULT = 4,
    TELEOP = 5
};
enum
{
    F_NONE = 0,
    F_NOT_READY = 1,
    F_FEEDBACK = 2,
    F_OVERSPEED = 3,
    F_CAN = 4,
    F_MANUAL = 5,
    F_OTHER_CONTROLLER = 6,
    F_NO_MOTION = 7,
    F_BAD_CMD = 8,
    F_STOP_TIMEOUT = 9,
    F_UNEXPECTED_MOTION = 10
};

/* CAN反馈快照：rpm为转子转速，angle为0..8191编码器刻度，last_ms为毫秒。 */
typedef struct
{
    int32_t rpm;
    uint32_t angle, last_ms, seen, temp;
} MotorFeedback;

typedef struct
{
    uint32_t state, start_ms, stop_ms, still_since, duration_ms;
    uint32_t ack_token, done_token, fault, fault_channel;
    float target_rpm[4], integral[4], filtered_rpm[4];
    float overspeed_trip; /* 遥控减速过渡中的收紧阈值，仍受硬上限约束 */
    float gain_scale;     /* 目标幅值相对已验证工况(576 转子rpm)的缩放 */
    int32_t output[4], last_angle[4], ticks[4];
    uint32_t reverse_since[4], overspeed_since[4], brake_since;
    uint32_t peak_rpm, peak_current;
} Chassis;

extern volatile uint32_t g_teleop_fault[32]; /* 超速现场：主机采集未启动时也可事后读取 */
void Chassis_Init(Chassis *c);
void Chassis_Request(Chassis *c, const float wheel_rpm[4], uint32_t duration_ms, uint32_t token,
                     uint32_t now, const MotorFeedback fb[4]);
void Chassis_Stop(Chassis *c, uint32_t reason);
void Chassis_Update(Chassis *c, uint32_t now, uint32_t dt_ms, const MotorFeedback fb[4],
                    uint32_t can_fault, uint32_t manual_stop);
/* 串口遥控通道：active=1 时按 wheel_rpm 闭环；active=0 时主动制动至低速，最长 1200 ms 后断电流。
 * 遥控丢帧进入短时制动；CAN 错误/电机反馈超时/超速等故障锁存，不得自动覆盖。 */
void Chassis_Teleop(Chassis *c, uint32_t now, uint32_t dt_ms, const float wheel_rpm[4], int active,
                    const MotorFeedback fb[4], uint32_t can_fault);
#endif
