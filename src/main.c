#include "stm32f4xx_hal.h"
#include "chassis.h"
#include "omni_x45.h"
#include "uart_teleop.h"
#include "rc_drive.h"
#include "rc_pwm.h"
#include "rc_dbus.h"
#include "rc_serial.h"
#include "rc_sniff.h"
#include "bmi088.h"
#include "drive_trace.h"
#include <math.h>
#include <stdlib.h>
#include "rc_sniff_dummy.h"
extern volatile uint32_t g_dbus_edges;
extern volatile uint16_t g_dbus_high_us;
#include <string.h>

/* 全向底盘混控 + 四轮速度闭环固件。
 * 遥控器全回中后解锁；SBUS、串口遥控和限时 RAM 指令共用四轮速度控制。
 * 手动停车优先于所有控制源，故障锁存；CAN/GPIO/串口/SPI 均通过 HAL 操作。
 */
CAN_HandleTypeDef hcan1;
static IWDG_HandleTypeDef hiwdg;
static Chassis control;
static volatile MotorFeedback motors[4];
static volatile uint32_t can_error, other_controller;
static volatile uint32_t total_rx;
static uint32_t tx_success, tx_fail;
static int32_t last_queued[4];

#define CMD_MAGIC 0x43484331U /* 'CHC1' */
/* 主机先写参数，最后写 token 提交；复位后全零，不会重播。 */
volatile uint32_t g_cmd[8];
volatile uint32_t g_stop_request;

#define STATUS_WORDS 64
volatile uint32_t g_chassis_status[STATUS_WORDS] = {0x43485331U, 1U};

/* 车体几何与方向：轮径/轴距/轮距为按照片与轮径 15 cm 修正后的初值。
 * sign为电调正转相对混控轮速正方向的符号；四轮从里看外顺时针为正，
 * 当前均为+1。旋转项的物理符号已通过SB与陀螺同步观察确认。 */
static const OmniX45Config OMNI = {
    .half_length_m = 0.150f,
    .half_width_m = 0.160f,
    .wheel_radius_m = 0.075f,
    .motor_turns_per_wheel_turn = 19.203208556f,
    .max_motor_rpm = TELEOP_MAX_TARGET_RPM, /* 转子目标上限8000rpm；混合运动按比例限速 */
    .sign = {1, 1, 1, 1},
};

static int32_t last_vx_mm, last_vy_mm, last_omega_mrad;
/* 诊断：C620 反馈里的实际转矩电流（原始单位）与状态字 */
volatile int32_t g_can_fb_current[4];
volatile uint32_t g_can_fb_status[4];
static TeleopCmd teleop;
static int teleop_active;
/* 遥控映射与死区集中在 rc_drive.c；SB 控制 CH3 自旋，CH1 右摇杆控制转向。
 * 默认只做场地坐标变换。启用航向保持前须实车核对陀螺/混控方向一致。 */
#ifndef HEADING_HOLD_ENABLE
#define HEADING_HOLD_ENABLE 0
#endif
/* 控制构建允许遥控驱动；诊断构建设为0，并在CAN最终打包处强制零电流。 */
#ifndef RC_DRIVE_ENABLE
#define RC_DRIVE_ENABLE 1
#endif
/* 诊断构建在最终 CAN 打包处强制零电流，所有控制源均无法绕过。 */
#ifndef MOTOR_OUTPUT_ENABLE
#define MOTOR_OUTPUT_ENABLE 1
#endif
static uint16_t rc_width[RC_CHANNELS];
static uint16_t rc_dbus_us;
static int rc_armed;
static RcDriveReceiver rc_receiver;
static RcDriveFrame rc_frame;
static RcDriveSpinRamp rc_spin_ramp;
static RcDriveCommand rc_last_cmd;
static float rc_relative_yaw;
/* [0]=RCF1 [1]=平移参考航向 [2]=车体相对参考角 [3]=右摇杆转向
 * [4]=SB自旋 [5]=总角速度（角度mrad、角速度mrad/s）[6]=时间ms [7]=解锁 */
volatile int32_t g_rc_frame[8] = {0x52434631};
static int dbg_have_sbus;
static int32_t dbg_sbus_age;
static int dbg_rc_active;
static float dbg_yaw_corr;
/* 调试：航向/变换中间量 */
volatile int32_t g_yawdbg[8];
volatile int32_t g_omni_dbg[6]; /* [0]=x [1]=y [2]=yaw [3]=om_mr [4]=vx [5]=vy */
/* IMU 诊断与航向：g_imu[0]=gyro id [1]=accel id [2..4]=角速度 mrad/s [5]=加速度z
 * [6]=航向 mrad [7]=零偏 mrad/s [8]=校准样本数
 * [9]=航向目标 mrad [10]=纠偏 mrad/s [11]=传感器原始 Z mrad/s
 * [4] 为安装方向修正后的车体 Z，逆时针为正。 */
volatile int32_t g_imu[12];
static float yaw_rad, gyro_bias_z, yaw_setpoint;
static float gyro_rate_z; /* 实测角速度（已扣零偏） */
static uint32_t cal_count, gyro_last_ms, neutral_since;
static int neutral_waiting;
static float cal_sum;
static uint32_t rc_valid;

static void Fatal(uint32_t code)
{
    g_chassis_status[4] = 0x100U + code;
    while (1)
    {
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* 原理图标注外部晶振 12 MHz。PLL: 12/12*336/2 = 168 MHz。 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 12;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Fatal(1);
    clk.ClockType =
        RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
        Fatal(2);
}

static void TxGuard_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /* 先预置高电平再切换输出模式，避免初始化时出现低电平脉冲。 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *can)
{
    if (can->Instance != CAN1)
        return;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

static void CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
        Fatal(3);
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        Fatal(4);
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Fatal(5);
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                                 CAN_IT_ERROR | CAN_IT_BUSOFF |
                                                 CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING |
                                                 CAN_IT_LAST_ERROR_CODE) != HAL_OK)
        Fatal(6);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0))
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        total_rx++;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
            continue;
        /* 见到其他控制器的电流帧即停止争用。 */
        if (header.StdId == 0x200U || header.StdId == 0x1FFU || header.StdId == 0x2FFU)
            other_controller = 1;
        if (header.StdId < 0x201U || header.StdId > 0x204U || header.DLC != 8U)
            continue;
        uint32_t index = header.StdId - 0x201U;
        motors[index].rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        /* C620反馈4/5字节为转矩电流，高速记录时与rpm一起冻结快照。 */
        g_can_fb_current[index] = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        motors[index].angle = ((uint32_t)data[0] << 8) | data[1];
        motors[index].temp = data[6];
        motors[index].last_ms = HAL_GetTick();
        motors[index].seen = 1;
    }
}

volatile uint32_t can_error_ms; /* 最后一次 CAN 错误的时刻（0=从未出错） */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    can_error |= HAL_CAN_GetError(can);
    can_error_ms = HAL_GetTick();
}

static void SendCurrents(void)
{
    uint8_t data[8];
    uint32_t mailbox;
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = 0x200U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    for (int i = 0; i < 4; i++)
    {
        int32_t value = control.output[i];
        /* TELEOP(串口/遥控受控) 同样必须把输出真正发出去，之前这里漏了 TELEOP，
         * 导致遥控时 CAN 帧恒为 0、电调收不到油门（绿灯闪烁待机）。 */
        if (!MOTOR_OUTPUT_ENABLE || (control.state != RUNNING && control.state != TELEOP))
            value = 0;
        if (value > CURRENT_LIMIT)
            value = CURRENT_LIMIT;
        else if (value < -CURRENT_LIMIT)
            value = -CURRENT_LIMIT;
        data[2 * i] = (uint8_t)((uint16_t)(int16_t)value >> 8);
        data[2 * i + 1] = (uint8_t)value;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U)
    {
        /* 不排队旧电流；残留帧未发出时放弃本轮并进入零电流故障。 */
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        Chassis_Stop(&control, F_CAN);
        memset(data, 0, sizeof(data));
        tx_fail++;
    }
    if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)
    {
        tx_fail++;
        Chassis_Stop(&control, F_CAN);
    }
    else
    {
        tx_success++;
        for (unsigned i = 0; i < MOTOR_COUNT; i++)
            last_queued[i] = (int16_t)(((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
    }
}

static void Publish(uint32_t now, const MotorFeedback fb[4])
{
    g_chassis_status[2]++;
    __DMB();
    g_chassis_status[3] = now;
    g_chassis_status[4] = control.state;
    g_chassis_status[5] = control.fault;
    g_chassis_status[6] = control.fault_channel;
    g_chassis_status[7] = control.start_ms ? (uint32_t)(now - control.start_ms) : 0U;
    g_chassis_status[8] = control.duration_ms;
    g_chassis_status[9] = control.ack_token;
    g_chassis_status[10] = control.done_token;
    for (int i = 0; i < 4; i++)
    {
        g_chassis_status[11 + i] = (uint32_t)(int32_t)(control.target_rpm[i] * 1000.0f);
        g_chassis_status[15 + i] = (uint32_t)fb[i].rpm;
        g_chassis_status[19 + i] = (uint32_t)control.output[i];
        g_chassis_status[23 + i] = fb[i].temp;
        g_chassis_status[27 + i] = fb[i].seen ? (uint32_t)(now - fb[i].last_ms) : 0xFFFFFFFFU;
        g_chassis_status[41 + i] = (uint32_t)control.ticks[i];
        g_chassis_status[45 + i] = (uint32_t)(int32_t)(control.filtered_rpm[i] * 1000.0f);
    }
    g_chassis_status[31] = can_error;
    g_chassis_status[32] = total_rx;
    g_chassis_status[33] = tx_success;
    g_chassis_status[34] = tx_fail;
    g_chassis_status[35] = other_controller;
    g_chassis_status[36] = control.peak_rpm;
    g_chassis_status[37] = control.peak_current;
    g_chassis_status[38] = (uint32_t)last_vx_mm;
    g_chassis_status[39] = (uint32_t)last_vy_mm;
    g_chassis_status[40] = (uint32_t)last_omega_mrad;
    g_chassis_status[49] = (uint32_t)teleop_active;
    g_chassis_status[50] = (uint32_t)teleop.vx_mm_s;
    g_chassis_status[51] = (uint32_t)teleop.vy_mm_s;
    g_chassis_status[52] = (uint32_t)teleop.omega_mrad_s;
    g_chassis_status[53] =
        control.state == TELEOP || teleop_active ? (uint32_t)(now - teleop.last_ms) : 0xFFFFFFFFU;
    g_chassis_status[54] = teleop.rx_bytes;
    g_chassis_status[55] = teleop.rx_lines;
    g_chassis_status[56] = teleop.rx_errors;
    for (int i = 0; i < 5; i++)
        g_chassis_status[57 + i] =
            rc_width[i]; /* 旧诊断布局：57..60为C1..C4，61随后由解锁标志覆盖 */
    g_chassis_status[61] = (uint32_t)rc_armed; /* 遥控解锁标志 */
    g_chassis_status[62] =
        (uint32_t)(dbg_have_sbus | (dbg_rc_active << 8)); /* 接收有效/控制有效位 */
    g_chassis_status[63] =
        rc_valid | ((rc_dbus_us ? 1U : 0U) << 8) | ((g_dbus_edges & 0xFFFFU) << 16);
    __DMB();
    g_rc_frame[1] = (int32_t)(rc_frame.heading_rad * 1000.0f);
    g_rc_frame[2] = (int32_t)(rc_relative_yaw * 1000.0f);
    g_rc_frame[3] = rc_last_cmd.turn_mrad_s;
    g_rc_frame[4] = rc_last_cmd.spin_mrad_s;
    g_rc_frame[5] = rc_last_cmd.omega_mrad_s;
    g_rc_frame[6] = (int32_t)now;
    g_rc_frame[7] = rc_armed;
    __DMB();
    g_chassis_status[2]++;
}

/* 高速记录与电机控制解耦：只有主机采集令牌生效时才组帧，满后自动冻结。
 * CAN转速/电流取自同一临界区快照；陀螺来自本次读取或保留其陈旧时间。
 * 不在中断内拷贝大块缓存，不通过串口打印，不暂停控制主循环。 */
static int16_t TraceI16(int32_t value)
{
    if (value > 32767)
        return 32767;
    if (value < -32768)
        return -32768;
    return (int16_t)value;
}
static uint16_t TraceAge(uint32_t age)
{
    return age > 65535U ? 65535U : (uint16_t)age;
}
static void TraceRecord(uint32_t now, uint32_t dt, const MotorFeedback fb[4],
                        const int32_t feedback_current[4])
{
    static uint32_t sequence;
    sequence++;
    DriveTrace_Step(NULL);
    uint32_t state = g_trace_meta[TM_STATE];
    if (state != TRACE_ARMED && state != TRACE_RECORDING)
        return;
    DriveTraceFrame frame = {0};
    frame.time_ms = now;
    frame.step_dt_ms = TraceAge(dt);
    frame.imu_age_ms = TraceAge((uint32_t)(now - gyro_last_ms));
    for (int i = 0; i < 4; i++)
    {
        frame.target_rpm[i] = TraceI16((int32_t)lroundf(control.target_rpm[i]));
        frame.actual_rpm[i] = TraceI16(fb[i].rpm);
        frame.filtered_rpm[i] = TraceI16((int32_t)lroundf(control.filtered_rpm[i]));
        frame.command[i] = TraceI16(control.output[i]);
        frame.queued[i] = TraceI16(last_queued[i]);
        frame.feedback_current[i] = TraceI16(feedback_current[i]);
        frame.feedback_age_ms[i] = fb[i].seen ? TraceAge((uint32_t)(now - fb[i].last_ms)) : 65535U;
    }
    frame.gyro[0] = TraceI16(g_imu[2]);
    frame.gyro[1] = TraceI16(g_imu[3]);
    frame.gyro[2] = TraceI16((int32_t)lroundf(gyro_rate_z * 1000.0f));
    frame.yaw_mrad = TraceI16((int32_t)lroundf(yaw_rad * 1000.0f));
    frame.reference_mrad = TraceI16((int32_t)lroundf(rc_frame.heading_rad * 1000.0f));
    frame.relative_mrad = TraceI16((int32_t)lroundf(rc_relative_yaw * 1000.0f));
    frame.vx_mm_s = TraceI16(last_vx_mm);
    frame.vy_mm_s = TraceI16(last_vy_mm);
    frame.omega_mrad_s = TraceI16(last_omega_mrad);
    frame.state = (uint16_t)control.state;
    frame.fault = (uint16_t)control.fault;
    frame.loop_work_ms = TraceAge((uint32_t)(HAL_GetTick() - now));
    frame.sequence = sequence;
    frame.tx_fail = tx_fail;
    DriveTrace_Step(&frame);
}

/* 把主机写入的 (vx, vy, omega) 换算成四轮转子 rpm 目标；参数非法则锁定故障。 */
static void HandleCommand(uint32_t now, const MotorFeedback fb[4])
{
    uint32_t token = g_cmd[6];
    __DMB();
    if (!token || token == control.ack_token)
        return;
    int32_t vx = (int32_t)g_cmd[2], vy = (int32_t)g_cmd[3], omega = (int32_t)g_cmd[4];
    uint32_t duration = g_cmd[5];
    if (g_cmd[0] != CMD_MAGIC || g_cmd[1] != 1U || vx < -CMD_VXY_LIMIT_MM_S ||
        vx > CMD_VXY_LIMIT_MM_S || vy < -CMD_VXY_LIMIT_MM_S || vy > CMD_VXY_LIMIT_MM_S ||
        omega < -CMD_OMEGA_LIMIT_MRAD_S || omega > CMD_OMEGA_LIMIT_MRAD_S)
    {
        control.ack_token = token;
        Chassis_Stop(&control, F_BAD_CMD);
        return;
    }
    float wheel[4];
    if (!omni_x45_rpm(&OMNI, (float)vx / 1000.0f, (float)vy / 1000.0f, (float)omega / 1000.0f, 0.0f,
                      wheel))
    {
        control.ack_token = token;
        Chassis_Stop(&control, F_BAD_CMD);
        return;
    }
    last_vx_mm = vx;
    last_vy_mm = vy;
    last_omega_mrad = omega;
    Chassis_Request(&control, wheel, duration, token, now, fb);
}

int main(void)
{
    HAL_Init();
    TxGuard_Init();
    SystemClock_Config();
    Chassis_Init(&control);
    CAN_Init();
    Teleop_Init(); /* USART1备用ASCII调试口，115200；当前主要使用SBUS */
    RcPwm_Init();  /* PWM 口 C1..C7 输入捕获读遥控 */
    /* PC11接入反相后的SBUS串行流；具体8E2配置由RcSerial_Init统一设置。 */
    RcSerial_Init(100000, 1); /* SBUS: 100k 8E2（偶校验+2停止位） */
    Bmi088_Init();            /* 板载 BMI088：SPI1，仅 SWD 模式 */
    g_imu[0] = Bmi088_GyroId();
    g_imu[1] = Bmi088_AccelId();
    Teleop_Stop(&teleop, HAL_GetTick());
    /* 约 125 ms 独立看门狗：主循环卡死即复位，且令牌清零不自动重启电机。 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = 999;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        Fatal(7);
    __HAL_DBGMCU_UNFREEZE_IWDG();

    uint32_t last_step = HAL_GetTick(), last_tx = last_step, last_publish = last_step;
    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_step) >= 2U)
        {
            MotorFeedback fb[4];
            int32_t feedback_current[4];
            HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
            for (int i = 0; i < 4; i++)
            {
                fb[i] = motors[i];
                feedback_current[i] = g_can_fb_current[i];
            }
            now = HAL_GetTick();
            HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

            uint32_t can_fault = other_controller ? 2U : (can_error ? 1U : 0U);
            RcSerial_ScanStep(now);
            /* IMU：前 500 次采样做零偏校准（静止），之后积分航向 */
            {
                float gx, gy, gz, ax, ay, az;
                if (Bmi088_ReadGyro(&gx, &gy, &gz) == 0)
                {
                    /* 物理方向标定：逆时针时原始 Z 为正，保留该符号。
                     * 偏置标定、航向积分和阻尼都必须统一为逆时针正。 */
                    g_imu[11] = (int32_t)(gz * 1000.0f); /* 原始传感器 Z，便于复查安装方向 */
                    gz *= CHASSIS_GYRO_Z_SIGN;
                    gyro_last_ms = now;
                    if (cal_count < 500U)
                    {
                        /* 零偏仅在车体静止时采集，防止上电自旋被当作零偏。 */
                        int still = fabsf(gz) < 0.15f;
                        for (int i = 0; i < 4; i++)
                            if (abs(fb[i].rpm) > 20)
                                still = 0;
                        if (still)
                        {
                            cal_sum += gz;
                            cal_count++;
                        }
                        else
                        {
                            cal_sum = 0.0f;
                            cal_count = 0;
                        }
                        if (cal_count == 500U)
                            gyro_bias_z = cal_sum / 500.0f;
                    }
                    else
                    {
                        float dt_s = (float)(now - last_step) / 1000.0f;
                        if (dt_s > 0.0f && dt_s < 0.05f)
                            yaw_rad = remainderf(yaw_rad + (gz - gyro_bias_z) * dt_s, 6.283185307f);
                    }
                    g_imu[2] = (int32_t)(gx * 1000.0f);
                    g_imu[3] = (int32_t)(gy * 1000.0f);
                    g_imu[4] = (int32_t)(gz * 1000.0f);
                    if (cal_count >= 500U)
                        gyro_rate_z = gz - gyro_bias_z;
                }
                if (Bmi088_ReadAccel(&ax, &ay, &az) == 0)
                    g_imu[5] = (int32_t)(az * 1000.0f);
                g_imu[6] = (int32_t)(yaw_rad * 1000.0f);
                g_imu[7] = (int32_t)(gyro_bias_z * 1000.0f);
                g_imu[8] = (int32_t)cal_count;
                g_imu[9] = (int32_t)(yaw_setpoint * 1000.0f);
                g_imu[10] = (int32_t)(dbg_yaw_corr * 1000.0f);
            }
            rc_valid = RcPwm_Read(now, rc_width);
            rc_dbus_us = RcDbus_Read(now);
            teleop_active = Teleop_Poll(now, &teleop);
            Teleop_Heartbeat(now);

            /* 同一帧的通道、标志和时间戳一起拷贝，避免串口中断撕裂数据。 */
            uint16_t channels[16];
            uint32_t sbus_magic, sbus_frames, sbus_flags, sbus_ms;
            HAL_NVIC_DisableIRQ(USART3_IRQn);
            sbus_magic = g_sbus[0];
            sbus_frames = g_sbus[1];
            sbus_flags = g_sbus[18];
            sbus_ms = g_sbus_last_ms;
            for (int i = 0; i < 16; i++)
                channels[i] = (uint16_t)g_sbus[2 + i];
            HAL_NVIC_EnableIRQ(USART3_IRQn);
            /* 使用复制帧之后的当前时刻；本轮 now 可能早于中断刚收到的新帧。 */
            int32_t sbus_age = (int32_t)(HAL_GetTick() - sbus_ms);
            RcDriveCommand rc = {0};
            int have_sbus =
                RcDrive_Receive(&rc_receiver, channels, (uint8_t)sbus_flags, sbus_ms, HAL_GetTick(),
                                sbus_magic == 0x53425553U && sbus_frames != 0U, &rc);
            int imu_ok =
                g_imu[0] == 0x0f && cal_count >= 500U && (uint32_t)(now - gyro_last_ms) <= 20U;
            int rc_active = 0;
            float rc_wheel[4] = {0};
            dbg_have_sbus = have_sbus;
            dbg_sbus_age = sbus_age;
            /* 上电/失联恢复后，全回中连续 500 ms 才解锁。SB 偏档不能自行起转。 */
            if (!have_sbus || !imu_ok || control.state == FAULT || g_stop_request)
            {
                rc_armed = 0;
                neutral_waiting = 0;
            }
            else if (!rc_armed)
            {
                if (RcDrive_IsNeutral(&rc))
                {
                    if (!neutral_waiting)
                    {
                        neutral_since = now;
                        neutral_waiting = 1;
                    }
                    if ((uint32_t)(now - neutral_since) >= 500U)
                    {
                        rc_armed = 1;
                        yaw_setpoint = yaw_rad;
                    }
                }
                else
                    neutral_waiting = 0;
            }
            RcDriveCommand applied;
            RcDrive_SpinStep(&rc_spin_ramp, &rc, (uint32_t)(now - last_step),
                             RC_DRIVE_ENABLE && have_sbus && imu_ok && rc_armed, &applied);
            rc_last_cmd = applied;
            rc_relative_yaw =
                RcDrive_FrameStep(&rc_frame, &rc, yaw_rad, (uint32_t)(now - last_step),
                                  RC_DRIVE_ENABLE && have_sbus && imu_ok && rc_armed);
            last_vx_mm = rc.vx_mm_s;
            last_vy_mm = rc.vy_mm_s;
            last_omega_mrad = applied.omega_mrad_s;
            dbg_yaw_corr = 0.0f;
            if (have_sbus && imu_ok && rc_armed)
            {
                float omega = RcDrive_Omega(&applied, yaw_rad, gyro_rate_z, HEADING_HOLD_ENABLE,
                                            &yaw_setpoint, &dbg_yaw_corr);
                rc_active = omni_x45_rpm(
                    &OMNI, (float)rc.vx_mm_s / 1000.0f, (float)rc.vy_mm_s / 1000.0f, omega,
                    RcDrive_MixingYaw(&rc, rc_relative_yaw, gyro_rate_z), rc_wheel);
            }
            dbg_rc_active = rc_active;

            /* ---- CAN 故障自动恢复 ----
             * 只关电池时电调掉电、MCU 仍由 USB 供电 → CAN 无应答 → 锁存 F_CAN。
             * 电池回来后：总线连续 1 秒无错误且四轮反馈齐全，就自动清除该故障，
             * 并要求重新回中解锁（其它故障仍保持锁存）。 */
            {
                static uint32_t can_clean_since;
                int fb_ok = 1;
                for (int i = 0; i < 4; i++)
                    if (!fb[i].seen || (uint32_t)(now - fb[i].last_ms) > FEEDBACK_TIMEOUT_MS)
                        fb_ok = 0;
                uint32_t since_err = can_error_ms ? (uint32_t)(now - can_error_ms) : 0xFFFFFFFFU;
                if (other_controller || since_err < 1000U)
                    can_clean_since = 0;
                else if (!can_clean_since)
                    can_clean_since = now;
                if (control.state == FAULT && control.fault == F_CAN && fb_ok && can_clean_since &&
                    (uint32_t)(now - can_clean_since) > 1000U)
                {
                    Chassis_Init(&control);
                    can_error = 0;
                    can_error_ms = 0;
                    rc_armed = 0;
                    can_clean_since = now;
                }
            }
            /* 停车请求优先于所有控制源；故障不得被 TELEOP 路径覆盖。 */
            if (g_stop_request)
            {
                Chassis_Stop(&control, F_MANUAL);
            }
            else if (RC_DRIVE_ENABLE && rc_active)
            {
                Chassis_Teleop(&control, now, (uint32_t)(now - last_step), rc_wheel, 1, fb,
                               can_fault);
            }
            else if (control.state == TELEOP && !teleop_active)
            {
                /* 遥控失效（摇杆丢信号/对频断开）：立即按零速制动。 */
                float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                Chassis_Teleop(&control, now, (uint32_t)(now - last_step), zero, 0, fb, can_fault);
            }
            else if (teleop_active || control.state == TELEOP)
            {
                float wheel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                int active = teleop_active;
                if (active && !omni_x45_rpm(&OMNI, (float)teleop.vx_mm_s / 1000.0f,
                                            (float)teleop.vy_mm_s / 1000.0f,
                                            (float)teleop.omega_mrad_s / 1000.0f, 0.0f, wheel))
                    active = 0; /* 参数异常按停止处理，不使用旧值 */
                if (active)
                {
                    last_vx_mm = teleop.vx_mm_s;
                    last_vy_mm = teleop.vy_mm_s;
                    last_omega_mrad = teleop.omega_mrad_s;
                }
                Chassis_Teleop(&control, now, (uint32_t)(now - last_step), wheel, active, fb,
                               can_fault);
            }
            else
            {
                HandleCommand(now, fb);
                Chassis_Update(&control, now, (uint32_t)(now - last_step), fb, can_fault,
                               g_stop_request);
            }
            uint32_t control_dt = (uint32_t)(now - last_step);
            last_step = now;
            if ((uint32_t)(now - last_tx) >= 4U)
            {
                SendCurrents();
                last_tx = now;
            }
            TraceRecord(now, control_dt, fb, feedback_current);
            if ((uint32_t)(now - last_publish) >= 100U)
            {
                Publish(now, fb);
                last_publish = now;
            }
            HAL_IWDG_Refresh(&hiwdg);
            __HAL_DBGMCU_UNFREEZE_IWDG();
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
void HardFault_Handler(void)
{
    HAL_NVIC_SystemReset();
}
