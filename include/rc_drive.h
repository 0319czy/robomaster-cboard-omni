#ifndef RC_DRIVE_H
#define RC_DRIVE_H
#include <stdint.h>

#define RC_TRANSLATION_MAX_MM_S 450 /* 前后/左右平移整体提高50%，死区保持原值 */

#define RC_SB_SPIN_MRAD_S 12566           /* SB 档整车自旋目标：约 1 秒两圈 */
#define RC_TOTAL_OMEGA_MRAD_S 14000       /* 含右摇杆叠加的合成角速度上限 */
#define RC_SPIN_ACCEL_MRAD_S2 20000       /* 指令约 0.63 s 爬升至每秒两圈，实际响应受电流限幅影响 */
#define RC_SPIN_DECEL_MRAD_S2 40000       /* 回中指令约 0.31 s 降到零，随后由反馈制动至停稳 */
#define RC_TRANSLATION_LEAD_MS 30         /* 100Hz实测约35ms等效滞后，先补偿30ms */
#define RC_TRANSLATION_LEAD_MAX_RAD 0.45f /* 只约束平移预测角，参考航向保持原值 */
#define RC_RIGHT_TURN_MRAD_S 1000         /* 右摇杆转向满量程 */

/* RF206S 的数值是 SBUS 原始刻度，不是微秒。
 * 通道：CH2 前后，CH4 左右，CH3/SB 自旋，CH1 右摇杆转向。 */
typedef struct
{
    int32_t vx_mm_s, vy_mm_s, omega_mrad_s;
    int32_t spin_mrad_s; /* SB：改变车体角度，保持平移参考方向 */
    int32_t turn_mrad_s; /* 右摇杆：改变车体角度，同时改变平移参考方向 */
} RcDriveCommand;
typedef struct
{
    float heading_rad;
} RcDriveFrame;
typedef struct
{
    float spin_mrad_s;
} RcDriveSpinRamp;
/* SB 高速目标作斜坡；反向先减到零再反向加速。失控/未解锁立即清除斜坡，
 * 由原停车路径处理，不能因斜坡延迟失控停车。原始指令仍用于回中解锁。 */
void RcDrive_SpinStep(RcDriveSpinRamp *ramp, const RcDriveCommand *requested, uint32_t dt_ms,
                      int enabled, RcDriveCommand *applied);
int RcDrive_IsNeutral(const RcDriveCommand *cmd);
/* 参考航向只积分右摇杆的转向指令；SB 动作不修改参考航向。
 * 返回车体相对该参考方向的角度，用于场地/参考方向到车体的混控变换。
 * 未解锁、失联、异常周期时不积累，防止恢复后突然改变操纵方向。 */
float RcDrive_FrameStep(RcDriveFrame *frame, const RcDriveCommand *cmd, float body_yaw,
                        uint32_t dt_ms, int enabled);
/* 平移混控使用预测角；相对航向及参考航向本身不被修改。 */
float RcDrive_MixingYaw(const RcDriveCommand *cmd, float relative_yaw, float gyro_rate);
typedef struct
{
    RcDriveCommand last;
    uint32_t last_ms;
    int valid;
} RcDriveReceiver;
/* 单帧丢失丢弃，最多保留上一有效命令 200 ms；失控标志立即失效。 */
int RcDrive_Receive(RcDriveReceiver *rx, const uint16_t channels[16], uint8_t flags,
                    uint32_t frame_ms, uint32_t now, int envelope_valid, RcDriveCommand *cmd);
int RcDrive_Map(const uint16_t channels[16], uint8_t flags, RcDriveCommand *cmd);
/* 完全回中时更新航向目标并返回零，禁止姿态控制把停车命令重新变成动作。 */
float RcDrive_Omega(const RcDriveCommand *cmd, float yaw, float gyro_rate, int heading_hold,
                    float *heading, float *correction);
#endif
