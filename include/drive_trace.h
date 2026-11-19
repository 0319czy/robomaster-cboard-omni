#ifndef DRIVE_TRACE_H
#define DRIVE_TRACE_H
#include <stdint.h>

/* 板内一次性高速记录器。只观察控制数据；主机只写此模块的采集配置，
 * 不能通过采集令牌解锁遥控、改变目标或驱动电机。缓冲满后冻结，禁止边写边导出。 */
#define DRIVE_TRACE_MAGIC 0x54524331U
#define DRIVE_TRACE_CAPACITY 1000U
enum
{
    TRACE_IDLE = 0,
    TRACE_ARMED = 1,
    TRACE_RECORDING = 2,
    TRACE_FROZEN = 3
};
enum
{
    TM_MAGIC = 0,
    TM_VERSION,
    TM_FRAME_BYTES,
    TM_CAPACITY,
    TM_STATE,
    TM_COUNT,
    TM_ACK_TOKEN,
    TM_INTERVAL_MS,
    TM_TRIGGER,
    TM_START_MS,
    TM_END_MS,
    TM_MAX_DT_MS,
    TM_REQUEST_TOKEN,
    TM_REQUEST_MODE,
    TM_REQUEST_INTERVAL_MS,
    TM_RESERVED
};
/* 全部为固定宽度整数；转速rpm，角度mrad，角速度mrad/s，速度mm/s。
 * gyro[2] 已扣Z零偏；X/Y保留传感器坐标。各轮按CAN ID1..4排列。
 * command=本轮PI输出，queued=最近成功交给CAN发送邮箱的电流指令，
 * feedback_current=C620反馈报文4/5字节的有符号转矩电流原始值，三者分别保留。 */
typedef struct
{
    uint32_t time_ms;
    uint16_t step_dt_ms, imu_age_ms;
    int16_t target_rpm[4], actual_rpm[4], filtered_rpm[4];
    int16_t command[4], queued[4], feedback_current[4];
    int16_t gyro[3], yaw_mrad, reference_mrad, relative_mrad;
    int16_t vx_mm_s, vy_mm_s, omega_mrad_s;
    uint16_t state, fault, feedback_age_ms[4], loop_work_ms;
    uint32_t sequence, tx_fail;
} DriveTraceFrame;
_Static_assert(sizeof(DriveTraceFrame) == 96, "高速记录帧布局改变，必须同步更新主机解码");
extern volatile uint32_t g_trace_meta[16];
extern volatile DriveTraceFrame g_trace_frames[DRIVE_TRACE_CAPACITY];
/* 每个控制周期末调用；内部按时间戳抽样，不阻塞，不访问外设，不分配堆内存。
 * mode=0立即记录；1等到自旋指令超过1rad/s再开始；2提前冻结。
 * interval只接受4..20ms。token必须最后写入；采集中忽略重新启动请求。 */
void DriveTrace_Step(const DriveTraceFrame *frame);
#endif
