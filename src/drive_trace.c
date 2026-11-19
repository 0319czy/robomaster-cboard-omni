#include "drive_trace.h"
#include <stdatomic.h>

volatile uint32_t g_trace_meta[16] = {[TM_MAGIC] = DRIVE_TRACE_MAGIC,
                                      [TM_VERSION] = 1,
                                      [TM_FRAME_BYTES] = sizeof(DriveTraceFrame),
                                      [TM_CAPACITY] = DRIVE_TRACE_CAPACITY,
                                      [TM_INTERVAL_MS] = 10,
                                      [TM_REQUEST_INTERVAL_MS] = 10};
volatile DriveTraceFrame g_trace_frames[DRIVE_TRACE_CAPACITY];
static uint32_t last_sample_ms;

void DriveTrace_Step(const DriveTraceFrame *frame)
{
    uint32_t request = g_trace_meta[TM_REQUEST_TOKEN];
    if (request && request != g_trace_meta[TM_ACK_TOKEN])
    {
        atomic_thread_fence(memory_order_acquire);
        uint32_t mode = g_trace_meta[TM_REQUEST_MODE];
        uint32_t period = g_trace_meta[TM_REQUEST_INTERVAL_MS];
        uint32_t state = g_trace_meta[TM_STATE];
        if (mode == 2U)
        {
            /* 提前冻结只结束观察，不向电机控制路径写任何停止或动作请求。 */
            atomic_thread_fence(memory_order_release);
            g_trace_meta[TM_STATE] = TRACE_FROZEN;
        }
        else if (state == TRACE_IDLE || state == TRACE_FROZEN)
        {
            if (mode <= 1U && period >= 4U && period <= 20U)
            {
                g_trace_meta[TM_COUNT] = 0;
                g_trace_meta[TM_INTERVAL_MS] = period;
                g_trace_meta[TM_TRIGGER] = mode;
                g_trace_meta[TM_START_MS] = 0;
                g_trace_meta[TM_END_MS] = 0;
                g_trace_meta[TM_MAX_DT_MS] = 0;
                atomic_thread_fence(memory_order_release);
                g_trace_meta[TM_STATE] = TRACE_ARMED;
            }
        }
        /* 非法参数、采集中重启均确认令牌但不改变本段内容，防止稍后意外重播。 */
        g_trace_meta[TM_ACK_TOKEN] = request;
    }
    if (!frame)
        return;
    if (g_trace_meta[TM_STATE] == TRACE_ARMED)
    {
        int spinning = frame->omega_mrad_s > 1000 || frame->omega_mrad_s < -1000;
        if (g_trace_meta[TM_TRIGGER] && !spinning)
            return;
        g_trace_meta[TM_START_MS] = frame->time_ms;
        last_sample_ms = frame->time_ms;
        g_trace_meta[TM_STATE] = TRACE_RECORDING;
    }
    if (g_trace_meta[TM_STATE] != TRACE_RECORDING)
        return;
    uint32_t count = g_trace_meta[TM_COUNT];
    if (count >= DRIVE_TRACE_CAPACITY)
        return;
    /* 无符号差允许HAL毫秒计数回绕；第一帧立即保存。 */
    if (count && (uint32_t)(frame->time_ms - last_sample_ms) < g_trace_meta[TM_INTERVAL_MS])
        return;
    g_trace_frames[count] = *frame;
    last_sample_ms = frame->time_ms;
    g_trace_meta[TM_END_MS] = frame->time_ms;
    if (frame->step_dt_ms > g_trace_meta[TM_MAX_DT_MS])
        g_trace_meta[TM_MAX_DT_MS] = frame->step_dt_ms;
    atomic_thread_fence(memory_order_release);
    g_trace_meta[TM_COUNT] = ++count;
    if (count == DRIVE_TRACE_CAPACITY)
    {
        atomic_thread_fence(memory_order_release);
        g_trace_meta[TM_STATE] = TRACE_FROZEN;
    }
}
