#include "drive_trace.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t token;
static void request(uint32_t mode, uint32_t interval)
{
    g_trace_meta[TM_REQUEST_MODE] = mode;
    g_trace_meta[TM_REQUEST_INTERVAL_MS] = interval;
    g_trace_meta[TM_REQUEST_TOKEN] = ++token;
    DriveTrace_Step(NULL);
}
int main(void)
{
    DriveTraceFrame f = {0};
    assert(sizeof(f) == 96);
    request(1, 10);
    assert(g_trace_meta[TM_STATE] == TRACE_ARMED);
    for (unsigned i = 0; i < 50; i++)
    {
        f.time_ms = i * 2;
        DriveTrace_Step(&f);
    }
    assert(g_trace_meta[TM_COUNT] == 0); /* 等待自旋，不记录静止等待时间 */
    f.omega_mrad_s = 12566;
    f.time_ms = 100;
    f.sequence = 50;
    f.target_rpm[0] = -6735;
    DriveTrace_Step(&f);
    assert(g_trace_meta[TM_COUNT] == 1 && g_trace_frames[0].sequence == 50);
    request(0, 4); /* 采集中不能重启/修改间隔 */
    assert(g_trace_meta[TM_COUNT] == 1 && g_trace_meta[TM_INTERVAL_MS] == 10);
    for (unsigned i = 1; i < 5000; i++)
    {
        f.time_ms = 100 + i * 2;
        f.sequence = 50 + i;
        f.step_dt_ms = 2;
        DriveTrace_Step(&f);
    }
    assert(g_trace_meta[TM_STATE] == TRACE_FROZEN && g_trace_meta[TM_COUNT] == 1000);
    assert(g_trace_frames[999].time_ms == 10090);
    f.time_ms = 99999;
    DriveTrace_Step(&f);
    assert(g_trace_frames[999].time_ms == 10090); /* 满缓冲不可覆盖 */
    request(0, 1);
    assert(g_trace_meta[TM_COUNT] == 1000); /* 非法参数不清除数据 */
    request(0, 10);
    f.time_ms = 0xfffffff8U;
    DriveTrace_Step(&f);
    f.time_ms = 2;
    DriveTrace_Step(&f);
    assert(g_trace_meta[TM_COUNT] == 2); /* 毫秒回绕仍正确抽样 */
    request(2, 10);
    assert(g_trace_meta[TM_STATE] == TRACE_FROZEN);
    assert(g_trace_meta[TM_COUNT] == 2);
    puts("高速记录：触发、抽样间隔、容量冻结、重入保护、非法参数和时间回绕通过");
}
