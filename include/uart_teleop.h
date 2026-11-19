#ifndef UART_TELEOP_H
#define UART_TELEOP_H
#include <stdint.h>

/* 备用ASCII串口：当前初始化USART1（PA9 TX / PB7 RX），115200 8N1。
 * USART6未启用，其PC6引脚让给PWM输入诊断；此串口通路未完成本轮实车验收。
 * 协议（ASCII 行，\n 结尾）:
 *   V,<vx_mm_s>,<vy_mm_s>,<omega_mrad_s>   设定速度（正=前/左/逆时针）
 *   X                                      立即停止
 * 任何超过 TELEOP_TIMEOUT_MS 没有新命令的情况都按停止处理。
 */
#define TELEOP_TIMEOUT_MS 150U
#define TELEOP_VXY_LIMIT_MM_S 1200
#define TELEOP_OMEGA_LIMIT_MRAD_S 4000
/* 遥控电流指令独立限幅；2457原始单位约3A，低于限时动作路径的4A。 */
#define TELEOP_CURRENT_LIMIT 2457 /* 地面自旋+平移同时受力，提到约 3 A */

typedef struct
{
    int32_t vx_mm_s, vy_mm_s, omega_mrad_s;
    uint32_t last_ms; /* 最近一次有效命令的时间戳 */
    uint32_t rx_bytes, rx_lines, rx_errors;
} TeleopCmd;

void Teleop_Init(void);
/* 在 2 ms 主循环里调用：解析已收到的字节，更新 cmd。返回 1 表示当前有有效命令。 */
int Teleop_Poll(uint32_t now, TeleopCmd *cmd);
/* 供 USB CDC 等其它通道注入原始字节（与串口共用同一解析器）。 */
void Teleop_PushBytes(const char *data, uint32_t len);
/* 每秒在已启用的USART1回一行心跳 "HB,<uptime_ms>,<rx_bytes>\n"，用于验证 TX 通路。 */
void Teleop_Heartbeat(uint32_t now);
/* 主动停止：清空命令并标记为过期。 */
void Teleop_Stop(TeleopCmd *cmd, uint32_t now);
#endif
