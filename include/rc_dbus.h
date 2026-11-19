#ifndef RC_DBUS_H
#define RC_DBUS_H
#include <stdint.h>

/* C 板 DBUS 口（J16 的 C8 → PC11）经 Q10 反相器接到 MCU：
 * 接收机的正向 PWM（空闲低、高脉冲 1000~2000 µs）在 PC11 上表现为
 * 空闲高、低脉冲等宽，因此这里测量 PC11 的低电平宽度。
 * 用 EXTI 双边沿 + TIM5(1 MHz) 时间戳，不占用 UART3。
 */
#define RC_DBUS_TIMEOUT_MS 150U

void RcDbus_Init(void);
/* 返回 BUS 引脚上测得的高脉冲宽度（µs）；超时或无效返回 0。 */
uint16_t RcDbus_Read(uint32_t now);
#endif
