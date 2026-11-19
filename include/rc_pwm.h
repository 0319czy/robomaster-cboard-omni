#ifndef RC_PWM_H
#define RC_PWM_H
#include <stdint.h>

/* C 板 PWM 接口 J16 的信号脚就是定时器通道，可直接做输入捕获读遥控 PWM：
 *   C1..C4 = TIM1_CH1..4 → PE9 / PE11 / PE13 / PE14
 *   C5..C7 = TIM8_CH1..3 → PC6 / PI6 / PI7
 * 通道数组下标 0..6 对应 C1..C7。脉宽单位 µs；超时或未接线的通道为 0。
 */
#define RC_CHANNELS 7
#define RC_TIMEOUT_MS 120U
#define RC_MIN_US 800U
#define RC_MAX_US 2200U

void RcPwm_Init(void);
/* 读出各通道脉宽；now 用于超时判定。返回当前有效通道数（0..7）。 */
uint32_t RcPwm_Read(uint32_t now, uint16_t width_us[RC_CHANNELS]);
#endif
