/* 历史接收链路诊断：主程序不调用本模块Init，PC11当前由SBUS UART占用。 */
#include "rc_dbus.h"
#include "stm32f4xx_hal.h"

/* PC11 = DBUS 口反相后的信号。EXTI 双边沿 + TIM5 1 MHz 时间戳测低电平宽度。 */
TIM_HandleTypeDef htim5_ts;

static volatile uint32_t fall_cnt;
static volatile uint32_t rise_cnt;
static volatile uint16_t low_us;  /* PC11 低电平宽度 = 接收机高脉冲宽度 */
static volatile uint16_t high_us; /* PC11 高电平宽度（诊断用） */
static volatile uint32_t edge_ms;
static volatile uint8_t have_fall;
volatile uint32_t g_dbus_edges;   /* 诊断：PC11 边沿总数 */
volatile uint16_t g_dbus_high_us; /* 诊断：PC11 高电平宽度 */
/* 诊断缓冲：记录最近 12 个边沿间隔（µs，交替为高/低），用于看清真实波形。 */
volatile uint32_t g_rc_diag[16] = {0x52434431U, 0U}; /* 'RCD1', 边沿序号 */

void RcDbus_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1 MHz 自由计数的时间基准（TIM5 为 32 位） */
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5_ts.Instance = TIM5;
    htim5_ts.Init.Prescaler = 83; /* APB1 定时器时钟 84 MHz → 1 MHz */
    htim5_ts.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5_ts.Init.Period = 0xFFFFFFFFU;
    htim5_ts.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5_ts.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim5_ts);
    HAL_TIM_Base_Start(&htim5_ts);

    /* PC11 输入 + EXTI 双边沿 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

#if 0 /* 诊断用：与 rc_sniff 的 EXTI 处理冲突，默认关闭 */
static uint32_t last_edge_cnt;
void EXTI15_10_IRQHandler(void)
{
    uint32_t now_cnt = __HAL_TIM_GET_COUNTER(&htim5_ts);
    uint32_t now_ms = HAL_GetTick();
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != 0x00U) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
        g_dbus_edges++;
        {
            uint32_t interval = now_cnt - last_edge_cnt;
            last_edge_cnt = now_cnt;
            uint32_t idx = g_rc_diag[1] % 12U;
            g_rc_diag[2 + idx] = interval;
            g_rc_diag[1]++;
        }
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET) {
            /* 下降沿：PC11 开始低电平（接收机脉冲开始） */
            fall_cnt = now_cnt;
            if (have_fall) {
                uint16_t high = (uint16_t)(fall_cnt - rise_cnt);
                if (high >= 200U && high <= 25000U) { high_us = high; g_dbus_high_us = high; }
            }
            have_fall = 1U;
        } else {
            /* 上升沿：低电平结束，低电平宽度即接收机高脉冲宽度 */
            if (have_fall) {
                uint16_t low = (uint16_t)(now_cnt - fall_cnt);
                if (low >= 300U && low <= 3000U) {
                    low_us = low;
                    edge_ms = now_ms;
                }
            }
            rise_cnt = now_cnt;
        }
    }
}

#endif

uint16_t RcDbus_Read(uint32_t now)
{
    uint16_t value = low_us;
    if (value == 0U || (uint32_t)(now - edge_ms) > RC_DBUS_TIMEOUT_MS)
        return 0U;
    return value;
}
