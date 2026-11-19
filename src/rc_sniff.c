/* 历史接收链路诊断：主程序不调用本模块Init，PC11当前由SBUS UART占用。 */
#include "rc_sniff.h"
#include "stm32f4xx_hal.h"

/* PC11 逻辑分析：EXTI 双边沿 + TIM5(1 µs) 记录 512 个边沿间隔，供离线解码。 */
TIM_HandleTypeDef htim5_sniff;
volatile uint32_t g_sniff[520];
static uint32_t last_cnt;

void RcSniff_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5_sniff.Instance = TIM5;
    htim5_sniff.Init.Prescaler = 83; /* APB1 定时器时钟 84 MHz → 1 MHz */
    htim5_sniff.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5_sniff.Init.Period = 0xFFFFFFFFU;
    htim5_sniff.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5_sniff.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim5_sniff);
    HAL_TIM_Base_Start(&htim5_sniff);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    g.Pin = GPIO_PIN_11;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    g_sniff[0] = 0x534E4646U; /* 'SNFF' */
}

void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != 0x00U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
        uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim5_sniff);
        uint32_t n = g_sniff[1];
        if (n < 512U)
        {
            g_sniff[2 + n] = cnt - last_cnt; /* 与上一个边沿的间隔(µs) */
            g_sniff[1] = n + 1U;
        }
        last_cnt = cnt;
    }
}
