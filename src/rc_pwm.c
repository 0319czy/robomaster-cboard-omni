#include "rc_pwm.h"
#include "stm32f4xx_hal.h"

/* TIM1(CH1-4) + TIM8(CH1-3) 输入捕获读遥控 PWM。
 * 频率 1 MHz（PSC=167），每个通道在上升沿记时间、翻转到下降沿算脉宽，再翻回去。 */
TIM_HandleTypeDef htim1_rc, htim8_rc;

static volatile uint16_t rise_us[RC_CHANNELS];
static volatile uint16_t width_us[RC_CHANNELS];
static volatile uint32_t edge_ms[RC_CHANNELS];

static const uint32_t tim1_channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                          TIM_CHANNEL_4};
static const uint32_t tim8_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

/* CCER 里的极性位：切边沿用。 */
static void toggle_polarity(TIM_TypeDef *tim, uint32_t channel)
{
    uint32_t mask;
    switch (channel)
    {
        case TIM_CHANNEL_1:
            mask = TIM_CCER_CC1P;
            break;
        case TIM_CHANNEL_2:
            mask = TIM_CCER_CC2P;
            break;
        case TIM_CHANNEL_3:
            mask = TIM_CCER_CC3P;
            break;
        case TIM_CHANNEL_4:
            mask = TIM_CCER_CC4P;
            break;
        default:
            return;
    }
    tim->CCER ^= mask;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t index;
    uint32_t value = HAL_TIM_ReadCapturedValue(htim, htim->Channel);
    if (htim->Instance == TIM1)
    {
        index = (htim->Channel == TIM_CHANNEL_1)   ? 0U
                : (htim->Channel == TIM_CHANNEL_2) ? 1U
                : (htim->Channel == TIM_CHANNEL_3) ? 2U
                                                   : 3U;
    }
    else if (htim->Instance == TIM8)
    {
        index = (htim->Channel == TIM_CHANNEL_1) ? 4U : (htim->Channel == TIM_CHANNEL_2) ? 5U : 6U;
    }
    else
    {
        return;
    }
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) && 0)
    {
    }
    /* 极性为 0 = 正在等上升沿 → 记下上升时刻；否则是下降沿 → 算宽度。 */
    uint32_t ccer_bit = (htim->Channel == TIM_CHANNEL_1)   ? (htim->Instance->CCER & TIM_CCER_CC1P)
                        : (htim->Channel == TIM_CHANNEL_2) ? (htim->Instance->CCER & TIM_CCER_CC2P)
                        : (htim->Channel == TIM_CHANNEL_3) ? (htim->Instance->CCER & TIM_CCER_CC3P)
                                                           : (htim->Instance->CCER & TIM_CCER_CC4P);
    if (ccer_bit == 0U)
    {
        rise_us[index] = (uint16_t)value;
        toggle_polarity(htim->Instance, htim->Channel);
    }
    else
    {
        uint16_t width = (uint16_t)(value - rise_us[index]);
        if (width >= RC_MIN_US && width <= RC_MAX_US)
        {
            width_us[index] = width;
            edge_ms[index] = HAL_GetTick();
        }
        toggle_polarity(htim->Instance, htim->Channel);
    }
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_6;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOI, &g);
}

static void timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    TIM_IC_InitTypeDef ic = {0};
    htim->Instance = instance;
    htim->Init.Prescaler = 167; /* 168 MHz / 168 = 1 MHz，1 µs 分辨率 */
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(htim);
    ic.ICPolarity = TIM_ICPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter = 0x0F; /* 最强滤波，抑制遥控线噪声 */
    for (uint32_t i = 0; i < 4U; i++)
    {
        if (instance == TIM1 || i < 3U)
            HAL_TIM_IC_ConfigChannel(htim, &ic,
                                     (instance == TIM1) ? tim1_channels[i] : tim8_channels[i]);
    }
}

void RcPwm_Init(void)
{
    gpio_init();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    timer_init(&htim1_rc, TIM1);
    timer_init(&htim8_rc, TIM8);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    for (uint32_t i = 0; i < 4U; i++)
        HAL_TIM_IC_Start_IT(&htim1_rc, tim1_channels[i]);
    for (uint32_t i = 0; i < 3U; i++)
        HAL_TIM_IC_Start_IT(&htim8_rc, tim8_channels[i]);
}

uint32_t RcPwm_Read(uint32_t now, uint16_t out[RC_CHANNELS])
{
    uint32_t valid = 0;
    for (uint32_t i = 0; i < RC_CHANNELS; i++)
    {
        uint16_t value = width_us[i];
        if (value && (uint32_t)(now - edge_ms[i]) <= RC_TIMEOUT_MS)
        {
            out[i] = value;
            valid++;
        }
        else
        {
            out[i] = 0;
        }
    }
    return valid;
}

void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1_rc);
}
void TIM8_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim8_rc);
}
