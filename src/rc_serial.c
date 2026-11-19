#include "rc_serial.h"
#include "stm32f4xx_hal.h"

/* DBUS 口经板载反相器接 PC11 = USART3_RX。实测该引脚上是 SBUS：
 * 100 kbaud、偶校验、2 停止位（8E2），帧头 0x0F，25 字节/帧，16 通道 11 bit。
 * 使用 HAL 完成字长/校验配置和中断接收；错误帧不进入控制链。 */
volatile uint32_t g_rc_serial_bytes;
volatile uint32_t g_rc_serial_errors;
volatile uint8_t g_rc_raw[256];
volatile uint32_t g_sbus[20]; /* [0]=magic [1]=帧数 [2..17]=ch1..16 [18]=flags [19]=错误 */
/* 通道行程记录：[0]=magic [1]=帧数 [2..17]=最小值 [18..33]=最大值 */
volatile uint32_t g_sbus_range[34];
static uint8_t range_init;
static uint32_t raw_index;
static uint8_t frame[25];
static uint32_t frame_len, last_byte_ms;
static UART_HandleTypeDef huart3_rc;
static uint8_t rx_byte;
volatile uint32_t g_rc_edges_dbg;
volatile uint32_t g_sbus_last_ms;

void RcSerial_Init(uint32_t baud, uint32_t parity)
{
    (void)parity; /* SBUS 固定 8E2 */
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Pin = GPIO_PIN_11;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &g);

    /* F4 开校验后校验位占用字长的最高位；8 数据位 + 偶校验必须设 9B。
     * 原来的 M=0 实际是 7E2，可能把奇偶校验位混进 SBUS 数据。 */
    huart3_rc.Instance = USART3;
    huart3_rc.Init.BaudRate = baud;
    huart3_rc.Init.WordLength = UART_WORDLENGTH_9B;
    huart3_rc.Init.StopBits = UART_STOPBITS_2;
    huart3_rc.Init.Parity = UART_PARITY_EVEN;
    huart3_rc.Init.Mode = UART_MODE_RX;
    huart3_rc.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3_rc.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3_rc) != HAL_OK)
        return;
    HAL_UART_Receive_IT(&huart3_rc, &rx_byte, 1);
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    g_sbus[0] = 0x53425553U;       /* 'SBUS' */
    g_sbus_range[0] = 0x52414E47U; /* 'RANG' */
}

static void ParseFrame(void)
{
    uint32_t ch[16];
    /* 标准 Futaba SBUS：16 通道 × 11 bit 连续打包在 frame[1..22]。 */
    for (int i = 0; i < 16; i++)
    {
        uint32_t bit = (uint32_t)i * 11U;
        uint32_t byte = bit / 8U;
        uint32_t shift = bit % 8U;
        uint32_t v = (uint32_t)frame[1 + byte] | ((uint32_t)frame[2 + byte] << 8) |
                     ((uint32_t)frame[3 + byte] << 16);
        ch[i] = (v >> shift) & 0x07FFU;
    }
    for (int i = 0; i < 16; i++)
        g_sbus[2 + i] = ch[i];
    if (!range_init)
    {
        for (int i = 0; i < 16; i++)
        {
            g_sbus_range[2 + i] = ch[i];
            g_sbus_range[18 + i] = ch[i];
        }
        range_init = 1;
    }
    for (int i = 0; i < 16; i++)
    {
        if (ch[i] < g_sbus_range[2 + i])
            g_sbus_range[2 + i] = ch[i];
        if (ch[i] > g_sbus_range[18 + i])
            g_sbus_range[18 + i] = ch[i];
    }
    g_sbus_range[1]++;
    g_sbus[18] = frame[23];
    g_sbus[1]++;
    g_sbus_last_ms = HAL_GetTick();
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3_rc);
}

void RcSerial_OnRxComplete(void)
{
    uint8_t byte = rx_byte;
    /* 先重新挂接收，缩短逐字节接收的空窗；25 字节解码在后面完成。 */
    /* 错误字节由 HAL 随后的 ErrorCallback 重新挂接，不得清掉 ErrorCode 再解析。 */
    if (huart3_rc.ErrorCode != HAL_UART_ERROR_NONE)
    {
        frame_len = 0;
        return;
    }
    HAL_UART_Receive_IT(&huart3_rc, &rx_byte, 1);
    g_rc_raw[raw_index] = byte;
    raw_index = (raw_index + 1U) % 256U;
    g_rc_serial_bytes++;
    g_rc_edges_dbg++;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_byte_ms) > 3U)
        frame_len = 0;
    last_byte_ms = now;
    if (!frame_len)
    {
        if (byte == 0x0fU)
            frame[frame_len++] = byte;
        return;
    }
    frame[frame_len++] = byte;
    if (frame_len == 25U)
    {
        /* RF206S 使用标准 SBUS 尾字节 0x00；坏帧不刷新有效帧时间戳。
         * 校验尾字节也可防止线缆插拔后错位拼成油门命令。 */
        if (frame[24] == 0U && (frame[23] & 0xf0U) == 0U)
            ParseFrame();
        else
            g_sbus[19]++;
        frame_len = 0;
    }
}

void RcSerial_OnError(void)
{
    g_rc_serial_errors++;
    g_sbus[19]++;
    frame_len = 0;
    /* HAL 对 ORE 会结束接收，对 PE/FE 则可能保持 BUSY_RX；只重挂已结束的接收。 */
    if (huart3_rc.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(&huart3_rc, &rx_byte, 1);
}
void RcSerial_ScanStep(uint32_t now)
{
    (void)now;
}
