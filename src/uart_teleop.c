#include "uart_teleop.h"
#include "stm32f4xx_hal.h"
#include "rc_serial.h"
#include <string.h>
#include <stdio.h>

/* 备用ASCII接收器：当前只初始化USART1（PA9 TX / PB7 RX）。
 * USART6的回调分支保留用于历史诊断，但PC6当前用于PWM捕获，不能同时启用。
 * SBUS由独立USART3模块处理，不经过本模块的ASCII解析器。 */
UART_HandleTypeDef huart1_teleop, huart6_teleop;

#define RING_SIZE 128U /* 必须是 2 的幂 */
static volatile uint8_t ring[RING_SIZE];
static volatile uint32_t head, tail;
static uint8_t rx_byte1, rx_byte6;
static char line[48];
static uint32_t line_len;
static uint32_t rx_bytes_total, rx_lines_total, rx_error_total;
static uint32_t last_echoed;
static int32_t cmd_vx_echo, cmd_vy_echo, cmd_omega_echo;

static int32_t clamp_int(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void Push(char c)
{
    uint32_t next = (head + 1U) & (RING_SIZE - 1U);
    if (next != tail)
    {
        ring[head] = (uint8_t)c;
        head = next;
    }
    rx_bytes_total++;
}

void Teleop_PushBytes(const char *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        Push(data[i]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        RcSerial_OnRxComplete();
        return;
    }
    if (huart->Instance == USART6)
    {
        Push((char)rx_byte6);
        HAL_UART_Receive_IT(&huart6_teleop, &rx_byte6, 1);
    }
    else if (huart->Instance == USART1)
    {
        Push((char)rx_byte1);
        HAL_UART_Receive_IT(&huart1_teleop, &rx_byte1, 1);
    }
}

static void InitOne(UART_HandleTypeDef *h, USART_TypeDef *inst, uint32_t baud)
{
    h->Instance = inst;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(h) != HAL_OK)
    {
        while (1)
        {
        }
    }
}

void Teleop_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PC6 已让给 TIM8_CH1 做遥控输入捕获，这里不再启用 USART6。 */

    /* USART1: PA9 TX / PB7 RX */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);
    InitOne(&huart1_teleop, USART1, 115200);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1_teleop, &rx_byte1, 1);
}

void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6_teleop);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
        RcSerial_OnError();
}
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1_teleop);
}

/* 解析V,vx,vy,omega或X。非法行计数并丢弃，不刷新有效命令时间；
 * 上一条有效命令最多维持TELEOP_TIMEOUT_MS，然后按失联停车。 */
static void ParseLine(const char *text, uint32_t len, uint32_t now, TeleopCmd *cmd)
{
    if (len == 0U)
        return;
    rx_lines_total++;
    if (text[0] == 'X' || text[0] == 'x')
    {
        cmd->vx_mm_s = 0;
        cmd->vy_mm_s = 0;
        cmd->omega_mrad_s = 0;
        cmd->last_ms = now;
        return;
    }
    if (text[0] != 'V' && text[0] != 'v')
    {
        rx_error_total++;
        return;
    }
    long values[3];
    int count = 0;
    uint32_t i = 1;
    while (i < len && count < 3)
    {
        while (i < len && (text[i] == ',' || text[i] == ' '))
            i++;
        int sign = 1;
        if (i < len && (text[i] == '-' || text[i] == '+'))
        {
            if (text[i] == '-')
                sign = -1;
            i++;
        }
        long value = 0;
        uint32_t digits = 0;
        while (i < len && text[i] >= '0' && text[i] <= '9' && digits < 8U)
        {
            value = value * 10 + (text[i] - '0');
            i++;
            digits++;
        }
        if (digits == 0U)
        {
            rx_error_total++;
            return;
        }
        values[count++] = sign * value;
    }
    if (count != 3)
    {
        rx_error_total++;
        return;
    }
    cmd->vx_mm_s = clamp_int((int32_t)values[0], -TELEOP_VXY_LIMIT_MM_S, TELEOP_VXY_LIMIT_MM_S);
    cmd->vy_mm_s = clamp_int((int32_t)values[1], -TELEOP_VXY_LIMIT_MM_S, TELEOP_VXY_LIMIT_MM_S);
    cmd->omega_mrad_s =
        clamp_int((int32_t)values[2], -TELEOP_OMEGA_LIMIT_MRAD_S, TELEOP_OMEGA_LIMIT_MRAD_S);
    cmd_vx_echo = cmd->vx_mm_s;
    cmd_vy_echo = cmd->vy_mm_s;
    cmd_omega_echo = cmd->omega_mrad_s;
    cmd->last_ms = now;
}

void Teleop_Stop(TeleopCmd *cmd, uint32_t now)
{
    cmd->vx_mm_s = 0;
    cmd->vy_mm_s = 0;
    cmd->omega_mrad_s = 0;
    cmd->last_ms = now - TELEOP_TIMEOUT_MS - 1U;
}

int Teleop_Poll(uint32_t now, TeleopCmd *cmd)
{
    while (tail != head)
    {
        char c = (char)ring[tail];
        tail = (tail + 1U) & (RING_SIZE - 1U);
        if (c == '\n' || c == '\r')
        {
            ParseLine(line, line_len, now, cmd);
            line_len = 0;
        }
        else if (line_len < sizeof(line) - 1U)
        {
            line[line_len++] = c;
        }
        else
        {
            line_len = 0;
            rx_error_total++;
        }
    }
    cmd->rx_bytes = rx_bytes_total;
    cmd->rx_lines = rx_lines_total;
    cmd->rx_errors = rx_error_total;
    return (uint32_t)(now - cmd->last_ms) <= TELEOP_TIMEOUT_MS;
}

void Teleop_Heartbeat(uint32_t now)
{
    static uint32_t last;
    if ((uint32_t)(now - last) < 1000U)
        return;
    last = now;
    char buf[48];
    int n = snprintf(buf, sizeof buf, "HB,%lu,%lu\n", (unsigned long)now,
                     (unsigned long)rx_bytes_total);
    if (n > 0)
    {
        HAL_UART_Transmit(&huart1_teleop, (uint8_t *)buf, (uint16_t)n, 20);
    }
    /* 收到有效行也回显，便于确认上行通路。 */
    if (rx_lines_total != last_echoed)
    {
        last_echoed = rx_lines_total;
        int m = snprintf(buf, sizeof buf, "RX,%lu,%ld,%ld,%ld\n", (unsigned long)rx_lines_total,
                         (long)cmd_vx_echo, (long)cmd_vy_echo, (long)cmd_omega_echo);
        if (m > 0)
        {
            HAL_UART_Transmit(&huart1_teleop, (uint8_t *)buf, (uint16_t)m, 20);
        }
    }
}
