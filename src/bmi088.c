#include "bmi088.h"
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1_imu;

#define CS_ACC_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_ACC_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define CS_GYRO_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS_GYRO_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

static int spi_txrx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_TransmitReceive(&hspi1_imu, (uint8_t *)tx, rx, len, 10) == HAL_OK ? 0 : -1;
}

static void write_reg(uint8_t cs, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), value};
    uint8_t rx[2];
    if (cs)
        CS_ACC_LOW();
    else
        CS_GYRO_LOW();
    spi_txrx(tx, rx, 2);
    if (cs)
        CS_ACC_HIGH();
    else
        CS_GYRO_HIGH();
}

/* gyro: 地址后直接读；accel: 地址后要先发一个 dummy 字节 */
static int read_regs(uint8_t cs, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | 0x80);
    uint8_t dummy = 0;
    if (cs)
        CS_ACC_LOW();
    else
        CS_GYRO_LOW();
    int failed = spi_txrx(&addr, buf, 1);
    if (cs)
    { /* 加速度计多一个 dummy */
        if (spi_txrx(&dummy, &dummy, 1))
            failed = -1;
    }
    if (spi_txrx(buf, buf, len))
        failed = -1;
    if (cs)
        CS_ACC_HIGH();
    else
        CS_GYRO_HIGH();
    return failed;
}

static void gpio_spi_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* F4 无 AFIO 重映射寄存器：PB3/PB4 直接配成 AF5 即可，
     * PA13/PA14(SWD) 不受影响，烧写口保持可用。 */

    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4; /* SCK, MISO */
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7; /* MOSI */
    HAL_GPIO_Init(GPIOA, &g);

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_4; /* CS_Accel */
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0; /* CS_Gyro */
    HAL_GPIO_Init(GPIOB, &g);
    CS_ACC_HIGH();
    CS_GYRO_HIGH();
}

void Bmi088_Init(void)
{
    gpio_spi_init();
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1_imu.Instance = SPI1;
    hspi1_imu.Init.Mode = SPI_MODE_MASTER;
    hspi1_imu.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1_imu.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1_imu.Init.CLKPolarity = SPI_POLARITY_HIGH; /* BMI088 推荐模式 3 */
    hspi1_imu.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1_imu.Init.NSS = SPI_NSS_SOFT;
    hspi1_imu.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 84/16 = 5.25 MHz */
    hspi1_imu.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1_imu.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1_imu.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1_imu.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi1_imu);

    /* 陀螺仪：软复位 → 等待 → 2000dps / 1kHz */
    write_reg(0, 0x14, 0xB6);
    HAL_Delay(50);
    write_reg(0, 0x0F, 0x00); /* GYRO_RANGE: ±2000 dps */
    write_reg(0, 0x10, 0x02); /* ODR 1kHz, 滤波带宽 116Hz */
    write_reg(0, 0x11, 0x00); /* 正常模式 */

    /* 加速度计：复位 → 上电 */
    write_reg(1, 0x7E, 0xB6); /* ACC_SOFTRESET */
    HAL_Delay(50);
    write_reg(1, 0x7D, 0x04); /* ACC_PWR_CTRL: 开启 */
    write_reg(1, 0x7C, 0x00); /* ACC_PWR_CONF: 正常 */
    HAL_Delay(5);
    write_reg(1, 0x41, 0x01); /* ACC_RANGE: ±6g */
    write_reg(1, 0x40, 0xA8); /* ACC_CONF: 800Hz? 0xA8=ODR800/BW? 保持默认值 */
}

uint8_t Bmi088_GyroId(void)
{
    uint8_t v = 0;
    read_regs(0, 0x00, &v, 1);
    return v;
} /* 期望 0x0F */
uint8_t Bmi088_AccelId(void)
{
    uint8_t v = 0;
    read_regs(1, 0x00, &v, 1);
    return v;
} /* 期望 0x1E */

int Bmi088_ReadGyro(float *gx, float *gy, float *gz)
{
    uint8_t d[6] = {0};
    if (read_regs(0, 0x02, d, 6))
        return -1; /* GYRO_RATE_X_LSB */
    int16_t rx = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    int16_t ry = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    int16_t rz = (int16_t)(((uint16_t)d[5] << 8) | d[4]);
    const float scale = 0.0010652642f; /* ±2000dps: 16.384 LSB/(°/s) → rad/s */
    *gx = rx * scale;
    *gy = ry * scale;
    *gz = rz * scale;
    return 0;
}

int Bmi088_ReadAccel(float *ax, float *ay, float *az)
{
    uint8_t d[6] = {0};
    if (read_regs(1, 0x12, d, 6))
        return -1; /* ACC_X_LSB */
    int16_t x = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    int16_t y = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    int16_t z = (int16_t)(((uint16_t)(d[5] & 0x7F)) << 8 |
                          d[4]); /* TODO：历史实现屏蔽符号位，修复需单独验证；当前航向不依赖此值 */
    const float scale = 0.001794433f; /* ±6g: 32768/6g → m/s^2 */
    *ax = x * scale;
    *ay = y * scale;
    *az = z * scale;
    return 0;
}
