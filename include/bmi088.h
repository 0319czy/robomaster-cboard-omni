#ifndef BMI088_H
#define BMI088_H
#include <stdint.h>

/* C 板板载 BMI088：SPI1(PB3=CLK, PB4=MISO, PA7=MOSI)，CS_Accel=PA4，CS_Gyro=PB0。
 * 注意 PB3/PB4 属 JTAG 引脚，初始化时把 SWJ 配置为"仅 SWD"。 */
#define BMI088_GYRO_RANGE_2000DPS 1
#define BMI088_ACC_RANGE_6G 1

void Bmi088_Init(void);
uint8_t Bmi088_GyroId(void);
uint8_t Bmi088_AccelId(void);
/* 返回 0 成功。单位: 角速度 rad/s，加速度 m/s^2 */
int Bmi088_ReadGyro(float *gx, float *gy, float *gz);
int Bmi088_ReadAccel(float *ax, float *ay, float *az);
#endif
