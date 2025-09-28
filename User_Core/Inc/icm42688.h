//
// Created by aM on 25-6-4.
//
#ifndef ICM42688_H
#define ICM42688_H

#include "stm32f4xx.h"
#define ICM42688_I2C_ADDR (0x69 << 1)

#define I2C_TimeOut_Ms 100

/* 常用寄存器地址（Bank 0）*/
#define ICM42688_REG_WHO_AM_I         0x75  // 只读，复位值 0x47
#define ICM42688_REG_DEVICE_CONFIG    0x11  // 软件复位：bit0 = 1
#define ICM42688_REG_INT_CONFIG       0x14
#define ICM42688_REG_PWR_MGMT0        0x4E
#define ICM42688_REG_GYRO_CONFIG0     0x4F
#define ICM42688_REG_ACCEL_CONFIG0    0x50
#define ICM42688_REG_GYRO_CONFIG1     0x51
#define ICM42688_REG_ACCEL_CONFIG1    0x52

/* 原始数据寄存器 */
#define ICM42688_REG_TEMP_DATA1       0x1D
#define ICM42688_REG_TEMP_DATA0       0x1E
#define ICM42688_REG_ACCEL_XOUT_H     0x1F
#define ICM42688_REG_ACCEL_XOUT_L     0x20
#define ICM42688_REG_ACCEL_YOUT_H     0x21
#define ICM42688_REG_ACCEL_YOUT_L     0x22
#define ICM42688_REG_ACCEL_ZOUT_H     0x23
#define ICM42688_REG_ACCEL_ZOUT_L     0x24
#define ICM42688_REG_GYRO_XOUT_H      0x25
#define ICM42688_REG_GYRO_XOUT_L      0x26
#define ICM42688_REG_GYRO_YOUT_H      0x27
#define ICM42688_REG_GYRO_YOUT_L      0x28
#define ICM42688_REG_GYRO_ZOUT_H      0x29
#define ICM42688_REG_GYRO_ZOUT_L      0x2A

/* Bank 切换寄存器（在所有 Bank 都可访问） */
#define ICM42688_REG_BANK_SEL         0x76

typedef enum
{
    ICM42688_OK = 0,
    ICM42688_ERR_I2C = -1,
    ICM42688_ERR_WHOAMI = -2,
    ICM42688_ERR_CONFIG = -3,
}icm42688_status_t;

/* 传感器数据结构 */
typedef struct {
    float ax;    // 单位 g
    float ay;    // 单位 g
    float az;    // 单位 g
    float gx;    // 单位 dps (°/s)
    float gy;    // 单位 dps
    float gz;    // 单位 dps
    float temp;  // 单位 °C
} icm42688_data_t;

// 欧拉角结构体
typedef struct {
    float roll;      // 横滚角(度)
    float pitch;    // 俯仰角(度)
    float yaw;      // 偏航角(度)
} euler_angles_t;

icm42688_status_t ICM42688_ReadReg(uint8_t reg, uint8_t *pData);
icm42688_status_t ICM42688_WriteReg(uint8_t reg, uint8_t Data);
icm42688_status_t ICM42688_CheckID(void);
icm42688_status_t ICM42688_Init(void);
icm42688_status_t ICM42688_Get6Axis(icm42688_data_t *data);
// 初始化姿态解算
void Attitude_Init(void);
// 互补滤波姿态解算
void Complementary_Filter(const icm42688_data_t *imu, euler_angles_t *angles);

#endif //SOFTIIC_H
