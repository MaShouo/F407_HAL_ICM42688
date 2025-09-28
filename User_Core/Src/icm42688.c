//
// Created by aM on 25-6-4.
//

#include "stm32f4xx_hal.h"
#include "icm42688.h"
#include "i2c.h"
#include "icm42688.h"
#include "math.h"
#include "Task.h"

#define COMPLEMENTARY_FILTER_ALPHA 0.98f
#define RAD2DEG 57.295779513f
#define DEG2RAD 0.01745329252f

static uint32_t prev_tick = 0;
static float gx_offset = 0, gy_offset = 0, gz_offset = 0;
static int8_t first_update = 1;

/**
   HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                    uint16_t DevAddress,
                    uint16_t MemAddress,
                    uint16_t MemAddSize,
                    uint8_t *pData, uint16_t Size,
                    uint32_t Timeout);
   第1个参数为I2C操作句柄
   第2个参数为从机设备地址
   第3个参数为从机寄存器地址
   第4个参数为从机寄存器地址长度
   第5个参数为发送的数据的起始地址
   第6个参数为传输数据的大小
   第7个参数为操作超时时间 　　
**/

/**
 *@brief 读取多个数据3
 *@param reg ： 写入地址
 *@param pData ：发送数据的地址
 *@param len ： 数据大小
 **/

static icm42688_status_t ICM42688_ReadBytes(uint8_t reg, uint8_t *pData, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1,
                        ICM42688_I2C_ADDR,
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        pData,
                        len,
                        I2C_TimeOut_Ms) != HAL_OK)
    {
        return ICM42688_ERR_I2C;
    }
    return ICM42688_OK;
}

static icm42688_status_t ICM42688_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c1,
                        ICM42688_I2C_ADDR,
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        pData,
                        len,
                        I2C_TimeOut_Ms) != HAL_OK)
    {
        return ICM42688_ERR_I2C;
    }
    return ICM42688_OK;
}

/**
 *@brief 读取单个寄存器
 *@param reg ：寄存器地址
 *@param pData ：读取数据的首地址
 **/
icm42688_status_t ICM42688_ReadReg(uint8_t reg, uint8_t *pData)
{
    return ICM42688_ReadBytes(reg, pData, 1);
}

icm42688_status_t ICM42688_WriteReg(uint8_t reg, uint8_t Data)
{
    return ICM42688_WriteBytes(reg, &Data, 1);
}

icm42688_status_t ICM42688_CheckID(void)
{
    uint8_t ID = 0;
    if (ICM42688_ReadReg(ICM42688_REG_WHO_AM_I, &ID) != ICM42688_OK)
    {
        return ICM42688_ERR_I2C;
    }
    if (ID != 0x47)
    {
        return ICM42688_ERR_WHOAMI;
    }
    return ICM42688_OK;
}

/** 软件复位 **/
static icm42688_status_t ICM42688_SoftReset(void)
{
    if (ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, 0x01) != ICM42688_OK)
    {
        return ICM42688_ERR_I2C;
    }
    HAL_Delay(100);
    return ICM42688_OK;
}

/** Bank选择 **/
static icm42688_status_t ICM42688_SelectBank(uint8_t bank)
{
    if (bank > 4) return ICM42688_ERR_CONFIG;
    return ICM42688_WriteReg(ICM42688_REG_BANK_SEL, bank);
}

/** 配置加速度计 **/
static icm42688_status_t ICM42688_ConfigAccel(void)
{
    //保证处于Bank0
    if (ICM42688_SelectBank(0) != ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }

    if (ICM42688_WriteReg(ICM42688_REG_ACCEL_CONFIG0, 0x66) != ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }
    return ICM42688_OK;
}

/** 配置陀螺仪 **/
static icm42688_status_t ICM42688_ConfigGyro(void)
{
    if (ICM42688_SelectBank(0) != ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }

    if (ICM42688_WriteReg(ICM42688_REG_GYRO_CONFIG0, 0x06) != ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }
    return ICM42688_OK;
}

/** 配置电源选项 **/
static icm42688_status_t ICM42688_ConfigPWR(void)
{
    if (ICM42688_WriteReg(ICM42688_REG_PWR_MGMT0, 0x2F) != ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }
    return ICM42688_OK;
}

/** ICM42688初始化 **/
icm42688_status_t ICM42688_Init(void)
{
    if (ICM42688_CheckID() != ICM42688_OK)
    {
        return ICM42688_ERR_WHOAMI;
    }

    if (ICM42688_SoftReset() != ICM42688_OK)
    {
        return ICM42688_ERR_I2C;
    }

    if (ICM42688_SelectBank(0) != ICM42688_OK)
    {
        return ICM42688_ERR_I2C;
    }

    if (ICM42688_ConfigAccel()!= ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }

    if (ICM42688_ConfigGyro()!= ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }

    if (ICM42688_ConfigPWR()!= ICM42688_OK)
    {
        return ICM42688_ERR_CONFIG;
    }
    HAL_Delay(100);

    return ICM42688_OK;
}

icm42688_status_t ICM42688_Get6Axis(icm42688_data_t *data)
{
    uint8_t data_bytes[12];
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    if (ICM42688_ReadBytes(ICM42688_REG_ACCEL_XOUT_H, data_bytes, 12) != ICM42688_OK)
    {
        return ICM42688_ERR_I2C;
    }

    raw_ax = (int16_t)((data_bytes[0] << 8) | data_bytes[1]);
    raw_ay = (int16_t)((data_bytes[2] << 8) | data_bytes[3]);
    raw_az = (int16_t)((data_bytes[4] << 8) | data_bytes[5]);
    raw_gx = (int16_t)((data_bytes[6] << 8) | data_bytes[7]);
    raw_gy = (int16_t)((data_bytes[8] << 8) | data_bytes[9]);
    raw_gz = (int16_t)((data_bytes[10] << 8) | data_bytes[11]);

    float accel_scale = 1.0f / 16384.0f;
    float gyro_scale = 1.0f / 16.4f;

    data->ax = (float)raw_ax * accel_scale;
    data->ay = (float)raw_ay * accel_scale;
    data->az = (float)raw_az * accel_scale;

    /* 陀螺仪换算 (°/s) */
    data->gx = raw_gx * gyro_scale - 1.5;
    data->gy = raw_gy * gyro_scale + 9;
    data->gz = raw_gz * gyro_scale - 1.2;

    return ICM42688_OK;
}

// 初始化：校准偏置、设置初始角度
void Attitude_Init(void)
{
    prev_tick = HAL_GetTick();
    // 偏置校准：确保设备静止且水平放置
    icm42688_data_t imu;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int samples = 100;
    HAL_Delay(50);
    for (int i = 0; i < samples; i++) {
        if (ICM42688_Get6Axis(&imu) == ICM42688_OK) {
            sum_gx += imu.gx;
            sum_gy += imu.gy;
            sum_gz += imu.gz;
        }
        HAL_Delay(5);
    }
    gx_offset = sum_gx / samples;
    gy_offset = sum_gy / samples;
    gz_offset = sum_gz / samples;
    // 初始化角度：根据静止时加速度可做一次初始化，也可直接设0
    car_angles.roll  = 0.0f;
    car_angles.pitch = 0.0f;
    car_angles.yaw   = 0.0f;
    first_update = 1;
}

// 互补滤波更新，angles 指向 car_angles
void Complementary_Filter(const icm42688_data_t *imu, euler_angles_t *angles)
{
    uint32_t cur = HAL_GetTick();
    float dt = (cur - prev_tick) * 0.001f; // s
    if (dt <= 0) {
        // 时间异常，跳过
        return;
    }
    prev_tick = cur;

    // 若首次更新或 dt 过大（如刚启动/任务延迟），仅做加速度初始化
    if (first_update || dt > 0.1f) {
        // 归一化加速度
        float acc_norm = sqrtf(imu->ax*imu->ax + imu->ay*imu->ay + imu->az*imu->az);
        if (acc_norm < 0.1f) {
            // 数据异常，跳过
            return;
        }
        float ax = imu->ax / acc_norm;
        float ay = imu->ay / acc_norm;
        float az = imu->az / acc_norm;
        // 计算弧度姿态
        float acc_roll_rad  = atan2f(ay, az);
        float acc_pitch_rad = atan2f(-ax, sqrtf(ay*ay + az*az));
        angles->roll  = acc_roll_rad * RAD2DEG;
        angles->pitch = acc_pitch_rad * RAD2DEG;
        angles->yaw   = 0.0f;
        first_update = 0;
        return;
    }

    // 归一化加速度
    float acc_norm = sqrtf(imu->ax*imu->ax + imu->ay*imu->ay + imu->az*imu->az);
    if (acc_norm < 0.1f) {
        return;
    }
    float ax = imu->ax / acc_norm;
    float ay = imu->ay / acc_norm;
    float az = imu->az / acc_norm;
    // 加速度估计角度（弧度）
    float acc_roll_rad  = atan2f(ay, az);
    float acc_pitch_rad = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 角速度去偏置并转为弧度/s
    float gx = (imu->gx - gx_offset) * DEG2RAD;
    float gy = (imu->gy - gy_offset) * DEG2RAD;
    float gz = (imu->gz - gz_offset) * DEG2RAD;

    // 上次角度转弧度
    float last_roll_rad  = angles->roll  * DEG2RAD;
    float last_pitch_rad = angles->pitch * DEG2RAD;
    float last_yaw_rad   = angles->yaw   * DEG2RAD;

    // 陀螺积分（弧度）
    float gyro_roll_rad  = last_roll_rad  + gx * dt;
    float gyro_pitch_rad = last_pitch_rad + gy * dt;
    float gyro_yaw_rad   = last_yaw_rad   + gz * dt;

    // 互补融合
    float fused_roll_rad  = COMPLEMENTARY_FILTER_ALPHA * gyro_roll_rad
                           + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * acc_roll_rad;
    float fused_pitch_rad = COMPLEMENTARY_FILTER_ALPHA * gyro_pitch_rad
                           + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * acc_pitch_rad;
    float fused_yaw_rad   = gyro_yaw_rad; // 无磁力计校正，仅积分

    // 转回度并存储
    angles->roll  = fused_roll_rad  * RAD2DEG;
    angles->pitch = fused_pitch_rad * RAD2DEG;
    angles->yaw   = fused_yaw_rad   * RAD2DEG;
}
