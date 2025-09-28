//
// Created by aM on 25-6-8.
//
#include "stm32f4xx.h"
#include "OLED.h"
#include "icm42688.h"
#include "main.h"
#include "PID.h"
#include "Motor.h"

euler_angles_t car_angles;
extern icm42688_data_t sensor;
extern PID_t pidAngle;
extern PID_t pidSpeed;

// void Task1(void)
// {
//     static uint16_t Count1 = 0;
//     Count1++;
//     if (Count1 > 1000)
//     {
//         Count1 = 0;
//     }
//     else if (Count1 > 500)
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
//     }
//     else
//     {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//     }
// }
void Task_OLED(void)
{
   //icm42688结构体
    OLED_Printf(0, 8,OLED_6X8,"PS= %.2f", pidSpeed.Out);

    OLED_Printf(0, 48, OLED_6X8, "Pitch:%.1f", car_angles.pitch);
    OLED_Printf(64, 48, OLED_6X8, "Roll:%.1f", car_angles.roll);
    OLED_Printf(0, 56, OLED_6X8, "Roll:%.1f", car_angles.yaw);
    OLED_Update();
}

void Task_Attitude(void)
{
    OLED_Printf(0, 16, OLED_6X8, "L:%d R:%d", Encoder_GetLeft(), Encoder_GetRight());
    OLED_Printf(0, 24, OLED_6X8, "S:%.2f T:%.2f", pidSpeed.Actual, pidSpeed.Target);
}