//
// Created by aM on 25-6-8.
//
#include "stm32f4xx.h"
#include "tim.h"

void Motor_SetLeftCompare(uint16_t LeftCompare)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, LeftCompare);
}

void Motor_SetRightCompare(uint16_t RightCompare)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, RightCompare);
}

int16_t Encoder_GetLeft(void)
{
    int16_t Temp;
    Temp = __HAL_TIM_GetCounter(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return Temp;
}

int16_t Encoder_GetRight(void)
{
    int16_t Temp;
    Temp = __HAL_TIM_GetCounter(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return Temp;
}

void Motor_SetLeftPWM(int16_t LeftPWM)
{
    if (LeftPWM < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        Motor_SetLeftCompare(-LeftPWM);
    }
    else if (LeftPWM > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        Motor_SetLeftCompare(LeftPWM);
    }
}

void Motor_SetRightPWM(int16_t RightPWM)
{
    if (RightPWM < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        Motor_SetRightCompare(-RightPWM);
    }
    else if (RightPWM > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        Motor_SetRightCompare(RightPWM);
    }
}
