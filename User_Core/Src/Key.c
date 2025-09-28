#include "stm32f4xx_hal.h"

uint8_t Get_KeyNum(void)
{
    uint8_t KeyNum = 0;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        // while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 1;
    }
    return KeyNum;
}