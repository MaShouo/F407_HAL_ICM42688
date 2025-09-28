/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Key.h"
#include "OLED.h"
#include "icm42688.h"
#include "TMT.h"
#include "Task.h"
#include "Motor.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t KeyNum = 0;
int8_t IDCheck = 0;
int16_t speed = 0;
icm42688_data_t sensor;
int8_t Speed = 0;
float wheel_circ = 3.1415926f * 0.065f;       // 0.2042 m
float pulses_per_rev = 22 * 30;              // 660 脉冲/轮圈
float control_dt = 0.01f;

PID_t pidAngle = {
  .Kp = 8.8f,      // 初始值，根据实际调
  .Ki = 0.01f,      // 初始 0，可后续微调
  .Kd = 4.8f,     // 初始值，根据振荡情况调
  .OutMax = 200,   // 根据 PWM 定时器最大值设置
  .OutMin = -200,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Target = 0,
  .Actual = 0,
};
PID_t pidSpeed = {
  .Kp = 5.0f,      // 初始值，根据实际调
  .Ki = 0.0f,      // 初始 0，可后续微调
  .Kd = 0.5f,     // 初始值，根据振荡情况调
  .OutMax = 50,
  .OutMin = -50,
  .Error0 = 0,
  .Error1 = 0,
  .ErrorInt = 0,
  .Target = 0,
  .Actual = 0,
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM14_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  tmt_init();
  HAL_Delay(50);
  OLED_Init();


  HAL_TIM_Base_Start_IT(&htim14);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  if (ICM42688_Init() != ICM42688_OK)
  {
    OLED_Printf(0, 0, OLED_6X8, "ICM42688 init failed!");
    OLED_Update();
    while (1);
  }
  Attitude_Init(); // 初始化姿态解算
  OLED_Printf(0, 0, OLED_6X8, "ICM42688 init success");
  OLED_Update();

  HAL_TIM_Base_Start_IT(&htim12);

  // tmt.create(Task1, 1);
  tmt.create(Task_OLED, 100);
  tmt.create(Task_Attitude, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    tmt.run();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t count6 = 0;
  static uint16_t count12 = 0;

  if (htim == (&htim12))
  {
    count12++;
    if(count12 >= 10)
    {
      count12 = 0;

      int16_t cntL = Encoder_GetLeft();
      int16_t cntR = Encoder_GetRight();

      /** 外环速度环 **/
      float speed_avg = (float)(cntL + cntR) * 0.5f;
      float speed_m_s = speed_avg * wheel_circ / pulses_per_rev / control_dt;
      pidSpeed.Target = Speed;
      pidSpeed.Actual = speed_m_s;
      PID_Update(&pidSpeed);
      float angle_offset = pidSpeed.Out;

      /** 内环直立环 **/
      float roll = car_angles.roll;
      pidAngle.Target = -angle_offset;       // 目标直立z
      pidAngle.Actual = roll;               // 反馈
      PID_Update(&pidAngle);
      float pwm = pidAngle.Out;

      Motor_SetLeftPWM(pwm);
      Motor_SetRightPWM(pwm);


    }
  }
  if (htim == (&htim14))
  {
    tmt.tick();
  }
  if (htim == (&htim6))
  {
    count6++;
    if (count6 >= 5)
    {
      count6 = 0;
      ICM42688_Get6Axis(&sensor);
      Complementary_Filter(&sensor, &car_angles);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
