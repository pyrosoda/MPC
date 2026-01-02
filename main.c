/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum {
  ST_VEH_GREEN_PED_RED = 0,   // 차량G, 보행R (10s)
  ST_VEH_YELLOW_PED_RED,      // 차량Y, 보행R (2s)
  ST_VEH_RED_PED_GREEN        // 차량R, 보행G (10s)
} TrafficState;

static TrafficState g_state = ST_VEH_GREEN_PED_RED;
static uint32_t g_state_start_ms = 0;

static volatile uint8_t g_emergency = 0; // 0: normal, 1: emergency
static uint32_t g_blink_last_ms = 0;
static uint8_t g_blink_on = 0;

/* ---- 시간 파라미터 ---- */
#define T_VEH_GREEN_MS  (10u * 1000u)
#define T_VEH_YELLOW_MS ( 2u * 1000u)
#define T_PED_GREEN_MS  (10u * 1000u)
#define T_BLINK_MS      (500u)

static void all_off(void)
{
  HAL_GPIO_WritePin(GPIOB, VEH_R_Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, VEH_Y_Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, VEH_G_Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, PED_R_Pin_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, PED_G_Pin_Pin, GPIO_PIN_RESET);
}

static void apply_state(TrafficState s)
{
  // 안전하게: 항상 다 끄고 필요한 것만 켬
  all_off();

  switch (s)
  {
    case ST_VEH_GREEN_PED_RED:
      HAL_GPIO_WritePin(GPIOB, VEH_G_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, PED_R_Pin_Pin, GPIO_PIN_SET);
      break;

    case ST_VEH_YELLOW_PED_RED:
      HAL_GPIO_WritePin(GPIOB, VEH_Y_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, PED_R_Pin_Pin, GPIO_PIN_SET);
      break;

    case ST_VEH_RED_PED_GREEN:
      HAL_GPIO_WritePin(GPIOB, VEH_R_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, PED_G_Pin_Pin, GPIO_PIN_SET);
      break;

    default:
      break;
  }
}

static void normal_fsm_update(uint32_t now)
{
  uint32_t elapsed = now - g_state_start_ms;

  switch (g_state)
  {
    case ST_VEH_GREEN_PED_RED:
      if (elapsed >= T_VEH_GREEN_MS) {
        g_state = ST_VEH_YELLOW_PED_RED;
        g_state_start_ms = now;
        apply_state(g_state);
      }
      break;

    case ST_VEH_YELLOW_PED_RED:
      if (elapsed >= T_VEH_YELLOW_MS) {
        g_state = ST_VEH_RED_PED_GREEN;
        g_state_start_ms = now;
        apply_state(g_state);
      }
      break;

    case ST_VEH_RED_PED_GREEN:
      if (elapsed >= T_PED_GREEN_MS) {
        g_state = ST_VEH_GREEN_PED_RED;
        g_state_start_ms = now;
        apply_state(g_state);
      }
      break;

    default:
      break;
  }
}

static void emergency_update(uint32_t now)
{
  // 비상시: 모든 LED 동시 점멸
  if ((now - g_blink_last_ms) >= T_BLINK_MS) {
    g_blink_last_ms = now;
    g_blink_on = !g_blink_on;

    if (g_blink_on) {
      HAL_GPIO_WritePin(GPIOB, VEH_R_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, VEH_Y_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, VEH_G_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, PED_R_Pin_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, PED_G_Pin_Pin, GPIO_PIN_SET);
    } else {
      all_off();
    }
  }
}

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
  /* USER CODE BEGIN 2 */
  g_state = ST_VEH_GREEN_PED_RED;
  g_state_start_ms = HAL_GetTick();
  apply_state(g_state);
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (g_emergency) {
      emergency_update(now);
    } else {
      normal_fsm_update(now);
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, VEH_R_Pin_Pin|VEH_Y_Pin_Pin|VEH_G_Pin_Pin|PED_R_Pin_Pin
                          |PED_G_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VEH_R_Pin_Pin VEH_Y_Pin_Pin VEH_G_Pin_Pin PED_R_Pin_Pin
                           PED_G_Pin_Pin */
  GPIO_InitStruct.Pin = VEH_R_Pin_Pin|VEH_Y_Pin_Pin|VEH_G_Pin_Pin|PED_R_Pin_Pin
                          |PED_G_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) { // PC13 USER 버튼(EXTI13)
    g_emergency = !g_emergency;

    if (g_emergency) {
      g_blink_last_ms = HAL_GetTick();
      g_blink_on = 0;
      all_off();
    } else {
      g_state = ST_VEH_GREEN_PED_RED;
      g_state_start_ms = HAL_GetTick();
      apply_state(g_state);
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
#ifdef USE_FULL_ASSERT
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
