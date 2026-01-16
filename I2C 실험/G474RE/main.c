/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NUCLEO-G474RE 4x4 Keypad -> I2C2 Master TX (send 0~15 keycode)
  *                  + USART2 debug log (printf)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---------------- I2C target(F401RE I2C2 Slave) ----------------
#define I2C_SLAVE_ADDR_7BIT   (0x12)
#define I2C_SLAVE_ADDR_8BIT   (I2C_SLAVE_ADDR_7BIT << 1)
#define I2C_TX_TIMEOUT_MS     (20)

// ---------------- Keypad scan ----------------
#define ROW_STEP_DELAY_MS     (2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t g_row_idx = 0;                 // 0..3
static bool    g_key_locked = false;          // press 후 release까지 락
static bool    g_any_pressed_in_cycle = false;// 4행 사이클 동안 눌림 있었는지
static uint8_t g_rows_done = 0;               // 0..4
static uint8_t g_last_sent = 0xFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Keypad_SetAllRowsHigh(void);
static void Keypad_SetActiveRow(uint8_t row);
static uint8_t Keypad_ReadColMask(void);
static int  FirstSetBitIndex4(uint8_t m);
static uint8_t Keypad_MapToKeycode(uint8_t row, uint8_t col); // 0..15
static void I2C_SendKeycode(uint8_t code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------- printf -> USART2 -------------------- */
int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}


static void Keypad_SetAllRowsHigh(void)
{
  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_SET);
}

static void Keypad_SetActiveRow(uint8_t row)
{
  Keypad_SetAllRowsHigh();

  switch (row)
  {
    case 0: HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET); break;
    case 1: HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET); break;
    case 2: HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET); break;
    case 3: HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_RESET); break;
    default: break;
  }
}

static uint8_t Keypad_ReadColMask(void)
{
  uint8_t m = 0;

  if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin) == GPIO_PIN_RESET) m |= (1u << 0);
  if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin) == GPIO_PIN_RESET) m |= (1u << 1);
  if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin) == GPIO_PIN_RESET) m |= (1u << 2);
  if (HAL_GPIO_ReadPin(COL4_GPIO_Port, COL4_Pin) == GPIO_PIN_RESET) m |= (1u << 3);

  return m;
}

static int FirstSetBitIndex4(uint8_t m)
{
  for (int i = 0; i < 4; i++)
  {
    if (m & (1u << i)) return i;
  }
  return -1;
}

// keycode = row*4 + col  (0..15)
static uint8_t Keypad_MapToKeycode(uint8_t row, uint8_t col)
{
  if (row > 3 || col > 3) return 0xFF;
  return (uint8_t)(row * 4u + col);
}

static void I2C_SendKeycode(uint8_t code)
{
  if (code > 15) return;

  // 같은 코드 계속 보내는 걸 피하고 싶으면 (선택)
  if (code == g_last_sent) return;

  HAL_StatusTypeDef st =
      HAL_I2C_Master_Transmit(&hi2c2, I2C_SLAVE_ADDR_8BIT, &code, 1, I2C_TX_TIMEOUT_MS);

  if (st == HAL_OK)
  {
    g_last_sent = code;
    printf("[I2C2 TX OK] code=%u (0x%02X)\r\n", (unsigned)code, (unsigned)code);
  }
  else
  {
    uint32_t err = HAL_I2C_GetError(&hi2c2);
    printf("[I2C2 TX FAIL] st=%d err=0x%08lX code=%u\r\n",
           (int)st, (unsigned long)err, (unsigned)code);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\nBOOT OK (G474)\r\n");
  printf("USART2 ready @115200\r\n");
  printf("I2C2 Master -> Slave 0x%02X\r\n", I2C_SLAVE_ADDR_7BIT);

  Keypad_SetAllRowsHigh();
  HAL_Delay(10);

  g_row_idx = 0;
  g_key_locked = false;
  g_any_pressed_in_cycle = false;
  g_rows_done = 0;
  g_last_sent = 0xFF;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    // 1) row 활성화
    Keypad_SetActiveRow(g_row_idx);
    HAL_Delay(ROW_STEP_DELAY_MS);

    // 2) col 읽기
    uint8_t colMask = Keypad_ReadColMask();

    // 3) press 1회 확정 + release까지 락
    if (colMask != 0)
    {
      g_any_pressed_in_cycle = true;

      if (!g_key_locked)
      {
        int col = FirstSetBitIndex4(colMask);
        if (col >= 0)
        {
          uint8_t code = Keypad_MapToKeycode(g_row_idx, (uint8_t)col);
          if (code <= 15)
          {
            I2C_SendKeycode(code);
            g_key_locked = true;
          }
        }
      }
    }

    // 4) 다음 row
    g_row_idx = (g_row_idx + 1u) & 0x03u;
    g_rows_done++;

    // 5) 4행 한 사이클 끝에서 release 판단
    if (g_rows_done >= 4)
    {
      if (!g_any_pressed_in_cycle)
      {
        // 한 사이클 동안 완전 무입력이면 release로 판단
        g_key_locked = false;
        // release 후 동일키 재입력도 허용하려면 last_sent도 초기화(선택)
        g_last_sent = 0xFF;
      }

      g_any_pressed_in_cycle = false;
      g_rows_done = 0;
    }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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
  * @brief I2C2 Initialization Function
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // ROW: 각각 포트/핀 기준으로 초기화
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = ROW1_Pin;
  HAL_GPIO_Init(ROW1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW2_Pin;
  HAL_GPIO_Init(ROW2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW3_Pin;
  HAL_GPIO_Init(ROW3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW4_Pin;
  HAL_GPIO_Init(ROW4_GPIO_Port, &GPIO_InitStruct);

  // ROW 기본은 HIGH
  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_SET);

  // COL: 입력 Pull-up
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = COL1_Pin;
  HAL_GPIO_Init(COL1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL2_Pin;
  HAL_GPIO_Init(COL2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL3_Pin;
  HAL_GPIO_Init(COL3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL4_Pin;
  HAL_GPIO_Init(COL4_GPIO_Port, &GPIO_InitStruct);
}


/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

