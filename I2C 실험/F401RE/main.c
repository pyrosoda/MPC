/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NUCLEO-F401RE
  *                  I2C1: LCD(JLX1602A-4, addr 0x3C, 0x00/0x40 control byte)
  *                  I2C3: Slave RX from G474RE (16-key code only)
  *                  USART2: debug printf
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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -------------------- LCD (I2C1) Settings -------------------- */
#define LCD_ADDR_7BIT   (0x3C)     // ✅ I2C scan에서 확인된 주소
#define LCD_CTRL_CMD    (0x00)     // control byte for command
#define LCD_CTRL_DATA   (0x40)     // control byte for data

/* -------------------- I2C3 Slave RX Settings -------------------- */
#define I2C_SLAVE_OWN_ADDR_7BIT    (0x12)   // ✅ G474RE가 보내는 목적지 주소(7-bit)
#define RX_LEN                     (1)

/* 최신 입력 표시 문구 */
#define PREFIX                     "Latest Input : "

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t  g_rx = 0;
static volatile bool g_rx_flag = false;

static char g_latest = '?';
static char g_prev   = '\0';


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
static uint8_t I2C_ScanFirst7bit(I2C_HandleTypeDef *hi2c);

static void LCD_Write(uint8_t control, uint8_t data);
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(uint8_t data);
static void LCD_SetCursor(uint8_t line, uint8_t column);
static void LCD_Print16(uint8_t line, uint8_t column, const char *s);
static void LCD_Init(void);

static void I2C3_StartSlaveRx_IT(void);
static char Map16Key(uint8_t v);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------- UART printf retarget -------------------- */
int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* -------------------- I2C Scan (I2C1) -------------------- */
static uint8_t I2C_ScanFirst7bit(I2C_HandleTypeDef *hi2c)
{
  for (uint8_t a = 0x08; a <= 0x77; a++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(a << 1), 2, 50) == HAL_OK)
      return a;
  }
  return 0;
}

/* -------------------- LCD Low-level Write (2 bytes) -------------------- */
static void LCD_Write(uint8_t control, uint8_t data)
{
  uint8_t buf[2];
  buf[0] = control;
  buf[1] = data;

  (void)HAL_I2C_Master_Transmit(&hi2c1,
                               (uint16_t)(LCD_ADDR_7BIT << 1),
                               buf, 2,
                               HAL_MAX_DELAY);

  HAL_Delay(1);
}

/* -------------------- LCD API -------------------- */
static void LCD_SendCommand(uint8_t cmd)
{
  LCD_Write(LCD_CTRL_CMD, cmd);

  if (cmd == 0x01 || cmd == 0x02)
    HAL_Delay(2);
}

static void LCD_SendData(uint8_t data)
{
  LCD_Write(LCD_CTRL_DATA, data);
}

static void LCD_SetCursor(uint8_t line, uint8_t column)
{
  if (line < 1) line = 1;
  if (line > 2) line = 2;
  if (column < 1) column = 1;
  if (column > 16) column = 16;

  uint8_t addr = (uint8_t)((line - 1) * 0x40 + (column - 1));
  LCD_SendCommand((uint8_t)(0x80 | addr));
}

static void LCD_Print16(uint8_t line, uint8_t column, const char *s)
{
  LCD_SetCursor(line, column);

  int i = 0;
  for (; i < 16 && s[i] != '\0'; i++)
    LCD_SendData((uint8_t)s[i]);

  for (; i < 16; i++)
    LCD_SendData((uint8_t)' ');
}

/* -------------------- LCD Init -------------------- */
static void LCD_Init(void)
{
  HAL_Delay(50);

  LCD_SendCommand(0x38);   // 8-bit, 2-line, 5x8
  LCD_SendCommand(0x0C);   // Display ON, cursor OFF
  LCD_SendCommand(0x01);   // Clear
  HAL_Delay(2);
  LCD_SendCommand(0x06);   // Entry mode
}

/* -------------------- 16-key mapper --------------------
   0~15 -> '1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'
*/
static char Map16Key(uint8_t v)
{
  static const char table[16] = {
    '1','2','3','A',
    '4','5','6','B',
    '7','8','9','C',
    '*','0','#','D'
  };

  if (v < 16) return table[v];
  return '?';
}

/* -------------------- I2C3 Slave RX (IT) -------------------- */
static void I2C3_StartSlaveRx_IT(void)
{
  //g_rx_flag = false;
  (void)HAL_I2C_Slave_Receive_IT(&hi2c3, &g_rx, RX_LEN);
}

/* (선택) Listen 완료 콜백: F4에서 Listen 모드 재활성화용 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    HAL_I2C_EnableListen_IT(&hi2c3);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    printf("I2C3 RX callback! g_rx=0x%02X\r\n", g_rx);
    g_rx_flag = true;

    // 연속 수신
    I2C3_StartSlaveRx_IT();
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    uint32_t err = HAL_I2C_GetError(&hi2c3);
    printf("[I2C3 ERROR] err=0x%08lX -> reset I2C3\r\n", (unsigned long)err);

    // ✅ 고착 풀기: 주변장치 리셋
    HAL_I2C_DeInit(&hi2c3);
    MX_I2C3_Init();

    HAL_I2C_EnableListen_IT(&hi2c3);
    I2C3_StartSlaveRx_IT();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\nBOOT OK (F401RE)\r\n");
  printf("I2C1(LCD) init done\r\n");
  printf("I2C3(Slave RX) init done, own=0x%02X\r\n", I2C_SLAVE_OWN_ADDR_7BIT);
  printf("USART2 init done @115200\r\n");

  uint8_t found = I2C_ScanFirst7bit(&hi2c1);
  if (!found)
  {
    printf("I2C1 scan: no device found.\r\n");
    while (1) HAL_Delay(500);
  }
  printf("I2C1 scan: found device at 0x%02X (7-bit)\r\n", found);

  LCD_Init();

  LCD_Print16(1, 1, PREFIX);
  LCD_Print16(2, 1, "Waiting...");

  // I2C3 수신 시작
  //HAL_I2C_EnableListen_IT(&hi2c3);
  I2C3_StartSlaveRx_IT();
  printf("I2C3 Slave RX started.\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (g_rx_flag)
    {
      g_rx_flag = false;

      if (g_rx < 16)
      {
        g_latest = Map16Key(g_rx);
      }
      else
      {
        char c = (char)g_rx;
        if (c >= 0x20 && c <= 0x7E) g_latest = c;
        else g_latest = '?';
      }

      printf("RX: 0x%02X -> '%c'\r\n", g_rx, g_latest);
    }

    if (g_latest != g_prev)
    {
      g_prev = g_latest;

      char line1[17];
      snprintf(line1, sizeof(line1), "%s%c", PREFIX, g_latest);

      LCD_Print16(1, 1, line1);
      LCD_Print16(2, 1, "                ");
    }

    HAL_Delay(10);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function (LCD)
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function (Slave RX)
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;

  // ✅ Slave는 OwnAddress1을 세팅해두는 게 안정적
  hi2c3.Init.OwnAddress1 = (uint16_t)(I2C_SLAVE_OWN_ADDR_7BIT << 1);

  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
