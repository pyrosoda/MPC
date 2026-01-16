/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NUCLEO-F401RE
  *                  - I2C1: LCD(JLX1602A-4, addr 0x3C, 0x00/0x40 control byte)
  *                  - Keypad 4x4: scan -> UI
  *                  - I2C3 (MASTER): send commands to G474RE filter board
  *                  - USART2: debug printf
  *
  *  UI:
  *   - NORMAL: show MODE + Fc
  *       A: next filter mode (0..6)
  *       B: enter/exit Fc setting mode
  *   - SET_FC: numeric input for Fc (Hz)
  *       0-9: append digit
  *       *  : backspace
  *       #  : apply (send to G474) and return to NORMAL
  *       D  : cancel and return to NORMAL
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
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  UI_NORMAL = 0,
  UI_SET_FC = 1
} ui_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -------------------- LCD (I2C1) Settings -------------------- */
#define LCD_ADDR_7BIT   (0x3C)
#define LCD_CTRL_CMD    (0x00)
#define LCD_CTRL_DATA   (0x40)

/* -------------------- G474 Control (I2C3 Master) -------------------- */
#define G474_ADDR_7BIT   (0x20)              // G474 I2C slave addr (7-bit)
#define G474_ADDR        (G474_ADDR_7BIT<<1) // HAL expects 8-bit addr

/* Filter ranges */
#define FC_MIN_HZ        (1)
#define FC_MAX_HZ        (2000)

/* Debounce */
#define DEBOUNCE_COUNT   (5)                 // 5 * scan_period(ms)

/* Key scan period */
#define SCAN_PERIOD_MS   (2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static ui_state_t g_ui = UI_NORMAL;

/* Filter state (F401 keeps UI truth, sends to G474) */
static uint8_t  g_mode = 0;      // 0..6
static uint16_t g_fc_hz = 100;   // Hz
static uint16_t g_q100  = 500;   // Q*100 (default 5.00)

/* Fc input buffer */
static char g_fc_buf[6] = {0};   // up to 4 digits + null
static uint8_t g_fc_len = 0;

/* Key debounce */
static char g_last_raw = 0;
static char g_stable   = 0;
static uint8_t g_stable_cnt = 0;
static char g_key_event = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* ---------- LCD ---------- */
static void LCD_Write(uint8_t control, uint8_t data);
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(uint8_t data);
static void LCD_SetCursor(uint8_t line, uint8_t column);
static void LCD_Print16(uint8_t line, uint8_t column, const char *s);
static void LCD_Init_User(void);

/* ---------- Keypad ---------- */
static char Map16Key(uint8_t v);
static char Keypad_ScanRaw(void);
static void Keypad_Task(void);

/* ---------- UI ---------- */
static const char* ModeName(uint8_t mode);
static void UI_Render(void);
static void UI_OnKey(char k);

/* ---------- G474 control ---------- */
static void G474_SendMode(uint8_t mode);
static void G474_SendFc(uint16_t fc_hz);
static void G474_SendQ(uint16_t q100);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------- UART printf retarget -------------------- */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* ==================== LCD (I2C1, 0x00/0x40) ==================== */
static void LCD_Write(uint8_t control, uint8_t data)
{
  uint8_t buf[2] = { control, data };
  (void)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(LCD_ADDR_7BIT<<1), buf, 2, HAL_MAX_DELAY);
  HAL_Delay(1);
}

static void LCD_SendCommand(uint8_t cmd)
{
  LCD_Write(LCD_CTRL_CMD, cmd);
  if (cmd == 0x01 || cmd == 0x02) HAL_Delay(2);
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
  for (; i < 16 && s[i] != '\0'; i++) LCD_SendData((uint8_t)s[i]);
  for (; i < 16; i++) LCD_SendData((uint8_t)' ');
}

static void LCD_Init_User(void)
{
  HAL_Delay(50);
  LCD_SendCommand(0x38);
  LCD_SendCommand(0x0C);
  LCD_SendCommand(0x01);
  HAL_Delay(2);
  LCD_SendCommand(0x06);
}

/* ==================== Key Mapper ==================== */
static char Map16Key(uint8_t v)
{
  static const char table[16] = {
    '1','2','3','A',
    '4','5','6','B',
    '7','8','9','C',
    '*','0','#','D'
  };
  if (v < 16) return table[v];
  return 0;
}

/* ==================== Keypad Scan (Active-Low) ==================== */
static void Rows_AllHigh(void)
{
  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_SET);
}

static void Row_SetActive(uint8_t r)
{
  Rows_AllHigh();
  if (r == 0) HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
  if (r == 1) HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
  if (r == 2) HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
  if (r == 3) HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_RESET);
}

static uint8_t Col_ReadIndex(void)
{
  if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin) == GPIO_PIN_RESET) return 0;
  if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin) == GPIO_PIN_RESET) return 1;
  if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin) == GPIO_PIN_RESET) return 2;
  if (HAL_GPIO_ReadPin(COL4_GPIO_Port, COL4_Pin) == GPIO_PIN_RESET) return 3;
  return 0xFF;
}

static char Keypad_ScanRaw(void)
{
  for (uint8_t r = 0; r < 4; r++)
  {
    Row_SetActive(r);
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    uint8_t c = Col_ReadIndex();
    if (c != 0xFF)
    {
      uint8_t idx = (uint8_t)(r * 4 + c);
      return Map16Key(idx);
    }
  }
  Rows_AllHigh();
  return 0;
}

static void Keypad_Task(void)
{
  char raw = Keypad_ScanRaw();

  if (raw == g_last_raw)
  {
    if (g_stable_cnt < 255) g_stable_cnt++;
  }
  else
  {
    g_last_raw = raw;
    g_stable_cnt = 0;
  }

  if (g_stable_cnt >= DEBOUNCE_COUNT)
  {
    if (raw != g_stable)
    {
      g_stable = raw;
      if (g_stable != 0) g_key_event = g_stable; // press event
    }
    g_stable_cnt = DEBOUNCE_COUNT;
  }
}

/* ==================== UI ==================== */
static const char* ModeName(uint8_t mode)
{
  switch (mode)
  {
    case 0: return "LPF1";
    case 1: return "HPF1";
    case 2: return "APF1";
    case 3: return "LPF2";
    case 4: return "HPF2";
    case 5: return "BPF ";
    case 6: return "NOTC";
    default:return "----";
  }
}

static void UI_Render(void)
{
  char l1[17] = {0};
  char l2[17] = {0};

  if (g_ui == UI_NORMAL)
  {
    /* 16-char 안전 포맷 (truncation 방지) */
    snprintf(l1, sizeof(l1), "M:%-4s Fc:%4u", ModeName(g_mode), (unsigned)g_fc_hz);
    snprintf(l2, sizeof(l2), "A Mode  B FcSet");
  }
  else
  {
    snprintf(l1, sizeof(l1), "Set Fc(Hz)     ");
    if (g_fc_len == 0)
      snprintf(l2, sizeof(l2), "Fc=____ *< #OK ");
    else
      snprintf(l2, sizeof(l2), "Fc=%-4s *< #OK ", g_fc_buf);
  }

  LCD_Print16(1, 1, l1);
  LCD_Print16(2, 1, l2);
}

static void UI_OnKey(char k)
{
  if (!k) return;

  printf("[KEY] '%c'\r\n", k);

  if (g_ui == UI_NORMAL)
  {
    if (k == 'A')
    {
      g_mode = (uint8_t)((g_mode + 1) % 7);
      G474_SendMode(g_mode);
      UI_Render();
    }
    else if (k == 'B')
    {
      g_ui = UI_SET_FC;
      g_fc_len = 0;
      memset(g_fc_buf, 0, sizeof(g_fc_buf));
      UI_Render();
    }
  }
  else
  {
    if (k >= '0' && k <= '9')
    {
      if (g_fc_len < 4)
      {
        g_fc_buf[g_fc_len++] = k;
        g_fc_buf[g_fc_len] = 0;
        UI_Render();
      }
    }
    else if (k == '*')
    {
      if (g_fc_len > 0)
      {
        g_fc_len--;
        g_fc_buf[g_fc_len] = 0;
        UI_Render();
      }
    }
    else if (k == '#')
    {
      if (g_fc_len > 0)
      {
        uint16_t v = (uint16_t)atoi(g_fc_buf);
        if (v < FC_MIN_HZ) v = FC_MIN_HZ;
        if (v > FC_MAX_HZ) v = FC_MAX_HZ;
        g_fc_hz = v;
        G474_SendFc(g_fc_hz);
      }
      g_ui = UI_NORMAL;
      UI_Render();
    }
    else if (k == 'D' || k == 'B')
    {
      g_ui = UI_NORMAL;
      UI_Render();
    }
  }
}

/* ==================== G474 Control via I2C3 Master ==================== */
static void G474_SendMode(uint8_t mode)
{
  uint8_t buf[8] = {0};
  buf[0] = 'M';
  buf[1] = (mode <= 6) ? mode : 0;

  if (HAL_I2C_Master_Transmit(&hi2c3, G474_ADDR, buf, sizeof(buf), 50) == HAL_OK)
    printf("[TX] Mode=%u\r\n", (unsigned)buf[1]);
  else
    printf("[TX] Mode FAIL\r\n");
}

static void G474_SendFc(uint16_t fc_hz)
{
  uint8_t buf[8] = {0};
  buf[0] = 'F';
  buf[1] = (uint8_t)(fc_hz & 0xFF);
  buf[2] = (uint8_t)((fc_hz >> 8) & 0xFF);

  if (HAL_I2C_Master_Transmit(&hi2c3, G474_ADDR, buf, sizeof(buf), 50) == HAL_OK)
    printf("[TX] Fc=%uHz\r\n", (unsigned)fc_hz);
  else
    printf("[TX] Fc FAIL\r\n");
}

static void G474_SendQ(uint16_t q100)
{
  uint8_t buf[8] = {0};
  buf[0] = 'Q';
  buf[1] = (uint8_t)(q100 & 0xFF);
  buf[2] = (uint8_t)((q100 >> 8) & 0xFF);

  if (HAL_I2C_Master_Transmit(&hi2c3, G474_ADDR, buf, sizeof(buf), 50) == HAL_OK)
    printf("[TX] Q=%u/100\r\n", (unsigned)q100);
  else
    printf("[TX] Q FAIL\r\n");
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

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\nBOOT OK (F401RE UI)\r\n");
  printf("I2C1(LCD) init done\r\n");
  printf("I2C3(Master->G474) init done, dst=0x%02X\r\n", (unsigned)G474_ADDR_7BIT);
  printf("USART2 init done @115200\r\n");

  LCD_Init_User();
  UI_Render();

  HAL_Delay(10);
  G474_SendMode(g_mode);
  HAL_Delay(10);
  G474_SendFc(g_fc_hz);
  HAL_Delay(10);
  G474_SendQ(g_q100);

  uint32_t t_last = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if ((now - t_last) >= SCAN_PERIOD_MS)
    {
      t_last = now;
      Keypad_Task();

      if (g_key_event != 0)
      {
        char k = g_key_event;
        g_key_event = 0;
        UI_OnKey(k);
      }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  { Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, ROW2_Pin|ROW4_Pin|ROW3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = COL4_Pin|COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROW1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW2_Pin|ROW4_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
