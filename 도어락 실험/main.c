/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Keypad(4x4) + 4-digit 7seg + Doorlock(0106)
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PD */
/* ====== 고정 조건 ======
   - 7SEG: Common Cathode  -> SEG HIGH=ON
   - DIG: 실측상 LOW=ON
   - Keypad: COL PullDown  -> 선택 ROW를 HIGH로 올리면, 눌린 COL이 HIGH가 됨
*/
#define SEG_ON   GPIO_PIN_SET
#define SEG_OFF  GPIO_PIN_RESET

#define DIG_ON   GPIO_PIN_RESET   // LOW = ON
#define DIG_OFF  GPIO_PIN_SET

#define KP_SCAN_PERIOD_MS   5
#define KP_STABLE_COUNT     3
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static const char PASSWORD[5] = "0106";

static volatile char input_buf[5] = "____";
static volatile uint8_t input_len = 0;

static volatile uint8_t mux_digit = 0;

static volatile uint8_t scan_row = 0;
static volatile uint16_t scan_div = 0;
static volatile char last_sample = 0;
static volatile uint8_t stable_cnt = 0;
static volatile bool key_locked = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
static void tick_1ms(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* ===== 7-seg 폰트 (bit: A B C D E F G DP) ===== */
static uint8_t seg_font(char c)
{
  switch (c) {
    case '0': return 0b00111111;
    case '1': return 0b00000110;
    case '2': return 0b01011011;
    case '3': return 0b01001111;
    case '4': return 0b01100110;
    case '5': return 0b01101101;
    case '6': return 0b01111101;
    case '7': return 0b00000111;
    case '8': return 0b01111111;
    case '9': return 0b01101111;
    case '_': return 0b00001000; // D만 살짝
    case '-': return 0b01000000; // G만
    default:  return 0;
  }
}

static void seg_write(uint8_t pat)
{
  HAL_GPIO_WritePin(SEG_A_GPIO_Port,  SEG_A_Pin,  (pat & (1<<0)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_B_GPIO_Port,  SEG_B_Pin,  (pat & (1<<1)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_C_GPIO_Port,  SEG_C_Pin,  (pat & (1<<2)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_D_GPIO_Port,  SEG_D_Pin,  (pat & (1<<3)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_E_GPIO_Port,  SEG_E_Pin,  (pat & (1<<4)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_F_GPIO_Port,  SEG_F_Pin,  (pat & (1<<5)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_G_GPIO_Port,  SEG_G_Pin,  (pat & (1<<6)) ? SEG_ON : SEG_OFF);
  HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, (pat & (1<<7)) ? SEG_ON : SEG_OFF);
}

static void digits_all_off(void)
{
  HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, DIG_OFF);
  HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, DIG_OFF);
  HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, DIG_OFF);
  HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, DIG_OFF);
}

static void digit_on(uint8_t d)
{
  digits_all_off();
  switch (d) {
    case 0: HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, DIG_ON); break;
    case 1: HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, DIG_ON); break;
    case 2: HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, DIG_ON); break;
    case 3: HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, DIG_ON); break;
    default: break;
  }
}

/* ===== Keypad scan =====
   COL PullDown 이므로:
   - 기본 COL = LOW
   - 선택 ROW를 HIGH로 올리고, 눌린 키의 COL이 HIGH가 됨
*/
static void keypad_set_row(uint8_t r)
{
  // 기본: 전부 HIGH
  HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);

  // 선택 ROW만 LOW
  switch (r) {
    case 0: HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_RESET); break;
    case 1: HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_RESET); break;
    case 2: HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_RESET); break;
    case 3: HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_RESET); break;
    default: break;
  }
}

static uint8_t keypad_read_cols_mask(void)
{
  uint8_t m = 0;
  if (HAL_GPIO_ReadPin(COL_1_GPIO_Port, COL_1_Pin) == GPIO_PIN_RESET) m |= (1<<0);
  if (HAL_GPIO_ReadPin(COL_2_GPIO_Port, COL_2_Pin) == GPIO_PIN_RESET) m |= (1<<1);
  if (HAL_GPIO_ReadPin(COL_3_GPIO_Port, COL_3_Pin) == GPIO_PIN_RESET) m |= (1<<2);
  if (HAL_GPIO_ReadPin(COL_4_GPIO_Port, COL_4_Pin) == GPIO_PIN_RESET) m |= (1<<3);
  return m;
}


static char keypad_read_key_at_row(uint8_t row)
{
  static const char map[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
  };

  uint8_t cols = keypad_read_cols_mask();
  if (!cols) return 0;

  for (uint8_t c=0; c<4; c++) {
    if (cols & (1<<c)) return map[row][c];
  }
  return 0;
}

/* ===== Doorlock helpers ===== */
static void lock_reset(void)
{
  input_len = 0;
  input_buf[0] = '_';
  input_buf[1] = '_';
  input_buf[2] = '_';
  input_buf[3] = '_';
  input_buf[4] = '\0';

  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_GPIO_Port,   RED_Pin,   GPIO_PIN_RESET);

  key_locked = false;
  last_sample = 0;
  stable_cnt = 0;
}

static void push_digit(char d)
{
  if (input_len < 4) {
    input_buf[input_len++] = d;
  }
}

static void check_password(void)
{
  if (input_len == 4 && strncmp((char*)input_buf, PASSWORD, 4) == 0) {
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RED_GPIO_Port,   RED_Pin,   GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_GPIO_Port,   RED_Pin,   GPIO_PIN_SET);
  }
}

static void handle_key(char k)
{
  if (k >= '0' && k <= '9') {
    push_digit(k);
  } else if (k == '*') {
    lock_reset();
  } else if (k == '#') {
    check_password();
  } else {
    /* A,B,C,D 무시 */
  }
}

/* ===== 1ms tick: 7seg + keypad scan ===== */
static void tick_1ms(void)
{
  /* --- 7seg mux --- */
  digits_all_off();
  seg_write(0x00);

  uint8_t pat = seg_font((char)input_buf[mux_digit]);
  seg_write(pat);
  digit_on(mux_digit);
  mux_digit = (mux_digit + 1) & 0x03;

  /* --- keypad scan (강한 방식: release까지 락) --- */
  static uint8_t any_pressed_in_cycle = 0;

  if (++scan_div >= KP_SCAN_PERIOD_MS) {
    scan_div = 0;

    keypad_set_row(scan_row);
    for (volatile int i=0; i<300; i++) __NOP();

    // row에서 raw cols 확인(눌림 감지용)
    uint8_t cols = keypad_read_cols_mask();
    if (cols) any_pressed_in_cycle = 1;

    // 어떤 키인지 읽기
    char k = keypad_read_key_at_row(scan_row);

    // 눌린 순간 1번만 처리
    if (k != 0 && !key_locked) {
      handle_key(k);
      key_locked = true;
    }

    // 다음 row
    scan_row = (scan_row + 1) & 0x03;

    // row 한 바퀴 끝에서 "완전 release" 판단
    if (scan_row == 0) {
      if (!any_pressed_in_cycle) {
        key_locked = false;   // 완전히 떼어졌으면 다음 입력 허용
      }
      any_pressed_in_cycle = 0;
    }
  }
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  lock_reset();

  scan_row = 0;
  keypad_set_row(0);

  digits_all_off();
  seg_write(0x00);

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  while (1)
  {
    __WFI();   // TIM6 인터럽트가 7SEG/키패드 전부 처리
  }
}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169; // 170MHz/(169+1)=1MHz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;    // 1MHz/(999+1)=1kHz => 1ms
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* DIG + LED (PC0..3, PC6, PC8) */
  HAL_GPIO_WritePin(GPIOC, DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin|GREEN_Pin|RED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin|GREEN_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ROW outputs (PC7, PB6, PA7, PA6) */
  HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = ROW_1_Pin;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW_2_Pin;
  HAL_GPIO_Init(ROW_2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROW_3_Pin|ROW_4_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* COL inputs (PullDown) : PA9, PA8, PB10, PB4 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = COL_1_Pin|COL_2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL_3_Pin|COL_4_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SEG outputs : (B: D,E,F,A,B,C) + (A: G,DP) */
  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_A_Pin|SEG_B_Pin|SEG_C_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_A_Pin|SEG_B_Pin|SEG_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SEG_G_Pin|SEG_DP_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) tick_1ms();
}

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
#endif
