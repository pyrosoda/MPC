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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===== 메트로놈 설정 ===== */
#define BPM_MIN        40
#define BPM_MAX        208

/* 7-seg 극성: 실제 7-seg 타입에 맞춰 바꾸세요.
   - 공통캐소드: SEG_ACTIVE_LOW=0, DIGIT_ACTIVE_LOW=0 (보통)
   - 공통애노드: SEG_ACTIVE_LOW=1, DIGIT_ACTIVE_LOW=1
*/
#define SEG_ACTIVE_LOW   0
#define DIGIT_ACTIVE_LOW 1

/* 피에조 클릭 */
#define CLICK_FREQ_HZ  2000
#define CLICK_MS       12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* ===== 시간/상태 ===== */
static volatile uint32_t g_ms = 0;

/* ADC / BPM */
static volatile uint16_t adc_raw = 0;
static float adc_f = 0.0f;               // EMA 필터
static volatile uint16_t bpm = 120;
static volatile uint32_t beat_period_ms = 500;

/* 박자 진행 */
static volatile uint32_t next_beat_ms = 0;
static volatile uint8_t beat_idx = 0;    // 0~3

/* 피에조 클릭 종료 */
static volatile uint8_t clicking = 0;
static volatile uint32_t click_off_ms = 0;

/* 7-seg 표시/스캔 */
static volatile uint16_t disp_value = 120; // bpm 표시
static volatile uint8_t scan_digit = 0;    // 0~3

/* 7-seg 숫자 LUT: bit0=a ... bit6=g, bit7=dp */
static const uint8_t seg_lut[10] = {
  /*0*/ 0b00111111,
  /*1*/ 0b00000110,
  /*2*/ 0b01011011,
  /*3*/ 0b01001111,
  /*4*/ 0b01100110,
  /*5*/ 0b01101101,
  /*6*/ 0b01111101,
  /*7*/ 0b00000111,
  /*8*/ 0b01111111,
  /*9*/ 0b01101111
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

static uint16_t MapADCToBPM(uint16_t raw);
static void BeatLED_Update(uint8_t idx);

static void SEG_WriteRaw(uint8_t bits);
static void DIGIT_AllOff(void);
static void DIGIT_On(uint8_t d);
static void SevenSeg_Scan1ms(void);

static void Piezo_On(void);
static void Piezo_Off(void);

static void Metronome_Tick1ms(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===== ADC -> BPM 매핑 ===== */
static uint16_t MapADCToBPM(uint16_t raw)
{
  const uint32_t ADC_MAX = 4095; // 12bit
  uint32_t v = raw;
  uint32_t b = BPM_MIN + (v * (BPM_MAX - BPM_MIN)) / ADC_MAX;
  if (b < BPM_MIN) b = BPM_MIN;
  if (b > BPM_MAX) b = BPM_MAX;
  return (uint16_t)b;
}

/* ===== 4박 LED: GREEN1, GREEN2, GREEN3, RED ===== */
static void BeatLED_Update(uint8_t idx)
{
  HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, (idx==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN2_GPIO_Port, GREEN2_Pin, (idx==1)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN3_GPIO_Port, GREEN3_Pin, (idx==2)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_GPIO_Port,    RED_Pin,    (idx==3)?GPIO_PIN_SET:GPIO_PIN_RESET);
}

/* ===== 7-seg: 세그먼트 출력 ===== */
static void SEG_WriteRaw(uint8_t bits)
{
#if SEG_ACTIVE_LOW
  bits = (uint8_t)~bits;
#endif

  HAL_GPIO_WritePin(SEG_A_GPIO_Port,  SEG_A_Pin,  (bits & (1<<0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_B_GPIO_Port,  SEG_B_Pin,  (bits & (1<<1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_C_GPIO_Port,  SEG_C_Pin,  (bits & (1<<2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_D_GPIO_Port,  SEG_D_Pin,  (bits & (1<<3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_E_GPIO_Port,  SEG_E_Pin,  (bits & (1<<4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_F_GPIO_Port,  SEG_F_Pin,  (bits & (1<<5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_G_GPIO_Port,  SEG_G_Pin,  (bits & (1<<6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, (bits & (1<<7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void DIGIT_AllOff(void)
{
#if DIGIT_ACTIVE_LOW
  HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET);
#endif
}

static void DIGIT_On(uint8_t d)
{
  DIGIT_AllOff();
#if DIGIT_ACTIVE_LOW
  if (d==0) HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_RESET);
  if (d==1) HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_RESET);
  if (d==2) HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_RESET);
  if (d==3) HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_RESET);
#else
  if (d==0) HAL_GPIO_WritePin(DIG1_GPIO_Port, DIG1_Pin, GPIO_PIN_SET);
  if (d==1) HAL_GPIO_WritePin(DIG2_GPIO_Port, DIG2_Pin, GPIO_PIN_SET);
  if (d==2) HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, GPIO_PIN_SET);
  if (d==3) HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, GPIO_PIN_SET);
#endif
}

/* 1ms마다 한 자리 스캔 */
static void SevenSeg_Scan1ms(void)
{
  uint16_t val = disp_value;
  uint8_t d0 = (val / 1000) % 10;
  uint8_t d1 = (val / 100)  % 10;
  uint8_t d2 = (val / 10)   % 10;
  uint8_t d3 = (val / 1)    % 10;

  uint8_t digit_num = 0;
  switch (scan_digit) {
    case 0: digit_num = d0; break;
    case 1: digit_num = d1; break;
    case 2: digit_num = d2; break;
    default: digit_num = d3; break;
  }

  /* 선행 0 숨김(원치 않으면 이 블럭을 지우세요) */
  uint8_t blank = 0;
  if (scan_digit==0 && d0==0) blank = 1;
  if (scan_digit==1 && d0==0 && d1==0) blank = 1;

  DIGIT_AllOff();
  SEG_WriteRaw(blank ? 0x00 : seg_lut[digit_num]);
  DIGIT_On(scan_digit);

  scan_digit = (scan_digit + 1) & 0x03;
}

/* ===== 피에조(PWM) ===== */
static void Piezo_On(void)
{
  /* TIM2 설정은 2kHz(PSC=169, ARR=499)로 이미 되어 있음.
     듀티 50% -> CCR = (ARR+1)/2 = 250 정도 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  clicking = 1;
  click_off_ms = g_ms + CLICK_MS;
}

static void Piezo_Off(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  clicking = 0;
}

/* ===== 1ms tick 메트로놈 ===== */
static void Metronome_Tick1ms(void)
{
  if (clicking && (int32_t)(g_ms - click_off_ms) >= 0) {
    Piezo_Off();
  }

  if ((int32_t)(g_ms - next_beat_ms) >= 0) {
    next_beat_ms += beat_period_ms;       // 드리프트 방지
    beat_idx = (beat_idx + 1) & 0x03;     // 0~3
    BeatLED_Update(beat_idx);
    Piezo_On();
  }
}

/* TIM6 1ms 콜백 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    g_ms++;
    SevenSeg_Scan1ms();
    Metronome_Tick1ms();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* ADC 시작 (ContinuousConvMode=ENABLE 이므로 Start 한번이면 충분) */
  HAL_ADC_Start(&hadc1);

  /* 초기 박자/표시 */
  bpm = 120;
  disp_value = bpm;
  beat_period_ms = 60000U / bpm;

  beat_idx = 0;
  BeatLED_Update(beat_idx);

  next_beat_ms = g_ms + beat_period_ms;

  /* TIM6 1ms 인터럽트 시작 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 만약 CubeMX에서 NVIC를 체크 안 했으면, 아래 두 줄이 도움이 됩니다.
     (stm32g4xx_it.c에 TIM6_DAC_IRQHandler가 있어야 실제로 인터럽트가 들어옵니다.) */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE END 2 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ADC 값 읽기 (폴링) */
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

    /* EMA 필터 */
    if (adc_f == 0.0f) adc_f = (float)adc_raw;
    adc_f = adc_f * 0.90f + (float)adc_raw * 0.10f;

    uint16_t new_bpm = MapADCToBPM((uint16_t)adc_f);

    /* 1BPM 이상 변할 때만 갱신 */
    if ((new_bpm > bpm && (new_bpm - bpm) >= 1) ||
        (bpm > new_bpm && (bpm - new_bpm) >= 1))
    {
      bpm = new_bpm;
      disp_value = bpm;
      beat_period_ms = 60000U / bpm;
    }

    /* ADC Continuous ON이라 Start 재호출 불필요 */
    HAL_Delay(15);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin
                          |GREEN1_Pin|RED_Pin|GREEN3_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_A_Pin
                          |SEG_B_Pin|SEG_C_Pin|GREEN2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin|DIG3_Pin|DIG4_Pin
                          |GREEN1_Pin|RED_Pin|GREEN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SEG_D_Pin|SEG_E_Pin|SEG_F_Pin|SEG_A_Pin
                          |SEG_B_Pin|SEG_C_Pin|GREEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SEG_G_Pin|SEG_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
