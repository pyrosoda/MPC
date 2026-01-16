/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ADC -> 1st order LPF -> DAC(OUT2=PA5), TIM6 10kHz ISR
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FS_HZ   (10000.0f)   // sampling frequency = 10kHz
#define TS_SEC  (1.0f/FS_HZ)
#define FC_HZ   (100.0f)     // 1st LPF cutoff = 100Hz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint32_t g_adc_raw = 0;
static volatile uint32_t g_dac_raw = 0;

/* 1st LPF state (raw-domain) */
static float lpf_x1 = 0.0f;   // x[n-1]
static float lpf_y1 = 0.0f;   // y[n-1]

/* 1st LPF coefficients (Tustin) */
static float lpf_b0 = 0.0f;
static float lpf_b1 = 0.0f;
static float lpf_fb = 0.0f;   // feedback gain = (-A1) in y = fb*y1 + b0*x + b1*x1

/* 2nd-order LPF (Butterworth) state */
static float lpf2_x1 = 0.0f, lpf2_x2 = 0.0f;
static float lpf2_y1 = 0.0f, lpf2_y2 = 0.0f;

/* 2nd-order LPF coefficients */
static float lpf2_b0, lpf2_b1, lpf2_b2;
static float lpf2_a1, lpf2_a2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_Start_Peripherals(void);
static void LPF1_Init(float fc_hz, float fs_hz);
static inline float clampf(float v, float lo, float hi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* 1st-order LPF coefficients using Tustin(bilinear transform)
   Analog: H(s) = wc / (s + wc), wc = 2*pi*fc
   Discrete:
     y[n] = fb*y[n-1] + b0*x[n] + b1*x[n-1]
   where:
     b0=b1 = wc / (2/T + wc)
     A1    = (-2/T + wc) / (2/T + wc)
     fb    = -A1
*/
static void LPF1_Init(float fc_hz, float fs_hz)
{
  const float T  = 1.0f / fs_hz;
  const float wc = 2.0f * 3.14159265358979f * fc_hz;
  const float two_over_T = 2.0f / T;

  const float denom = (two_over_T + wc);
  const float b = wc / denom;
  const float A1 = (-two_over_T + wc) / denom;

  lpf_b0 = b;
  lpf_b1 = b;
  lpf_fb = -A1;

  /* reset state */
  lpf_x1 = 0.0f;
  lpf_y1 = 0.0f;
}
static void HPF1_Init(float fc_hz, float fs_hz)
{
  const float T  = 1.0f / fs_hz;
  const float wc = 2.0f * 3.14159265358979f * fc_hz;
  const float two_over_T = 2.0f / T;

  const float denom = (two_over_T + wc);

  const float b0 = (two_over_T) / denom;
  const float b1 = -b0;

  const float A1 = (wc - two_over_T) / denom;  // same as (-two_over_T + wc)/denom
  const float fb = -A1;

  lpf_b0 = b0;
  lpf_b1 = b1;
  lpf_fb = fb;

  /* reset state */
  lpf_x1 = 0.0f;
  lpf_y1 = 0.0f;
}
static void APF1_Init(float fc_hz, float fs_hz)
{
  const float k = tanf(3.14159265358979f * (fc_hz / fs_hz));
  const float a = (1.0f - k) / (1.0f + k);   // all-pass coefficient

  // y[n] = (-a)*y[n-1] + a*x[n] + 1*x[n-1]
  lpf_fb = -a;
  lpf_b0 =  a;
  lpf_b1 =  1.0f;

  /* reset state */
  lpf_x1 = 0.0f;
  lpf_y1 = 0.0f;
}
static void LPF2_Butter_Init(float fc_hz, float fs_hz)
{
  const float Q = 0.70710678f;   // Butterworth
  const float k = tanf(3.14159265358979f * fc_hz / fs_hz);
  const float k2 = k * k;

  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  lpf2_b0 = k2 * norm;
  lpf2_b1 = 2.0f * lpf2_b0;
  lpf2_b2 = lpf2_b0;

  lpf2_a1 = 2.0f * (k2 - 1.0f) * norm;
  lpf2_a2 = (1.0f - (k / Q) + k2) * norm;

  /* reset state */
  lpf2_x1 = lpf2_x2 = 0.0f;
  lpf2_y1 = lpf2_y2 = 0.0f;
}
static void HPF2_Butter_Init(float fc_hz, float fs_hz)
{
  const float Q = 0.70710678f;   // Butterworth
  const float k = tanf(3.14159265358979f * fc_hz / fs_hz);
  const float k2 = k * k;

  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  // High-pass biquad
  lpf2_b0 = 1.0f * norm;
  lpf2_b1 = -2.0f * lpf2_b0;
  lpf2_b2 = lpf2_b0;

  lpf2_a1 = 2.0f * (k2 - 1.0f) * norm;
  lpf2_a2 = (1.0f - (k / Q) + k2) * norm;

  // reset state
  lpf2_x1 = lpf2_x2 = 0.0f;
  lpf2_y1 = lpf2_y2 = 0.0f;
}
static void BPF_Init(float fc_hz, float Q, float fs_hz)
{
  const float k  = tanf(3.14159265358979f * fc_hz / fs_hz);
  const float k2 = k * k;

  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  // Band-pass (constant skirt gain)
  lpf2_b0 = (k / Q) * norm;
  lpf2_b1 = 0.0f;
  lpf2_b2 = -lpf2_b0;

  lpf2_a1 = 2.0f * (k2 - 1.0f) * norm;
  lpf2_a2 = (1.0f - (k / Q) + k2) * norm;

  // reset state
  lpf2_x1 = lpf2_x2 = 0.0f;
  lpf2_y1 = lpf2_y2 = 0.0f;
}
static void BSF_Init(float fc_hz, float Q, float fs_hz)
{
  const float k  = tanf(3.14159265358979f * fc_hz / fs_hz);
  const float k2 = k * k;

  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  // Notch (band-stop)
  lpf2_b0 = (1.0f + k2) * norm;
  lpf2_b1 = 2.0f * (k2 - 1.0f) * norm;
  lpf2_b2 = lpf2_b0;

  lpf2_a1 = lpf2_b1;
  lpf2_a2 = (1.0f - (k / Q) + k2) * norm;

  // reset state
  lpf2_x1 = lpf2_x2 = 0.0f;
  lpf2_y1 = lpf2_y2 = 0.0f;
}


static void App_Start_Peripherals(void)
{
  /* ADC calibration (single-ended) */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start DAC OUT2 (PA5) */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Init LPF (fc=100Hz, Fs=10kHz) */
  BSF_Init(FC_HZ, 5.0f, FS_HZ);



  /* Start TIM6 update interrupt (10kHz) */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  App_Start_Peripherals();
  /* USER CODE END 2 */

  while (1)
  {
    /* main loop idle. ADC->LPF->DAC runs in TIM6 ISR */
  }
}

/* --- SystemClock_Config() --- */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/* --- MX_ADC1_Init() --- */
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_1; // 보통 PA0(ADC1_IN1)
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* --- MX_DAC1_Init()  OUT2=PA5, CH2) --- */
static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK) { Error_Handler(); }

  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) { Error_Handler(); }
}

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
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) { Error_Handler(); }
}

/* --- TIM6: 10kHz(100us) --- */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;  // 170MHz/(169+1)=1MHz tick
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;      // 1MHz/(99+1)=10kHz update
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL4_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL3_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* TIM6 ISR: ADC -> 1st LPF -> DAC */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    (void)HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
      const uint32_t adc_raw_u32 = (HAL_ADC_GetValue(&hadc1) & 0x0FFF);
      const float x = (float)adc_raw_u32;

      const float y =
          (lpf2_b0 * x)
        + (lpf2_b1 * lpf2_x1)
        + (lpf2_b2 * lpf2_x2)
        - (lpf2_a1 * lpf2_y1)
        - (lpf2_a2 * lpf2_y2);

      /* update state */
      lpf2_x2 = lpf2_x1;
      lpf2_x1 = x;
      lpf2_y2 = lpf2_y1;
      lpf2_y1 = y;


      /* clamp to DAC range */
      const float y_offset = y + 2048.0f*0.3f;
      const float y_clamped = clampf(y_offset, 0.0f, 4095.0f);
      const uint32_t dac_raw = (uint32_t)(y_clamped + 0.5f);

      (void)HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_raw);

      g_adc_raw = adc_raw_u32;
      g_dac_raw = dac_raw;
    }

    (void)HAL_ADC_Stop(&hadc1);
  }
}

/* USER CODE END 4 */

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

