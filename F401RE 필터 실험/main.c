/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32G474RE - ADC(10kHz) -> Digital Filter -> DAC(OUT2=PA5)
  *                   Control via I2C2 SLAVE (from F401RE UI board)
  *                   - Set mode(0..6), Fc(Hz), Q
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  FILT_LPF1 = 0,
  FILT_HPF1 = 1,
  FILT_APF1 = 2,
  FILT_LPF2 = 3,
  FILT_HPF2 = 4,
  FILT_BPF  = 5,
  FILT_NOTCH= 6
} filter_mode_t;

typedef struct {
  float b0, b1, b2;
  float a1, a2;     // y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
  float x1, x2;
  float y1, y2;
} biquad_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FS_HZ           (10000.0f)     // 10kHz sampling
#define FC_DEFAULT_HZ   (100.0f)
#define Q_DEFAULT       (5.0f)

#define FC_MIN_HZ       (1.0f)
#define FC_MAX_HZ       (2000.0f)      // Nyquist 근처 방지

// ---- I2C2 SLAVE address (7-bit) ----
#define G474_I2C_ADDR_7BIT   (0x20)    // F401이 여기에 보냄
#define I2C_CMD_MAX          (8)
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

static biquad_t g_filt;
static volatile filter_mode_t g_mode = FILT_LPF1;

static volatile float g_fc_hz = FC_DEFAULT_HZ;
static volatile float g_q     = Q_DEFAULT;

// I2C RX buffer (command from F401)
static uint8_t g_i2c_rx[I2C_CMD_MAX];
static volatile uint8_t g_i2c_rx_ready = 0;

// UART2 RX (optional backup)
static uint8_t g_rx_byte = 0;
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
static inline float clampf(float v, float lo, float hi);
static void filter_reset(biquad_t *f);
static float filter_process(biquad_t *f, float x);

static void coeff_lpf1(biquad_t *f, float fc, float fs);
static void coeff_hpf1(biquad_t *f, float fc, float fs);
static void coeff_apf1(biquad_t *f, float fc, float fs);

static void coeff_lpf2_butter(biquad_t *f, float fc, float fs);
static void coeff_hpf2_butter(biquad_t *f, float fc, float fs);

static void coeff_bpf(biquad_t *f, float fc, float Q, float fs);
static void coeff_notch(biquad_t *f, float fc, float Q, float fs);

static void filter_select(filter_mode_t mode);
static void filter_set_fc(float fc_hz);
static void filter_set_q(float q);

static void App_Start_Peripherals(void);

// I2C command protocol
static void I2C_ArmRx(void);
static void I2C_ParseAndApply(const uint8_t *buf, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void filter_reset(biquad_t *f)
{
  f->x1 = f->x2 = 0.0f;
  f->y1 = f->y2 = 0.0f;
}

static float filter_process(biquad_t *f, float x)
{
  const float y = (f->b0 * x)
                + (f->b1 * f->x1)
                + (f->b2 * f->x2)
                - (f->a1 * f->y1)
                - (f->a2 * f->y2);

  f->x2 = f->x1;
  f->x1 = x;
  f->y2 = f->y1;
  f->y1 = y;
  return y;
}

/* ===== 1st-order (bilinear, k = tan(pi*fc/fs)) ===== */
static void coeff_lpf1(biquad_t *f, float fc, float fs)
{
  const float k = tanf((float)M_PI * (fc / fs));
  const float norm = 1.0f / (1.0f + k);

  f->b0 = k * norm;
  f->b1 = f->b0;
  f->b2 = 0.0f;
  f->a1 = (1.0f - k) * norm;
  f->a2 = 0.0f;

  filter_reset(f);
}

static void coeff_hpf1(biquad_t *f, float fc, float fs)
{
  const float k = tanf((float)M_PI * (fc / fs));
  const float norm = 1.0f / (1.0f + k);

  f->b0 = 1.0f * norm;
  f->b1 = -1.0f * norm;
  f->b2 = 0.0f;
  f->a1 = (1.0f - k) * norm;
  f->a2 = 0.0f;

  filter_reset(f);
}

static void coeff_apf1(biquad_t *f, float fc, float fs)
{
  const float k = tanf((float)M_PI * (fc / fs));
  const float norm = 1.0f / (1.0f + k);
  const float a1 = (1.0f - k) * norm;

  f->b0 = a1;
  f->b1 = 1.0f;
  f->b2 = 0.0f;
  f->a1 = a1;
  f->a2 = 0.0f;

  filter_reset(f);
}

/* ===== 2nd-order Butterworth (Q=1/sqrt(2)) ===== */
static void coeff_lpf2_butter(biquad_t *f, float fc, float fs)
{
  const float Q = 0.70710678f;
  const float k = tanf((float)M_PI * (fc / fs));
  const float k2 = k * k;
  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  f->b0 = k2 * norm;
  f->b1 = 2.0f * f->b0;
  f->b2 = f->b0;

  f->a1 = 2.0f * (k2 - 1.0f) * norm;
  f->a2 = (1.0f - (k / Q) + k2) * norm;

  filter_reset(f);
}

static void coeff_hpf2_butter(biquad_t *f, float fc, float fs)
{
  const float Q = 0.70710678f;
  const float k = tanf((float)M_PI * (fc / fs));
  const float k2 = k * k;
  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  f->b0 = 1.0f * norm;
  f->b1 = -2.0f * f->b0;
  f->b2 = f->b0;

  f->a1 = 2.0f * (k2 - 1.0f) * norm;
  f->a2 = (1.0f - (k / Q) + k2) * norm;

  filter_reset(f);
}

/* ===== BPF / Notch (biquad) ===== */
static void coeff_bpf(biquad_t *f, float fc, float Q, float fs)
{
  const float k = tanf((float)M_PI * (fc / fs));
  const float k2 = k * k;
  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  f->b0 = (k / Q) * norm;
  f->b1 = 0.0f;
  f->b2 = -f->b0;

  f->a1 = 2.0f * (k2 - 1.0f) * norm;
  f->a2 = (1.0f - (k / Q) + k2) * norm;

  filter_reset(f);
}

static void coeff_notch(biquad_t *f, float fc, float Q, float fs)
{
  const float k = tanf((float)M_PI * (fc / fs));
  const float k2 = k * k;
  const float norm = 1.0f / (1.0f + (k / Q) + k2);

  f->b0 = (1.0f + k2) * norm;
  f->b1 = 2.0f * (k2 - 1.0f) * norm;
  f->b2 = f->b0;

  f->a1 = f->b1;
  f->a2 = (1.0f - (k / Q) + k2) * norm;

  filter_reset(f);
}

/* ===== Filter parameter setters ===== */
static void filter_select(filter_mode_t mode)
{
  g_mode = mode;

  const float fc = (float)g_fc_hz;
  const float q  = (float)g_q;

  switch (mode)
  {
    case FILT_LPF1:   coeff_lpf1(&g_filt, fc, FS_HZ); break;
    case FILT_HPF1:   coeff_hpf1(&g_filt, fc, FS_HZ); break;
    case FILT_APF1:   coeff_apf1(&g_filt, fc, FS_HZ); break;
    case FILT_LPF2:   coeff_lpf2_butter(&g_filt, fc, FS_HZ); break;
    case FILT_HPF2:   coeff_hpf2_butter(&g_filt, fc, FS_HZ); break;
    case FILT_BPF:    coeff_bpf(&g_filt, fc, q, FS_HZ); break;
    case FILT_NOTCH:  coeff_notch(&g_filt, fc, q, FS_HZ); break;
    default:          coeff_lpf1(&g_filt, fc, FS_HZ); break;
  }
}

static void filter_set_fc(float fc_hz)
{
  fc_hz = clampf(fc_hz, FC_MIN_HZ, FC_MAX_HZ);
  g_fc_hz = fc_hz;
  filter_select(g_mode);
}

static void filter_set_q(float q)
{
  q = clampf(q, 0.3f, 20.0f);
  g_q = q;
  filter_select(g_mode);
}

/* ===== I2C command protocol =====
   F401 -> G474 (I2C write)
   buf[0] = cmd
     'M' : set mode        buf[1]=0..6
     'F' : set fc (Hz)     buf[1]=LSB, buf[2]=MSB  (uint16)
     'Q' : set Q*100       buf[1]=LSB, buf[2]=MSB  (uint16, e.g. 500 => 5.00)
*/
static void I2C_ArmRx(void)
{
  g_i2c_rx_ready = 0;
  memset(g_i2c_rx, 0, sizeof(g_i2c_rx));
  (void)HAL_I2C_Slave_Receive_IT(&hi2c2, g_i2c_rx, I2C_CMD_MAX);
}

static void I2C_ParseAndApply(const uint8_t *buf, uint8_t len)
{
  (void)len;
  const uint8_t cmd = buf[0];

  if (cmd == 'M') {
    const uint8_t m = buf[1];
    if (m <= 6) filter_select((filter_mode_t)m);
  }
  else if (cmd == 'F') {
    const uint16_t f = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    filter_set_fc((float)f);
  }
  else if (cmd == 'Q') {
    const uint16_t q100 = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    filter_set_q(((float)q100) / 100.0f);
  }
  else {
    // ignore
  }
}

static void App_Start_Peripherals(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  g_fc_hz = FC_DEFAULT_HZ;
  g_q     = Q_DEFAULT;
  filter_select(FILT_LPF1);

  I2C_ArmRx();

  (void)HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  App_Start_Peripherals();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (g_i2c_rx_ready)
    {
      g_i2c_rx_ready = 0;
      I2C_ParseAndApply(g_i2c_rx, I2C_CMD_MAX);
      I2C_ArmRx();
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

  /** Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters */
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

  /** Initializes the CPU, AHB and APB buses clocks */
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = G474_I2C_ADDR_7BIT; // CubeMX 스타일: 7-bit 값 그대로
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
  htim6.Init.Period = 99;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    (void)HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
      const uint32_t adc_raw = HAL_ADC_GetValue(&hadc1) & 0x0FFF;
      const float x = (float)adc_raw;

      const float y = filter_process(&g_filt, x);

      const float y_clamped = clampf(y, 0.0f, 4095.0f);
      const uint32_t dac_raw = (uint32_t)(y_clamped + 0.5f);

      (void)HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_raw);

      g_adc_raw = adc_raw;
      g_dac_raw = dac_raw;
    }

    (void)HAL_ADC_Stop(&hadc1);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    g_i2c_rx_ready = 1;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    I2C_ArmRx();
  }
}

/* UART2 backup: '0'..'6' mode change (optional) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    const uint8_t c = g_rx_byte;
    if (c >= '0' && c <= '6') {
      filter_select((filter_mode_t)(c - '0'));
    }
    (void)HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
  }
}
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
