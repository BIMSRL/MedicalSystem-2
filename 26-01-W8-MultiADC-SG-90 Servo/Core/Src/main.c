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

#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint16_t channel[3U];
} AdcFrame_t;

typedef struct
{
  AdcFrame_t frame[256U];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  uint32_t overflow_count;
} AdcFrameQueue_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_CHANNEL_COUNT       3U
#define ADC_BUFFER_LENGTH       256U
#define ADC_SAMPLE_RATE_HZ      100.0f
#define FIR_CUTOFF_HZ           20.0f
#define FIR_TAP_COUNT           11U
/* SG90 표준 제어 구간: 1.0ms(최소) ~ 2.0ms(최대), 주기 20ms(50Hz) */
#define SERVO_MIN_PULSE_US      1000U
#define SERVO_MAX_PULSE_US      2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t g_led_toggle_count = 0U;
AdcFrameQueue_t g_adc_queue;
AdcFrame_t g_adc_current_frame;
volatile uint32_t g_adc_total_frames = 0U;
volatile uint8_t g_adc_rank_index = 0U;
uint8_t g_pwm_duty_percent = 0U;
arm_fir_instance_f32 g_fir[ADC_CHANNEL_COUNT];
float32_t g_fir_state[ADC_CHANNEL_COUNT][FIR_TAP_COUNT];

/* 100Hz 샘플링 기준 20Hz 저역통과 FIR 계수(11tap) */
static const float32_t g_fir_coeffs[FIR_TAP_COUNT] = {
  -0.000000000f,
  -0.012641976f,
  -0.024692258f,
   0.063505130f,
   0.274797751f,
   0.398062705f,
   0.274797751f,
   0.063505130f,
  -0.024692258f,
  -0.012641976f,
  -0.000000000f,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
static void AdcQueue_Init(AdcFrameQueue_t* queue);
static uint8_t AdcQueue_Push(AdcFrameQueue_t* queue, const AdcFrame_t* frame);
static uint8_t AdcQueue_Pop(AdcFrameQueue_t* queue, AdcFrame_t* frame);
static uint8_t AdcQueue_IsFull(const AdcFrameQueue_t* queue);
static uint8_t AdcQueue_IsEmpty(const AdcFrameQueue_t* queue);
static uint16_t __attribute__((unused)) AdcQueue_Count(const AdcFrameQueue_t* queue);
static uint32_t __attribute__((unused)) AdcQueue_OverflowCount(const AdcFrameQueue_t* queue);
static void SetPwmDutyPercent(uint8_t duty_percent);
static void ProcessPwmInput(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;

  /* 줄바꿈 문자를 CRLF로 변환 */
  if (c == '\n')
  {
    uint8_t cr = '\r';
    HAL_UART_Transmit(&huart2, &cr, 1U, HAL_MAX_DELAY);
  }

  HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 필터/큐/서보 상태 초기화 순서:
   * 1) 큐 초기화 -> 2) FIR 초기화 -> 3) 서보 초기 듀티 적용
   */
  AdcQueue_Init(&g_adc_queue);
  arm_fir_init_f32(&g_fir[0], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[0], 1U);
  arm_fir_init_f32(&g_fir[1], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[1], 1U);
  arm_fir_init_f32(&g_fir[2], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[2], 1U);
  SetPwmDutyPercent(0U);

  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* UART 안내 문구는 부팅 직후 연결 상태 확인에도 사용됨 */
  printf("ADC sampling started: 3ch, 100Hz trigger, 256-frame ring buffer\n");
  printf("Servo standard mode: 50Hz, 1.0ms~2.0ms pulse\n");
  printf("CMSIS-DSP FIR enabled: ch0,ch1,ch2, %.0fHz low-pass, %.0fHz sample rate\n",
      FIR_CUTOFF_HZ,
      ADC_SAMPLE_RATE_HZ);
  printf("Enter servo ratio 0~100 then press Enter\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* uint32_t last_report_tick = HAL_GetTick(); */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t popped;
    AdcFrame_t frame;

    ProcessPwmInput();

        /*
         * ISR(ADC callback)에서 push 중일 수 있으므로
         * pop 시점만 짧게 임계구역으로 보호한다.
         */
    __disable_irq();
    popped = AdcQueue_Pop(&g_adc_queue, &frame);
    __enable_irq();

    if (popped != 0U)
    {
          float32_t fir_input[ADC_CHANNEL_COUNT];
          float32_t fir_output[ADC_CHANNEL_COUNT];

          /* 큐에서 꺼낸 원시 ADC 프레임(정수)을 FIR 입력(float)으로 변환 */
          fir_input[0] = (float32_t)frame.channel[0];
          fir_input[1] = (float32_t)frame.channel[1];
          fir_input[2] = (float32_t)frame.channel[2];

          /* 채널별 독립 상태를 갖는 FIR 인스턴스로 1샘플씩 처리 */
          arm_fir_f32(&g_fir[0], &fir_input[0], &fir_output[0], 1U);
          arm_fir_f32(&g_fir[1], &fir_input[1], &fir_output[1], 1U);
          arm_fir_f32(&g_fir[2], &fir_input[2], &fir_output[2], 1U);

          /* 원시값 + 필터값 동시 출력(CSV): 추세 비교/검증 용도 */
          printf("%u,%u,%u,%.2f,%.2f,%.2f\n",
             (unsigned int)frame.channel[0],
             (unsigned int)frame.channel[1],
            (unsigned int)frame.channel[2],
            fir_output[0],
            fir_output[1],
            fir_output[2]);
    }

    /*
    now_tick = HAL_GetTick();
    if ((now_tick - last_report_tick) >= 1000U)
    {
      last_report_tick += 1000U;

      __disable_irq();
      queue_count_snapshot = AdcQueue_Count(&g_adc_queue);
      total_frames_snapshot = g_adc_total_frames;
      overflow_snapshot = AdcQueue_OverflowCount(&g_adc_queue);
      __enable_irq();

      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      printf("ADC frames: %lu, queue: %u/256, ovf: %lu, Servo: %u%%\n",
             (unsigned long)total_frames_snapshot,
             (unsigned int)queue_count_snapshot,
             (unsigned long)overflow_snapshot,
             (unsigned int)g_pwm_duty_percent);
      g_led_toggle_count++;
    }
    */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void AdcQueue_Init(AdcFrameQueue_t* queue)
{
  /* 링버퍼 인덱스/카운터를 모두 초기 상태로 설정 */
  queue->head = 0U;
  queue->tail = 0U;
  queue->count = 0U;
  queue->overflow_count = 0U;
}

static uint8_t AdcQueue_IsFull(const AdcFrameQueue_t* queue)
{
  return (queue->count >= ADC_BUFFER_LENGTH) ? 1U : 0U;
}

static uint8_t AdcQueue_IsEmpty(const AdcFrameQueue_t* queue)
{
  return (queue->count == 0U) ? 1U : 0U;
}

static uint16_t AdcQueue_Count(const AdcFrameQueue_t* queue)
{
  return queue->count;
}

static uint32_t AdcQueue_OverflowCount(const AdcFrameQueue_t* queue)
{
  return queue->overflow_count;
}

static uint8_t AdcQueue_Push(AdcFrameQueue_t* queue, const AdcFrame_t* frame)
{
  uint8_t pushed_without_overflow = 1U;

  if (AdcQueue_IsFull(queue) != 0U)
  {
    /* 실시간 스트리밍 우선 정책: 가장 오래된 프레임 폐기 후 최신 프레임 보존 */
    queue->head++;
    if (queue->head >= ADC_BUFFER_LENGTH)
    {
      queue->head = 0U;
    }
    queue->count--;
    queue->overflow_count++;
    pushed_without_overflow = 0U;
  }

  queue->frame[queue->tail] = *frame;
  queue->tail++;
  if (queue->tail >= ADC_BUFFER_LENGTH)
  {
    queue->tail = 0U;
  }
  queue->count++;

  return pushed_without_overflow;
}

static uint8_t AdcQueue_Pop(AdcFrameQueue_t* queue, AdcFrame_t* frame)
{
  if (AdcQueue_IsEmpty(queue) != 0U)
  {
    /* 소비할 데이터가 없으면 즉시 실패 반환 */
    return 0U;
  }

  *frame = queue->frame[queue->head];
  queue->head++;
  if (queue->head >= ADC_BUFFER_LENGTH)
  {
    queue->head = 0U;
  }
  queue->count--;

  return 1U;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t adc_value;
  uint8_t rank_index;

  /* 다른 ADC 인스턴스 인터럽트는 무시 */
  if (hadc->Instance != ADC1)
  {
    return;
  }

  /* 변환 완료된 현재 rank의 값 1개를 읽는다 */
  adc_value = (uint16_t)HAL_ADC_GetValue(hadc);
  rank_index = g_adc_rank_index;

  /* rank 순서대로 임시 프레임 버퍼에 저장 */
  if (rank_index < ADC_CHANNEL_COUNT)
  {
    g_adc_current_frame.channel[rank_index] = adc_value;
  }

  rank_index++;
  if (rank_index >= ADC_CHANNEL_COUNT)
  {
    /* 3채널이 모두 채워지면 1프레임 완성으로 큐에 적재 */
    rank_index = 0U;
    (void)AdcQueue_Push(&g_adc_queue, &g_adc_current_frame);
    g_adc_total_frames++;
  }

  g_adc_rank_index = rank_index;
}

static void SetPwmDutyPercent(uint8_t duty_percent)
{
  uint32_t pulse;
  uint32_t pulse_range;

  /* 입력값 상한 보호 */
  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  /* 0~100% 입력을 1000~2000us 펄스폭으로 선형 매핑 */
  pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
  pulse = SERVO_MIN_PULSE_US + ((pulse_range * (uint32_t)duty_percent) / 100U);
  /* CCR 갱신으로 PWM 듀티 반영 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);

  /* 현재 적용 상태 저장(상태 출력/디버깅 용도) */
  g_pwm_duty_percent = duty_percent;
  printf("Servo set: %u%% (%luus)\n",
         g_pwm_duty_percent,
         (unsigned long)pulse);
}

static void ProcessPwmInput(void)
{
  uint8_t rx_char;
  static char input_buf[8];
  static uint8_t input_len = 0U;

  /* non-blocking 1바이트 수신: 새 입력이 없으면 즉시 리턴 */
  if (HAL_UART_Receive(&huart2, &rx_char, 1U, 0U) != HAL_OK)
  {
    return;
  }

  /* 엔터 입력 시 지금까지 받은 문자열을 숫자로 해석 */
  if ((rx_char == '\r') || (rx_char == '\n'))
  {
    int value;

    /* 빈 라인 엔터는 무시 */
    if (input_len == 0U)
    {
      return;
    }

    input_buf[input_len] = '\0';
    value = atoi(input_buf);

    /* 유효 범위 입력만 서보 출력에 반영 */
    if ((value >= 0) && (value <= 100))
    {
      SetPwmDutyPercent((uint8_t)value);
    }
    else
    {
      printf("Enter servo ratio in range 0~100\n");
    }

    /* 다음 입력을 위해 버퍼 초기화 */
    input_len = 0U;
    return;
  }

  /* 숫자만 버퍼에 누적 */
  if ((rx_char >= '0') && (rx_char <= '9'))
  {
    if (input_len < (sizeof(input_buf) - 1U))
    {
      input_buf[input_len++] = (char)rx_char;
    }
    return;
  }

  /* 공백 문자는 무시 */
  if ((rx_char == ' ') || (rx_char == '\t'))
  {
    return;
  }

  /* 허용되지 않은 문자가 들어오면 현재 입력 폐기 */
  input_len = 0U;
  printf("Invalid input. Enter numeric value 0~100\n");
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
