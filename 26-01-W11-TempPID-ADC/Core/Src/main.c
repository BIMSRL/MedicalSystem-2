/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @date           : 2026-05-08
  * @author         : JeongWhan Lee
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
#include <ctype.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_MODE_IDLE = 0,
  APP_MODE_MELODY,
  APP_MODE_PID,
  APP_MODE_TEMP
} AppMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUZZER_TIMER_TICK_HZ 10000U
#define BUZZER_NOTE_MS       1000U
#define BUZZER_GAP_MS        100U
#define UART_CMD_BUF_LEN     64U
#define UART_RX_QUEUE_LEN    64U
#define ADC_MOVING_AVG_SAMPLES 8U

#define TMP235_OFFSET_V         0.5f
#define TMP235_SLOPE_V_PER_C    0.01f
#define ADC_VREF_V              3.3f
#define ADC_MAX_COUNT           4095.0f

#define PID_SETPOINT_DEFAULT_C  25.0f
#define PID_CONTROL_MS          50U
#define PID_PWM_FREQ_HZ         20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t led_toggle_time = 0;
const uint16_t melody_freq_hz[3] = {262U, 294U, 330U};
uint8_t melody_index = 0;
uint8_t melody_phase = 0;
uint32_t melody_tick = 0;
uint8_t uart_rx_byte = 0U;
char uart_cmd_buf[UART_CMD_BUF_LEN] = {0};
uint8_t uart_cmd_idx = 0U;
volatile uint8_t uart_rx_queue[UART_RX_QUEUE_LEN] = {0};
volatile uint16_t uart_rx_q_head = 0U;
volatile uint16_t uart_rx_q_tail = 0U;
volatile uint32_t uart_rx_q_overflow = 0U;

AppMode_t app_mode = APP_MODE_MELODY;
uint32_t pwm_arr_current = 0U;

float pid_setpoint_c = PID_SETPOINT_DEFAULT_C;
float pid_kp = 8.0f;
float pid_ki = 0.12f;
float pid_kd = 1.2f;
float pid_integral = 0.0f;
float pid_prev_error = 0.0f;
float pid_last_temp_c = 0.0f;
float pid_last_output = 0.0f;
uint16_t pid_last_adc_raw = 0U;
uint32_t pid_last_tick = 0U;
uint32_t pid_report_tick = 0U;
uint32_t temp_report_tick = 0U;
uint8_t pid_cv_csv_mode = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void Buzzer_SetFrequency(uint32_t freq_hz);
static void Buzzer_Stop(void);
static void UART_ProcessCommand(void);
static uint8_t UART_RxQueuePush(uint8_t data);
static uint8_t UART_RxQueuePop(uint8_t *data);
static void PWM_SetFrequency(uint32_t freq_hz);
static void PWM_SetDutyPercent(float duty_percent);
static uint16_t ADC_ReadChannel(uint32_t channel);
static float TMP235_ReadTempC(void);
static void EnterMelodyMode(void);
static void EnterPidMode(void);
static void EnterTempMode(void);
static void EnterIdleMode(void);
static void PID_Update(void);
static uint8_t PID_ParseSetpointCommand(const char *cmd, float *setpoint_out);
static uint8_t PID_ParseFloatToken(const char **p_inout, float *value_out);
static uint8_t PID_ParseCoeffCommand(const char *cmd,
                                     uint8_t *target_out,
                                     float *value_out,
                                     uint8_t *has_value_out,
                                     uint8_t *is_all_out,
                                     float *all_kp_out,
                                     float *all_ki_out,
                                     float *all_kd_out);
static const char *AppModeToString(AppMode_t mode);
static void UART_PrintCurrentStatus(void);
int __io_putchar(int ch);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Buzzer_SetFrequency(uint32_t freq_hz)
{
  if (freq_hz == 0U)
  {
    Buzzer_Stop();
    return;
  }

  PWM_SetFrequency(freq_hz);
  PWM_SetDutyPercent(50.0f);
}

static void PWM_SetFrequency(uint32_t freq_hz)
{
  uint32_t arr;

  if (freq_hz == 0U)
  {
    return;
  }

  arr = (BUZZER_TIMER_TICK_HZ / freq_hz);
  if (arr > 0U)
  {
    arr -= 1U;
  }
  if (arr < 1U)
  {
    arr = 1U;
  }

  pwm_arr_current = arr;
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  (void)HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

static void PWM_SetDutyPercent(float duty_percent)
{
  uint32_t compare;

  if (duty_percent < 0.0f)
  {
    duty_percent = 0.0f;
  }
  if (duty_percent > 100.0f)
  {
    duty_percent = 100.0f;
  }

  if (duty_percent >= 100.0f)
  {
    /* CCR > ARR makes output stay high continuously in PWM mode. */
    compare = pwm_arr_current + 1U;
  }
  else
  {
    compare = (uint32_t)(((float)(pwm_arr_current + 1U) * duty_percent) / 100.0f);
    if (compare > pwm_arr_current)
    {
      compare = pwm_arr_current;
    }
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
}

static void Buzzer_Stop(void)
{
  PWM_SetDutyPercent(0.0f);
}

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
  return ch;
}

static uint8_t UART_RxQueuePush(uint8_t data)
{
  uint16_t next = (uint16_t)((uart_rx_q_head + 1U) % UART_RX_QUEUE_LEN);

  if (next == uart_rx_q_tail)
  {
    uart_rx_q_overflow++;
    return 0U;
  }

  uart_rx_queue[uart_rx_q_head] = data;
  uart_rx_q_head = next;
  return 1U;
}

static uint8_t UART_RxQueuePop(uint8_t *data)
{
  if (uart_rx_q_head == uart_rx_q_tail)
  {
    return 0U;
  }

  *data = uart_rx_queue[uart_rx_q_tail];
  uart_rx_q_tail = (uint16_t)((uart_rx_q_tail + 1U) % UART_RX_QUEUE_LEN);
  return 1U;
}

static uint16_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  {
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return raw;
  }
}

static float TMP235_ReadTempC(void)
{
  static uint16_t adc_hist[ADC_MOVING_AVG_SAMPLES] = {0U};
  static uint32_t adc_sum = 0U;
  static uint8_t adc_idx = 0U;
  static uint8_t adc_count = 0U;
  uint16_t raw = ADC_ReadChannel(ADC_CHANNEL_6);
  uint16_t avg_raw;
  float voltage;

  if (adc_count < ADC_MOVING_AVG_SAMPLES)
  {
    adc_sum += raw;
    adc_hist[adc_idx] = raw;
    adc_idx = (uint8_t)((adc_idx + 1U) % ADC_MOVING_AVG_SAMPLES);
    adc_count++;
    avg_raw = (uint16_t)(adc_sum / adc_count);
  }
  else
  {
    adc_sum -= adc_hist[adc_idx];
    adc_hist[adc_idx] = raw;
    adc_sum += raw;
    adc_idx = (uint8_t)((adc_idx + 1U) % ADC_MOVING_AVG_SAMPLES);
    avg_raw = (uint16_t)(adc_sum / ADC_MOVING_AVG_SAMPLES);
  }

  pid_last_adc_raw = avg_raw;
  voltage = ((float)avg_raw * ADC_VREF_V) / ADC_MAX_COUNT;
  return (voltage - TMP235_OFFSET_V) / TMP235_SLOPE_V_PER_C;
}

static void EnterMelodyMode(void)
{
  app_mode = APP_MODE_MELODY;
  melody_index = 0U;
  melody_phase = 0U;
  melody_tick = HAL_GetTick();
  Buzzer_SetFrequency(melody_freq_hz[melody_index]);
}

static void EnterPidMode(void)
{
  app_mode = APP_MODE_PID;
  pid_integral = 0.0f;
  pid_prev_error = 0.0f;
  pid_last_tick = HAL_GetTick();
  pid_report_tick = pid_last_tick;
  PWM_SetFrequency(PID_PWM_FREQ_HZ);
  PWM_SetDutyPercent(0.0f);
}

static void EnterTempMode(void)
{
  app_mode = APP_MODE_TEMP;
  Buzzer_Stop();
  PWM_SetDutyPercent(0.0f);
  temp_report_tick = HAL_GetTick();
}

static void EnterIdleMode(void)
{
  app_mode = APP_MODE_IDLE;
  Buzzer_Stop();
}

static void PID_Update(void)
{
  float error;
  float derivative;
  float output;
  const float dt = ((float)PID_CONTROL_MS) / 1000.0f;

  pid_last_temp_c = TMP235_ReadTempC();
  error = pid_setpoint_c - pid_last_temp_c;

  pid_integral += error * dt;
  if (pid_integral > 100.0f)
  {
    pid_integral = 100.0f;
  }
  else if (pid_integral < -100.0f)
  {
    pid_integral = -100.0f;
  }

  derivative = (error - pid_prev_error) / dt;
  output = (pid_kp * error) + (pid_ki * pid_integral) + (pid_kd * derivative);

  if (output < 0.0f)
  {
    output = 0.0f;
  }
  if (output > 100.0f)
  {
    output = 100.0f;
  }

  pid_last_output = output;
  PWM_SetDutyPercent(output);
  pid_prev_error = error;
}

static uint8_t PID_ParseSetpointCommand(const char *cmd, float *setpoint_out)
{
  const char *p = cmd;
  int sign = 1;
  int32_t int_part = 0;
  int32_t frac_part = 0;
  int32_t frac_div = 1;
  float value;

  if ((p[0] != 's') || (p[1] != 'p'))
  {
    return 0U;
  }
  p += 2;

  while ((*p == ' ') || (*p == '\t') || (*p == '='))
  {
    p++;
  }

  if ((*p == '+') || (*p == '-'))
  {
    if (*p == '-')
    {
      sign = -1;
    }
    p++;
  }

  if (!isdigit((unsigned char)*p))
  {
    return 0U;
  }

  while (isdigit((unsigned char)*p))
  {
    int_part = (int_part * 10) + (int32_t)(*p - '0');
    p++;
  }

  if (*p == '.')
  {
    p++;
    while (isdigit((unsigned char)*p) && (frac_div < 1000))
    {
      frac_part = (frac_part * 10) + (int32_t)(*p - '0');
      frac_div *= 10;
      p++;
    }
  }

  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  if (*p != '\0')
  {
    return 0U;
  }

  value = (float)sign * ((float)int_part + ((float)frac_part / (float)frac_div));
  if ((value < 0.0f) || (value > 100.0f))
  {
    return 0U;
  }

  *setpoint_out = value;
  return 1U;
}

static uint8_t PID_ParseFloatToken(const char **p_inout, float *value_out)
{
  const char *p = *p_inout;
  int sign = 1;
  int32_t int_part = 0;
  int32_t frac_part = 0;
  int32_t frac_div = 1;
  float value;

  if ((*p == '+') || (*p == '-'))
  {
    if (*p == '-')
    {
      sign = -1;
    }
    p++;
  }

  if (!isdigit((unsigned char)*p))
  {
    return 0U;
  }

  while (isdigit((unsigned char)*p))
  {
    int_part = (int_part * 10) + (int32_t)(*p - '0');
    p++;
  }

  if (*p == '.')
  {
    p++;
    while (isdigit((unsigned char)*p) && (frac_div < 10000))
    {
      frac_part = (frac_part * 10) + (int32_t)(*p - '0');
      frac_div *= 10;
      p++;
    }
  }

  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  if (*p != '\0')
  {
    return 0U;
  }

  value = (float)sign * ((float)int_part + ((float)frac_part / (float)frac_div));
  if ((value < 0.0f) || (value > 1000.0f))
  {
    return 0U;
  }

  *p_inout = p;
  *value_out = value;
  return 1U;
}

static uint8_t PID_ParseCoeffCommand(const char *cmd,
                                     uint8_t *target_out,
                                     float *value_out,
                                     uint8_t *has_value_out,
                                     uint8_t *is_all_out,
                                     float *all_kp_out,
                                     float *all_ki_out,
                                     float *all_kd_out)
{
  const char *p = cmd;
  const char *name_start;
  int name_len = 0;

  if ((p[0] != 'c') || (p[1] != 'o') || (p[2] != 'e') || (p[3] != 'f') || (p[4] != 'f'))
  {
    return 0U;
  }
  p += 5;

  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  *is_all_out = 0U;
  if (*p == '\0')
  {
    *target_out = 0U;
    *has_value_out = 0U;
    return 1U;
  }

  name_start = p;
  while (isalpha((unsigned char)*p))
  {
    name_len++;
    p++;
  }

  if ((name_len == 2) && (name_start[0] == 'k') && (name_start[1] == 'p'))
  {
    *target_out = 1U;
  }
  else if ((name_len == 2) && (name_start[0] == 'k') && (name_start[1] == 'i'))
  {
    *target_out = 2U;
  }
  else if ((name_len == 2) && (name_start[0] == 'k') && (name_start[1] == 'd'))
  {
    *target_out = 3U;
  }
  else if ((name_len == 3) && (name_start[0] == 'a') && (name_start[1] == 'l') && (name_start[2] == 'l'))
  {
    *is_all_out = 1U;
    *target_out = 0U;
  }
  else
  {
    return 0U;
  }

  while ((*p == ' ') || (*p == '\t') || (*p == '='))
  {
    p++;
  }

  if (*is_all_out != 0U)
  {
    if (PID_ParseFloatToken(&p, all_kp_out) == 0U)
    {
      return 0U;
    }

    while ((*p == ' ') || (*p == '\t'))
    {
      p++;
    }
    if (PID_ParseFloatToken(&p, all_ki_out) == 0U)
    {
      return 0U;
    }

    while ((*p == ' ') || (*p == '\t'))
    {
      p++;
    }
    if (PID_ParseFloatToken(&p, all_kd_out) == 0U)
    {
      return 0U;
    }

    while ((*p == ' ') || (*p == '\t'))
    {
      p++;
    }

    if (*p != '\0')
    {
      return 0U;
    }

    *has_value_out = 1U;
    return 1U;
  }

  if (*p == '\0')
  {
    *has_value_out = 0U;
    return 1U;
  }

  if (PID_ParseFloatToken(&p, value_out) == 0U)
  {
    return 0U;
  }

  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }

  if (*p != '\0')
  {
    return 0U;
  }

  *has_value_out = 1U;
  return 1U;
}

static const char *AppModeToString(AppMode_t mode)
{
  switch (mode)
  {
    case APP_MODE_IDLE:
      return "IDLE";
    case APP_MODE_MELODY:
      return "MELODY";
    case APP_MODE_PID:
      return "PID";
    case APP_MODE_TEMP:
      return "TEMP";
    default:
      return "UNKNOWN";
  }
}

static void UART_PrintCurrentStatus(void)
{
  int32_t sp_x10 = (int32_t)(pid_setpoint_c * 10.0f);
  int32_t temp_x100 = (int32_t)(pid_last_temp_c * 100.0f);
  int32_t out_x10 = (int32_t)(pid_last_output * 10.0f);

  printf("=== STATUS ===\r\n");
  printf("MODE: %s\r\n", AppModeToString(app_mode));
  printf("SP: %ld.%01ldC\r\n",
         (long)(sp_x10 / 10),
         (long)(sp_x10 < 0 ? -(sp_x10 % 10) : (sp_x10 % 10)));
  printf("LAST TEMP: %ld.%02ldC\r\n",
         (long)(temp_x100 / 100),
         (long)(temp_x100 < 0 ? -(temp_x100 % 100) : (temp_x100 % 100)));
  printf("LAST ADC RAW: %u\r\n", (unsigned int)pid_last_adc_raw);
  printf("LAST PWM OUT: %ld.%01ld%%\r\n",
         (long)(out_x10 / 10),
         (long)(out_x10 < 0 ? -(out_x10 % 10) : (out_x10 % 10)));
}

static void UART_ProcessCommand(void)
{
  uint8_t data;

  while (UART_RxQueuePop(&data) != 0U)
  {
    if ((data == '\r') || (data == '\n'))
    {
      if (uart_cmd_idx == 0U)
      {
        continue;
      }

      uart_cmd_buf[uart_cmd_idx] = '\0';

      if (strcmp(uart_cmd_buf, "stop") == 0)
      {
        EnterIdleMode();
        printf("Output stopped\r\n");
      }
      else if (strcmp(uart_cmd_buf, "start") == 0)
      {
        EnterMelodyMode();
        printf("Melody mode started\r\n");
      }
      else if (strcmp(uart_cmd_buf, "temp") == 0)
      {
        EnterTempMode();
        printf("Temp monitor mode. RAW and temperature every 50ms (20Hz).\r\n");
      }
      else if (strcmp(uart_cmd_buf, "pid") == 0)
      {
        int32_t sp_x10;
        EnterPidMode();
        sp_x10 = (int32_t)(pid_setpoint_c * 10.0f);
        printf("PID mode started (SP=%ld.%01ldC)\r\n", (long)(sp_x10 / 10), (long)(sp_x10 < 0 ? -(sp_x10 % 10) : (sp_x10 % 10)));
      }
      else if ((strncmp(uart_cmd_buf, "sp", 2) == 0) && ((uart_cmd_buf[2] == '\0') || (uart_cmd_buf[2] == ' ') || (uart_cmd_buf[2] == '\t') || (uart_cmd_buf[2] == '=')))
      {
        float new_setpoint;
        int32_t sp_x10;

        if (PID_ParseSetpointCommand(uart_cmd_buf, &new_setpoint) != 0U)
        {
          pid_setpoint_c = new_setpoint;
          pid_integral = 0.0f;
          pid_prev_error = 0.0f;
          sp_x10 = (int32_t)(pid_setpoint_c * 10.0f);
          printf("SP updated: %ld.%01ldC\r\n", (long)(sp_x10 / 10), (long)(sp_x10 < 0 ? -(sp_x10 % 10) : (sp_x10 % 10)));
        }
        else
        {
          printf("Usage: sp <tempC> (0~100)\r\n");
        }
      }
      else if ((strncmp(uart_cmd_buf, "coeff", 5) == 0) && ((uart_cmd_buf[5] == '\0') || (uart_cmd_buf[5] == ' ') || (uart_cmd_buf[5] == '\t') || (uart_cmd_buf[5] == '=')))
      {
        uint8_t coeff_target = 0U;
        uint8_t has_value = 0U;
        uint8_t coeff_is_all = 0U;
        float coeff_value = 0.0f;
        float coeff_all_kp = 0.0f;
        float coeff_all_ki = 0.0f;
        float coeff_all_kd = 0.0f;
        int32_t kp_x100;
        int32_t ki_x100;
        int32_t kd_x100;

        if (PID_ParseCoeffCommand(uart_cmd_buf,
                                  &coeff_target,
                                  &coeff_value,
                                  &has_value,
                                  &coeff_is_all,
                                  &coeff_all_kp,
                                  &coeff_all_ki,
                                  &coeff_all_kd) == 0U)
        {
          printf("Usage: coeff | coeff <kp|ki|kd> [value] | coeff all <kp> <ki> <kd> (0~1000)\r\n");
        }
        else
        {
          if (has_value != 0U)
          {
            if (app_mode == APP_MODE_PID)
            {
              printf("Cannot change coeff in PID mode. Change mode first (stop/start/temp).\r\n");
            }
            else if (coeff_is_all != 0U)
            {
              pid_kp = coeff_all_kp;
              pid_ki = coeff_all_ki;
              pid_kd = coeff_all_kd;
              pid_integral = 0.0f;
              pid_prev_error = 0.0f;
              printf("PID coeff updated\r\n");
            }
            else if (coeff_target == 1U)
            {
              pid_kp = coeff_value;
              pid_integral = 0.0f;
              pid_prev_error = 0.0f;
              printf("PID coeff updated\r\n");
            }
            else if (coeff_target == 2U)
            {
              pid_ki = coeff_value;
              pid_integral = 0.0f;
              pid_prev_error = 0.0f;
              printf("PID coeff updated\r\n");
            }
            else if (coeff_target == 3U)
            {
              pid_kd = coeff_value;
              pid_integral = 0.0f;
              pid_prev_error = 0.0f;
              printf("PID coeff updated\r\n");
            }
          }

          kp_x100 = (int32_t)(pid_kp * 100.0f);
          ki_x100 = (int32_t)(pid_ki * 100.0f);
          kd_x100 = (int32_t)(pid_kd * 100.0f);
          printf("COEFF KP=%ld.%02ld KI=%ld.%02ld KD=%ld.%02ld\r\n",
                 (long)(kp_x100 / 100),
                 (long)(kp_x100 < 0 ? -(kp_x100 % 100) : (kp_x100 % 100)),
                 (long)(ki_x100 / 100),
                 (long)(ki_x100 < 0 ? -(ki_x100 % 100) : (ki_x100 % 100)),
                 (long)(kd_x100 / 100),
                 (long)(kd_x100 < 0 ? -(kd_x100 % 100) : (kd_x100 % 100)));
        }
      }
      else if ((strncmp(uart_cmd_buf, "cv", 2) == 0) && ((uart_cmd_buf[2] == '\0') || (uart_cmd_buf[2] == ' ') || (uart_cmd_buf[2] == '\t') || (uart_cmd_buf[2] == '=')))
      {
        const char *p = &uart_cmd_buf[2];

        while ((*p == ' ') || (*p == '\t') || (*p == '='))
        {
          p++;
        }

        if (*p == '\0')
        {
          printf("CV mode: %s (off=text, on=csv)\r\n", (pid_cv_csv_mode != 0U) ? "on" : "off");
        }
        else if ((strcmp(p, "on") == 0) || (strcmp(p, "0n") == 0))
        {
          pid_cv_csv_mode = 1U;
          printf("CV mode set: on (PID report in CSV for serial plotter)\r\n");
        }
        else if (strcmp(p, "off") == 0)
        {
          pid_cv_csv_mode = 0U;
          printf("CV mode set: off (PID report in text)\r\n");
        }
        else
        {
          printf("Usage: cv=on/off (or cv on/off)\r\n");
        }
      }
      else if (strcmp(uart_cmd_buf, "help") == 0)
      {
        printf("Commands: start / stop / temp / pid / sp <tempC> / coeff / cv=on/off / help\r\n");
        printf("PID coeff: coeff, coeff kp <v>, coeff ki <v>, coeff kd <v>, coeff all <kp> <ki> <kd>\r\n");
        printf("PID CV: cv=off(text), cv=on(csv for plotter)\r\n");
        UART_PrintCurrentStatus();
      }
      else
      {
        printf("Unknown cmd: %s\r\n", uart_cmd_buf);
      }

      uart_cmd_idx = 0U;
      continue;
    }

    if (uart_cmd_idx < (UART_CMD_BUF_LEN - 1U))
    {
      uart_cmd_buf[uart_cmd_idx++] = (char)tolower((int)data);
    }
    else
    {
      uart_cmd_idx = 0U;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  EnterMelodyMode();
  (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
  printf("USART2 ready. Commands: start / stop / temp / pid / sp <tempC> / coeff / cv=on/off / help\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 500ms마다 LED 토글
    if (HAL_GetTick() - led_toggle_time >= 500)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      led_toggle_time = HAL_GetTick();
    }

    UART_ProcessCommand();

    if ((app_mode == APP_MODE_MELODY) && (melody_phase == 0U) && (HAL_GetTick() - melody_tick >= BUZZER_NOTE_MS))
    {
      Buzzer_Stop();
      melody_phase = 1U;
      melody_tick = HAL_GetTick();
    }
    else if ((app_mode == APP_MODE_MELODY) && (melody_phase == 1U) && (HAL_GetTick() - melody_tick >= BUZZER_GAP_MS))
    {
      melody_index = (uint8_t)((melody_index + 1U) % 3U);
      Buzzer_SetFrequency(melody_freq_hz[melody_index]);
      melody_phase = 0U;
      melody_tick = HAL_GetTick();
    }

    if ((app_mode == APP_MODE_TEMP) && (HAL_GetTick() - temp_report_tick >= PID_CONTROL_MS))
    {
      float temp_c;
      int32_t temp_x100;

      temp_report_tick = HAL_GetTick();
      temp_c = TMP235_ReadTempC();
      temp_x100 = (int32_t)(temp_c * 100.0f);
      printf("TEMP RAW=%u T=%ld.%02ldC\r\n",
             (unsigned int)pid_last_adc_raw,
             (long)(temp_x100 / 100),
             (long)(temp_x100 < 0 ? -(temp_x100 % 100) : (temp_x100 % 100)));
    }

    if ((app_mode == APP_MODE_PID) && (HAL_GetTick() - pid_last_tick >= PID_CONTROL_MS))
    {
      pid_last_tick = HAL_GetTick();
      PID_Update();
    }

    if ((app_mode == APP_MODE_PID) && (HAL_GetTick() - pid_report_tick >= PID_CONTROL_MS))
    {
      int32_t temp_x100;
      int32_t sp_x10;
      int32_t out_x10;

      pid_report_tick = HAL_GetTick();
      temp_x100 = (int32_t)(pid_last_temp_c * 100.0f);
      sp_x10 = (int32_t)(pid_setpoint_c * 10.0f);
      out_x10 = (int32_t)(pid_last_output * 10.0f);

      if (pid_cv_csv_mode != 0U)
      {
        printf("%u,%ld.%02ld,%ld.%01ld,%ld.%01ld\r\n",
               (unsigned int)pid_last_adc_raw,
               (long)(temp_x100 / 100),
               (long)(temp_x100 < 0 ? -(temp_x100 % 100) : (temp_x100 % 100)),
               (long)(sp_x10 / 10),
               (long)(sp_x10 < 0 ? -(sp_x10 % 10) : (sp_x10 % 10)),
               (long)(out_x10 / 10),
               (long)(out_x10 < 0 ? -(out_x10 % 10) : (out_x10 % 10)));
      }
      else
      {
        printf("PID RAW=%u T=%ld.%02ldC SP=%ld.%01ldC OUT=%ld.%01ld%%\r\n",
               (unsigned int)pid_last_adc_raw,
               (long)(temp_x100 / 100),
               (long)(temp_x100 < 0 ? -(temp_x100 % 100) : (temp_x100 % 100)),
               (long)(sp_x10 / 10),
               (long)(sp_x10 < 0 ? -(sp_x10 % 10) : (sp_x10 % 10)),
               (long)(out_x10 / 10),
               (long)(out_x10 < 0 ? -(out_x10 % 10) : (out_x10 % 10)));
      }
    }
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    (void)UART_RxQueuePush(uart_rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
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
