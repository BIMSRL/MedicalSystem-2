# Servo-SG90 기능/설정 상세 문서

이 문서는 readme.md 내용을 기반으로, 실제 소스 코드 스니펫과 함께 설정 이유를 설명합니다.
대상 프로젝트는 STM32F411에서 다음 기능을 동시에 다룹니다.

- TIM3 TRGO를 이용한 ADC 외부 트리거 샘플링
- 3채널 ADC 데이터 큐잉
- CMSIS-DSP FIR 필터 적용
- TIM4 PWM 기반 SG90 서보 제어
- UART 명령 입력 기반 제어 및 로그 출력

문서 읽는 방법:
- 먼저 1~3장에서 전체 구조와 타이밍 설계를 이해합니다.
- 이후 4~8장에서 함수 단위 구현 이유를 확인합니다.
- 마지막으로 9~11장을 통해 실제 운영/디버깅 관점에서 점검합니다.

## 1. 전체 구조와 설계 의도

### 1.1 왜 타이머를 분리했는가

- TIM3: 샘플링 타이밍 전용 (100Hz)
- TIM4: 서보 PWM 전용 (50Hz)

분리 이유:
- ADC 샘플링 주기와 PWM 주기를 독립적으로 유지 가능
- 기능 간 간섭 감소
- 타이머 파라미터 조정 시 영향 범위를 명확히 분리

### 1.2 왜 큐를 사용하는가

ADC 인터럽트에서는 빠르게 데이터를 저장만 하고 빠져나와야 합니다. 
UART 출력, FIR 계산 같은 상대적으로 무거운 처리는 메인 루프에서 수행하는 것이 안정적입니다.

- ISR(생산자): ADC 프레임 push
- Main loop(소비자): 프레임 pop -> FIR -> UART 출력

이 구조는 실시간성(인터럽트 응답성)과 처리 안정성을 동시에 확보합니다.

### 1.3 데이터 경로 요약

- 센서 입력(아날로그) -> ADC1 3채널 변환
- ADC 인터럽트에서 3채널 프레임 구성 -> 큐 저장
- 메인 루프에서 프레임 pop -> FIR 처리 -> CSV 출력
- 동시에 UART 입력(0~100)을 받아 PWM 듀티 갱신

핵심은 "샘플링"과 "출력/연산"을 분리한 점입니다. 이 분리가 없으면 UART 출력 지연이 샘플링 타이밍을 흔들 수 있습니다.

## 2. ADC 외부 트리거(TIM3 -> ADC1)

### 2.1 핵심 설정 스니펫

```c
hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;  // TIM3 TRGO의 상승엣지에서 ADC를 시작
hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;         // ADC 외부 트리거 소스로 TIM3 TRGO 선택
```

```c
htim3.Init.Prescaler = 8399;                           // 타이머 카운터 클럭을 10kHz로 분주
htim3.Init.Period = 99;                                // 100카운트마다 update 이벤트 발생(10ms)
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  // update 이벤트를 TRGO로 출력
```

### 2.2 설정 이유

- ADC는 소프트웨어 호출이 아니라 타이머 이벤트로 시작되므로 샘플 간격이 균일해집니다.
- TRGO_UPDATE를 써야 TIM3 update event가 실제 트리거 신호로 전달됩니다.
- 이전 문제 사례처럼 TRGO 설정이 RESET이면 ADC 변환이 시작되지 않을 수 있습니다.

추가 설명:
- EOC를 single conversion으로 설정하면 rank마다 콜백이 발생하고, 이 콜백에서 rank index를 증가시키며 프레임을 완성할 수 있습니다.
- 타이머 기반 트리거는 제어 루프/로그 출력 부하와 무관하게 샘플 타이밍이 고정된다는 장점이 있습니다.

### 2.3 주기 계산 근거

- TIM3 입력 클럭 가정: 84MHz
- 카운터 클럭: 84MHz / (8399 + 1) = 10kHz
- 업데이트 주기: (99 + 1) / 10kHz = 10ms
- 결과: 100Hz

## 3. TIM4 PWM 서보 제어

### 3.1 TIM4 설정 스니펫

```c
htim4.Instance = TIM4;              // PWM 출력에 사용할 TIM4 인스턴스 선택
htim4.Init.Prescaler = 83;          // 84MHz/(83+1)=1MHz, 1tick=1us로 설정
htim4.Init.Period = 19999;          // 20000tick=20ms 주기(50Hz)

HAL_TIM_Base_Init(&htim4);          // TIM4 베이스 카운터 초기화
HAL_TIM_PWM_Init(&htim4);           // TIM4를 PWM 모드로 활성화

sConfigOC.OCMode = TIM_OCMODE_PWM1;                            // PWM1 출력 모드 선택
sConfigOC.Pulse = 0;                                           // 초기 CCR=0으로 시작
HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);  // CH1에 PWM 설정 반영
```

### 3.2 설정 이유

- 1MHz 카운터(1 tick = 1us)로 맞추면 서보 펄스폭 계산이 직관적입니다.
- SG90은 일반적으로 20ms 주기(50Hz), 1.0ms~2.0ms 펄스폭을 사용합니다.
- Pulse(= CCR) 값만 바꿔도 즉시 듀티 반영이 가능해 제어 코드가 단순합니다.

추가 설명:
- 서보는 듀티 퍼센트보다 절대 펄스폭(1~2ms)이 더 직접적인 제어 기준입니다.
- 따라서 본 코드처럼 CCR 값을 us 단위로 맞추는 방식이 튜닝과 디버깅에 유리합니다.

### 3.3 주기 계산 근거

- TIM4 입력 클럭 가정: 84MHz
- 카운터 클럭: 84MHz / (83 + 1) = 1MHz
- PWM 주기: (19999 + 1)us = 20000us = 20ms = 50Hz

## 4. PWM 저장/적용 함수 상세

### 4.1 함수 스니펫

```c
static void SetPwmDutyPercent(uint8_t duty_percent)
{
  uint32_t pulse;         // 최종 CCR에 기록할 펄스폭(us)
  uint32_t pulse_range;   // 최소~최대 펄스폭 범위

  if (duty_percent > 100U)
  {
    duty_percent = 100U;  // 입력값 상한 보호(클램프)
  }

  pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;                     // 2000-1000=1000us
  pulse = SERVO_MIN_PULSE_US + ((pulse_range * (uint32_t)duty_percent) / 100U); // 0~100% -> 1000~2000us
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);                       // TIM4 CH1 CCR 갱신

  g_pwm_duty_percent = duty_percent;  // 현재 적용 퍼센트 상태 저장
  printf("Servo set: %u%% (%luus)\n",
         g_pwm_duty_percent,          // 적용된 퍼센트 출력
         (unsigned long)pulse);       // 계산된 us 펄스폭 출력
}
```

### 4.2 동작 이유

- 상한 클램프: 잘못된 입력으로 인한 제어 범위 이탈 방지
- 선형 매핑: 사용자 입력(0~100)을 서보 제어 펄스(1000~2000us)로 직관적으로 변환
- 상태 저장: 마지막 적용 상태를 유지해 디버깅 및 상태 표시에 활용
- 로그 출력: 즉시 반영 여부를 UART에서 확인 가능

추가 설명:
- 이 함수에 제어 정책을 집중하면, 향후 지수형 맵핑/데드밴드 같은 요구가 생겨도 수정 지점이 한곳으로 고정됩니다.
- 출력 문자열을 영어로 유지한 것은 로그 파서/외부 도구 연동을 쉽게 하기 위한 선택입니다.

## 5. UART 입력 처리와 제어 연결

### 5.1 스니펫

```c
if (HAL_UART_Receive(&huart2, &rx_char, 1U, 0U) != HAL_OK)  // non-blocking 1바이트 수신
{
  return;                                                    // 새 입력 없음
}

if ((rx_char == '\r') || (rx_char == '\n'))                // Enter 입력으로 명령 확정
{
  int value;                                                 // 파싱된 숫자값

  if (input_len == 0U)                                       // 빈 엔터 입력은 무시
  {
    return;
  }

  input_buf[input_len] = '\0';                              // C 문자열 종료
  value = atoi(input_buf);                                   // 문자열 -> 정수 변환

  if ((value >= 0) && (value <= 100))                        // 0~100 유효 범위 확인
  {
    SetPwmDutyPercent((uint8_t)value);                       // 유효하면 PWM 적용
  }
  else
  {
    printf("Enter servo ratio in range 0~100\n");          // 범위 오류 안내
  }

  input_len = 0U;                                            // 버퍼 인덱스 초기화
  return;                                                    // 한 줄 처리 종료
}
```

### 5.2 설정 이유

- non-blocking 수신(timeout=0): 메인 루프 정지 없이 입력 확인
- 엔터 기반 확정 입력: 문자열 파싱 흐름 단순화
- 범위 검증 후 호출: PWM 적용 로직과 입력 검증 로직을 분리해 유지보수성 향상

추가 설명:
- 숫자 외 문자를 빠르게 거르는 현재 정책은 단순/안전하지만, 향후 명령형 인터페이스(fc=xx 등)를 넣으려면 파서를 확장해야 합니다.
- 입력 버퍼 크기는 현재 사용 시나리오(0~100)에는 충분하지만, 명령형 확장 시 길이 재검토가 필요합니다.

## 6. ADC 프레임 큐와 인터럽트 처리

### 6.1 인터럽트 스니펫

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t adc_value;   // 이번 rank에서 변환 완료된 ADC 값
  uint8_t rank_index;   // 현재 rank 위치(0,1,2)

  if (hadc->Instance != ADC1)
  {
    return;             // ADC1 이외 인터럽트는 무시
  }

  adc_value = (uint16_t)HAL_ADC_GetValue(hadc);  // 데이터 레지스터에서 값 읽기
  rank_index = g_adc_rank_index;                 // 전역 rank 인덱스 로드

  if (rank_index < ADC_CHANNEL_COUNT)
  {
    g_adc_current_frame.channel[rank_index] = adc_value;  // 해당 rank 위치에 저장
  }

  rank_index++;                              // 다음 rank로 이동
  if (rank_index >= ADC_CHANNEL_COUNT)
  {
    rank_index = 0U;                                           // 3채널 완성 후 rank 초기화
    (void)AdcQueue_Push(&g_adc_queue, &g_adc_current_frame);   // 완성 프레임 큐에 push
    g_adc_total_frames++;                                      // 누적 프레임 카운트 증가
  }

  g_adc_rank_index = rank_index;  // 다음 콜백을 위한 rank 상태 저장
}
```

### 6.2 설정 이유

- rank 기반 축적: 3개 채널을 1프레임으로 묶어 후처리 단순화
- ISR 내 최소 작업: 계산/출력은 메인 루프로 이동
- 프레임 카운트 유지: 샘플링 상태 모니터링 지표 제공

추가 설명:
- rank index 방식은 DMA가 없는 인터럽트 모드에서 다채널 순서를 명확히 유지하기 좋습니다.
- ISR 안에서 printf를 호출하지 않는 점이 안정성 측면에서 매우 중요합니다.

### 6.3 큐 overflow 정책 이유

- 큐 full 시 가장 오래된 프레임 폐기 + 최신 프레임 저장
- 실시간 스트리밍 관점에서 최신 데이터 유지가 유리

추가 설명:
- 만약 데이터 로깅(누락 최소화)이 목표라면 정책을 반대로 설계하거나, 생산/소비 속도 균형을 DMA/RTOS로 보완해야 합니다.

## 7. CMSIS-DSP FIR 적용

### 7.1 초기화 스니펫

```c
arm_fir_init_f32(&g_fir[0], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[0], 1U);  // CH0 FIR 상태 초기화
arm_fir_init_f32(&g_fir[1], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[1], 1U);  // CH1 FIR 상태 초기화
arm_fir_init_f32(&g_fir[2], FIR_TAP_COUNT, g_fir_coeffs, g_fir_state[2], 1U);  // CH2 FIR 상태 초기화
```

### 7.2 실행 스니펫

```c
arm_fir_f32(&g_fir[0], &fir_input[0], &fir_output[0], 1U);  // CH0 1샘플 FIR 연산
arm_fir_f32(&g_fir[1], &fir_input[1], &fir_output[1], 1U);  // CH1 1샘플 FIR 연산
arm_fir_f32(&g_fir[2], &fir_input[2], &fir_output[2], 1U);  // CH2 1샘플 FIR 연산
```

### 7.3 설정 이유

- 채널별 독립 상태 버퍼 사용으로 필터 메모리 간섭 방지
- 동일 컷오프 필터를 3채널에 대칭 적용해 채널 비교가 쉬움
- 원본 ADC와 FIR 결과를 동시에 출력해 필터 효과를 즉시 확인 가능

### 7.4 FIR 계수 해석(현재 설정)

- tap 수: 11
- 컷오프: 20Hz
- 샘플링: 100Hz

해석 포인트:
- tap 수가 커질수록 주파수 분리 성능은 좋아지지만 연산량과 지연이 증가합니다.
- 현재 값(11tap)은 실시간 UART 출력과 병행하는 교육/검증 목적에서 균형점으로 선택되었습니다.

## 8. printf float 출력 설정

### 8.1 CMake 스니펫

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")   # nano libc 사용(코드 크기 최적화)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")      # printf %.xf 실수 포맷 강제 링크
```

### 8.2 설정 이유

- newlib-nano 환경에서는 기본적으로 printf의 float 포맷이 비활성일 수 있습니다.
- -u _printf_float를 추가해야 %.2f 같은 실수 출력이 실제로 표시됩니다.
- 대가로 코드 크기(FLASH 사용량)가 증가하므로 배포 빌드에서는 필요성 판단이 필요합니다.

추가 설명:
- 개발 단계에서는 가시성을 위해 float 출력이 유용합니다.
- 릴리즈 단계에서는 정수 스케일링 출력으로 대체해 코드 크기를 줄이는 전략도 고려할 수 있습니다.

## 9. 초기화 및 런타임 순서

### 9.1 초기화 순서

1. HAL/Clock init
2. GPIO/UART/ADC/TIM3/TIM4 init
3. TIM4 PWM start
4. FIR init, 큐 init, 서보 초기값 적용
5. TIM3 start
6. ADC interrupt start

### 9.2 런타임 순서

1. UART 입력 처리(ProcessPwmInput)
2. 큐 pop 시도
3. pop 성공 시 FIR 처리
4. CSV 출력(ch0,ch1,ch2,fir_ch0,fir_ch1,fir_ch2)

보충 설명:
- 루프는 "입력 처리 -> 데이터 소비 -> 출력" 순서를 유지해 조작 반응성과 데이터 가시성을 동시에 만족합니다.
- pop 실패(큐 비어있음) 시에는 불필요한 연산/출력 없이 즉시 다음 루프로 넘어갑니다.

## 10. 디버깅 체크리스트

### 10.1 ADC 데이터가 안 나올 때

- TIM3 TRGO가 TIM_TRGO_UPDATE인지 확인
- ADC external trigger source가 TIM3 TRGO인지 확인
- ADC IRQ enable 여부 확인
- UART 모니터 baud 115200, 8N1 확인

### 10.2 실수 출력이 비어 보일 때

- 링크 옵션에 -u _printf_float 포함 여부 확인
- 새 바이너리 재플래시 여부 확인

### 10.3 샘플 누락/지연이 보일 때

- UART 출력량 과다 여부 확인
- 큐 overflow_count 증가 여부 확인
- 필요 시 출력 주기 조절 또는 DMA 전송 검토

### 10.4 점검 우선순위 추천

1. 초기 메시지 출력 여부 확인(통신 경로 정상 여부)
2. ADC 콜백 카운터 증가 여부 확인(트리거 경로 정상 여부)
3. 큐 count/overflow 추이 확인(처리량 병목 여부)
4. FIR 출력의 변화폭 확인(필터 계수/입력 신호 타당성)

## 11. 향후 확장 제안

- 런타임 FIR 컷오프 변경 명령(fc=xx) 추가
- 채널별 서로 다른 FIR 계수 적용
- ADC DMA + double buffer로 CPU 부하 감소
- CSV 헤더 출력 및 로깅 포맷 표준화

## 12. 운영 관점 권장사항

- 개발 빌드(Debug): float printf 유지, 상세 로그 유지
- 배포 빌드(Release): 로그 축소, 필요 시 float 출력 제거
- 성능 검증 시: overflow_count, frame 처리율, UART 대역 사용률을 함께 기록

이와 같은 운영 구분을 적용하면, 같은 코드 베이스로 개발 편의성과 제품 제약을 동시에 관리할 수 있습니다.
