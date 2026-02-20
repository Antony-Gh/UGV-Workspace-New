/**
 * @file    main.c
 * @brief   UGV Motor Controller — Application entry point.
 *
 * Initialises all peripherals (clocks, GPIO, timers, FDCAN, ADC),
 * then starts the module layer (Safety, Fault, Motors, CAN, Config,
 * Lights, Telemetry, Control Loop).
 *
 * Super-loop handles non-time-critical tasks:
 *   - Telemetry transmission (50/100/200 ms staggered)
 *   - LED pattern updates (~20 Hz)
 *   - Future: CAN diagnostics, watchdog refresh
 *
 * Time-critical work (1 kHz) is inside the TIM6 ISR:
 *   - Encoder reading
 *   - Velocity command latch (speed-clamped)
 *   - Odometry accumulation
 *   - PID control
 *   - PWM output
 *   - Fault detection
 *   - Safety FSM
 *
 * MCU: STM32H743ZI (480 MHz, Cortex-M7)
 *
 * @author  UGV Firmware Team
 */

#include "stm32h7xx_hal.h"

/* ── Module headers ─── */
#include "can_comm.h"
#include "config.h"
#include "control_loop.h"
#include "fault.h"
#include "lights.h"
#include "motor.h"
#include "pid.h"
#include "safety.h"
#include "telemetry.h"

/* ──────────────────── HAL Peripheral Handles ──────────────────── */

/* PWM timers (20 kHz, 4 channels total) */
TIM_HandleTypeDef htim1; /* FL + FR PWM         */
TIM_HandleTypeDef htim8; /* RL + RR PWM         */

/* Encoder timers (quadrature mode) */
TIM_HandleTypeDef htim2; /* FL encoder           */
TIM_HandleTypeDef htim3; /* FR encoder           */
TIM_HandleTypeDef htim4; /* RL encoder           */
TIM_HandleTypeDef htim5; /* RR encoder           */

/* Control loop timer (1 kHz ISR) */
TIM_HandleTypeDef htim6;

/* CAN-FD */
FDCAN_HandleTypeDef hfdcan1;

/* ADC for battery monitoring */
ADC_HandleTypeDef hadc1;

/* ──────────────────── Global Module State ─────────────────────── */

Motor_t g_motors[MOTOR_COUNT];
Fault_State_t g_fault_state;

/* ──────────────────── Forward Declarations ────────────────────── */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void Motor_HW_Config(void);

/* ============================================================== */
/*                          main()                                 */
/* ============================================================== */

int main(void) {
  /* ── HAL & system init ─────────────────────────────────────── */
  HAL_Init();
  SystemClock_Config();

  /* ── Peripheral init ───────────────────────────────────────── */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();

  /* ── Motor hardware descriptor setup ───────────────────────── */
  Motor_HW_Config();

  /* ── Module initialisation (order matters) ─────────────────── */
  Safety_Init();
  Fault_Init(&g_fault_state);
  Config_Init(); /* Sets default profile (IGVC) + fault timeout    */
  Motor_Init(g_motors);
  CAN_Comm_Init(&hfdcan1);
  Lights_Init();
  Telemetry_Init(&hadc1);

  /* ── Signal that all init is complete ──────────────────────── */
  Safety_SetReady();

  /* ── Start control loop (TIM6 interrupt — must be last) ──── */
  ControlLoop_Init(&htim6, g_motors, &g_fault_state);

  /* ============================================================ */
  /*                    Main Super-loop                            */
  /* ============================================================ */

  while (1) {
    /* Telemetry: motor (50 ms), odometry (100 ms), battery (200 ms) */
    Telemetry_Update(g_motors, &g_fault_state);

    /* LED pattern update (~20 Hz effective with main loop speed) */
    Lights_Update();

    /* Future: IWDG refresh, CAN diagnostics, etc. */
  }
}

/* ============================================================== */
/*                   HAL Callback Routing                          */
/* ============================================================== */

/**
 * @brief  Timer period-elapsed callback — routes TIM6 to control loop.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  ControlLoop_TimerCallback(htim);
}

/**
 * @brief  FDCAN RX FIFO 0 callback — routes to CAN comm.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  CAN_Comm_RxCallback(hfdcan, RxFifo0ITs);
}

/**
 * @brief  GPIO EXTI callback — handles both E-STOP inputs.
 * @param  GPIO_Pin  Pin that triggered the interrupt.
 *
 * Mechanical E-STOP : PC13 (GPIO_PIN_13, EXTI15_10_IRQn, priority 0,0)
 * Wireless E-STOP   : PG0  (GPIO_PIN_0,  EXTI0_IRQn,     priority 0,1)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ESTOP_GPIO_PIN || GPIO_Pin == ESTOP_WIRELESS_GPIO_PIN) {
    Safety_ESTOP_IRQ(g_motors);
  }
}

/* ============================================================== */
/*                  Peripheral Configuration                       */
/* ============================================================== */

/**
 * @brief  System clock configuration for STM32H743ZI.
 *         480 MHz SYSCLK, 240 MHz AHB, 120 MHz APB1/APB2.
 */
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Supply configuration — direct SMPS (Nucleo-144 default) */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /* HSE oscillator → PLL1 → 480 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; /* Nucleo ST-Link 8 MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1U;
  RCC_OscInitStruct.PLL.PLLN = 120U;
  RCC_OscInitStruct.PLL.PLLP = 2U;
  RCC_OscInitStruct.PLL.PLLQ = 20U; /* FDCAN kernel clock = 48 MHz */
  RCC_OscInitStruct.PLL.PLLR = 2U;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0U;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;  /* 240 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; /* 120 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; /* 120 MHz */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/**
 * @brief  GPIO initialisation.
 *
 *  Configured pins:
 *    Motor direction   : PA0–PA3  (push-pull output)
 *    Motor enable      : PA8–PA11 (push-pull output)
 *    Overcurrent input : PB4–PB7  (input, pull-up)
 *    Mechanical E-STOP : PC13     (EXTI falling-edge, pull-up, priority 0,0)
 *    Wireless E-STOP   : PG0      (EXTI falling-edge, pull-up, priority 0,1)
 *    Contactor relay   : PD4      (push-pull output)
 *    Isolation relay   : PD5      (push-pull output)
 *    Debug toggle      : PB0      (push-pull output)
 *    LED Red           : PF0      (push-pull output)
 *    LED Green         : PF1      (push-pull output)
 *    LED Blue          : PF2      (push-pull output)
 *    Laser pointer     : PF3      (push-pull output)
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* ── Motor direction: PA0–PA3 (push-pull output) ─────────── */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── Motor enable: PA8–PA11 (push-pull output) ──────────── */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── Overcurrent inputs: PB4–PB7 (input, pull-up) ──────── */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ── Debug toggle: PB0 (push-pull output) ────────────────── */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ── Mechanical E-STOP: PC13, EXTI falling-edge ──────────── */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI15_10 — highest priority for mechanical E-STOP */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* ── Wireless E-STOP: PG0, EXTI falling-edge ────────────── */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI0 — slightly lower sub-priority than mechanical */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* ── Contactor relay: PD4 (push-pull output, initial LOW) ── */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /* ── Isolation relay: PD5 (push-pull output, initial LOW) ── */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /* ── LED outputs: PF0–PF3 (push-pull output, initial LOW) ── */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                    GPIO_PIN_RESET);
}

/**
 * @brief  TIM1 initialisation — PWM for FL + FR motors (20 kHz).
 */
static void MX_TIM1_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U; /* 240 MHz timer clock (APB2 × 2) */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_ARR; /* 11999 → 20 kHz */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1); /* FL */
  HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2); /* FR */
}

/**
 * @brief  TIM8 initialisation — PWM for RL + RR motors (20 kHz).
 */
static void MX_TIM8_Init(void) {
  __HAL_RCC_TIM8_CLK_ENABLE();

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0U;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = MOTOR_PWM_ARR;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0U;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim8);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_1); /* RL */
  HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_2); /* RR */
}

/**
 * @brief  TIM2 encoder mode — Front-Left motor (32-bit).
 */
static void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFU; /* 32-bit */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef enc = {0};
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter = 0x0FU;
  enc.IC2Polarity = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter = 0x0FU;

  HAL_TIM_Encoder_Init(&htim2, &enc);
}

/**
 * @brief  TIM3 encoder mode — Front-Right motor (16-bit).
 */
static void MX_TIM3_Init(void) {
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFU; /* 16-bit */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef enc = {0};
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter = 0x0FU;
  enc.IC2Polarity = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter = 0x0FU;

  HAL_TIM_Encoder_Init(&htim3, &enc);
}

/**
 * @brief  TIM4 encoder mode — Rear-Left motor (16-bit).
 */
static void MX_TIM4_Init(void) {
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0U;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFFU;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef enc = {0};
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter = 0x0FU;
  enc.IC2Polarity = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter = 0x0FU;

  HAL_TIM_Encoder_Init(&htim4, &enc);
}

/**
 * @brief  TIM5 encoder mode — Rear-Right motor (32-bit).
 */
static void MX_TIM5_Init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0U;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFFU; /* 32-bit */
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef enc = {0};
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter = 0x0FU;
  enc.IC2Polarity = TIM_ICPOLARITY_RISING;
  enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC2Filter = 0x0FU;

  HAL_TIM_Encoder_Init(&htim5, &enc);
}

/**
 * @brief  TIM6 initialisation — 1 kHz control loop interrupt.
 *         APB1 timer clock = 240 MHz.
 *         PSC = 23, ARR = 9999 → 240 000 000 / (24 × 10000) = 1000 Hz.
 */
static void MX_TIM6_Init(void) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim6);

  /* TIM6 interrupt — higher priority than CAN, lower than E-STOP */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  FDCAN1 initialisation — 1 Mbps nominal / 5 Mbps data.
 *
 *  FDCAN kernel clock: PLL1Q = 48 MHz.
 *
 *  Nominal:  48 MHz / 3 = 16 tq → 1 Mbps
 *    Seg1 = 13, Seg2 = 2, SJW = 2   (SP = 87.5%)
 *
 *  Data:     48 MHz / 1 = 48 tq → 5 Mbps? No—
 *    Actually  48 / 1 / (7+2) = 5.33 Mbps — adjusted:
 *    PSC=1, Seg1=7, Seg2=2, 48/1/(1+7+2) = 4.8 Mbps
 *    For exact 5 Mbps, use PLL1Q = 40 MHz or adjust.
 *    Using closest achievable with 48 MHz kernel clock:
 *    PSC=1, Seg1=6, Seg2=1, SJW=1 → 48/1/(1+6+1) = 6 Mbps — too fast.
 *    PSC=1, Seg1=7, Seg2=2, SJW=2 → 48/1/(1+7+2) = 4.8 Mbps — close enough.
 *
 *  Note: Adjust PLL1Q for exact 5 Mbps if required.
 */
static void MX_FDCAN1_Init(void) {
  __HAL_RCC_FDCAN_CLK_ENABLE();

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  /* Nominal bit timing: 1 Mbps */
  hfdcan1.Init.NominalPrescaler = 3U;
  hfdcan1.Init.NominalSyncJumpWidth = 2U;
  hfdcan1.Init.NominalTimeSeg1 = 13U;
  hfdcan1.Init.NominalTimeSeg2 = 2U;

  /* Data bit timing: ~4.8 Mbps (closest to 5 Mbps with 48 MHz kernel) */
  hfdcan1.Init.DataPrescaler = 1U;
  hfdcan1.Init.DataSyncJumpWidth = 2U;
  hfdcan1.Init.DataTimeSeg1 = 7U;
  hfdcan1.Init.DataTimeSeg2 = 2U;

  /* Message RAM configuration */
  hfdcan1.Init.StdFiltersNbr = 4U; /* Filters 0–3 */
  hfdcan1.Init.ExtFiltersNbr = 0U;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8U;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0U;
  hfdcan1.Init.RxBuffersNbr = 0U;
  hfdcan1.Init.TxEventsNbr = 0U;
  hfdcan1.Init.TxBuffersNbr = 0U;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 4U;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;

  HAL_FDCAN_Init(&hfdcan1);

  /* FDCAN1 interrupt — lower priority than TIM6 */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  ADC1 initialisation for battery monitoring.
 *
 *  Channels:
 *    ADC_CHANNEL_4  (PA4) — Battery voltage (via voltage divider)
 *    ADC_CHANNEL_5  (PA5) — Battery current (ACS712 sensor)
 *    ADC_CHANNEL_6  (PA6) — Motor/MCU temperature (LM35 or similar)
 *
 *  Single conversion mode — channels are selected at read time.
 */
static void MX_ADC1_Init(void) {
  __HAL_RCC_ADC12_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; /* ~60 MHz / 4 = 15 MHz */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; /* Single channel at a time */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;

  HAL_ADC_Init(&hadc1);

  /* Run calibration */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

/**
 * @brief  Populate motor hardware descriptors.
 *
 *  Motor mapping:
 *    [0] FL — TIM1_CH1 PWM, TIM2 encoder, PA0 dir, PA8  en, PB4 OC
 *    [1] FR — TIM1_CH2 PWM, TIM3 encoder, PA1 dir, PA9  en, PB5 OC
 *    [2] RL — TIM8_CH1 PWM, TIM4 encoder, PA2 dir, PA10 en, PB6 OC
 *    [3] RR — TIM8_CH2 PWM, TIM5 encoder, PA3 dir, PA11 en, PB7 OC
 */
static void Motor_HW_Config(void) {
  /* ── FL ────────────────────────────────────────────────────── */
  g_motors[0].hw.pwm_timer = &htim1;
  g_motors[0].hw.pwm_channel = TIM_CHANNEL_1;
  g_motors[0].hw.enc_timer = &htim2;
  g_motors[0].hw.dir_port = GPIOA;
  g_motors[0].hw.dir_pin = GPIO_PIN_0;
  g_motors[0].hw.en_port = GPIOA;
  g_motors[0].hw.en_pin = GPIO_PIN_8;
  g_motors[0].hw.oc_port = GPIOB;
  g_motors[0].hw.oc_pin = GPIO_PIN_4;

  /* ── FR ────────────────────────────────────────────────────── */
  g_motors[1].hw.pwm_timer = &htim1;
  g_motors[1].hw.pwm_channel = TIM_CHANNEL_2;
  g_motors[1].hw.enc_timer = &htim3;
  g_motors[1].hw.dir_port = GPIOA;
  g_motors[1].hw.dir_pin = GPIO_PIN_1;
  g_motors[1].hw.en_port = GPIOA;
  g_motors[1].hw.en_pin = GPIO_PIN_9;
  g_motors[1].hw.oc_port = GPIOB;
  g_motors[1].hw.oc_pin = GPIO_PIN_5;

  /* ── RL ────────────────────────────────────────────────────── */
  g_motors[2].hw.pwm_timer = &htim8;
  g_motors[2].hw.pwm_channel = TIM_CHANNEL_1;
  g_motors[2].hw.enc_timer = &htim4;
  g_motors[2].hw.dir_port = GPIOA;
  g_motors[2].hw.dir_pin = GPIO_PIN_2;
  g_motors[2].hw.en_port = GPIOA;
  g_motors[2].hw.en_pin = GPIO_PIN_10;
  g_motors[2].hw.oc_port = GPIOB;
  g_motors[2].hw.oc_pin = GPIO_PIN_6;

  /* ── RR ────────────────────────────────────────────────────── */
  g_motors[3].hw.pwm_timer = &htim8;
  g_motors[3].hw.pwm_channel = TIM_CHANNEL_2;
  g_motors[3].hw.enc_timer = &htim5;
  g_motors[3].hw.dir_port = GPIOA;
  g_motors[3].hw.dir_pin = GPIO_PIN_3;
  g_motors[3].hw.en_port = GPIOA;
  g_motors[3].hw.en_pin = GPIO_PIN_11;
  g_motors[3].hw.oc_port = GPIOB;
  g_motors[3].hw.oc_pin = GPIO_PIN_7;
}

/* ============================================================== */
/*                     IRQ Handler Stubs                           */
/* ============================================================== */

/*
 * These IRQ handlers forward to HAL.  In a CubeMX-generated project,
 * they would be in stm32h7xx_it.c — included here for completeness.
 */

void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }

void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); }

void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
