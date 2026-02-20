/**
 * @file    main.c
 * @brief   UGV firmware entry point — STM32H743ZI bare-metal.
 *
 * Initialisation sequence:
 *   1. HAL_Init()
 *   2. SystemClock_Config()  — 480 MHz via PLL1
 *   3. GPIO init             — E-STOP, contactor, direction, enable, debug
 *   4. Timer init            — PWM (TIM1/TIM8), encoders (TIM2-5), loop (TIM6)
 *   5. FDCAN init            — FDCAN1 at 1/5 Mbps
 *   6. Module init           — Safety, Motors, Fault, CAN, Telemetry, Control
 * Loop
 *   7. Safety_SetReady()
 *
 * Main loop:
 *   - Telemetry_Update()    — 50 ms CAN TX
 *   - Optional UART debug   — compile with -DDEBUG=1
 *   - __WFI()               — idle until next interrupt
 *
 * All time-critical work runs in the TIM6 1 kHz ISR.
 *
 * @author  UGV Firmware Team
 */

/* ──────────────────── Includes ────────────────────────────────── */

#include "can_comm.h"
#include "control_loop.h"
#include "fault.h"
#include "motor.h"
#include "pid.h"
#include "safety.h"
#include "stm32h7xx_hal.h"
#include "telemetry.h"
#include <string.h>


/* ──────────────────── HAL Handles ─────────────────────────────── */

/* PWM timers (4 channels total across 2 timers) */
TIM_HandleTypeDef htim1; /* PWM: FL (CH1), FR (CH2) */
TIM_HandleTypeDef htim8; /* PWM: RL (CH1), RR (CH2) */

/* Encoder timers (one per motor) */
TIM_HandleTypeDef htim2; /* Encoder: FL */
TIM_HandleTypeDef htim3; /* Encoder: FR */
TIM_HandleTypeDef htim4; /* Encoder: RL */
TIM_HandleTypeDef htim5; /* Encoder: RR */

/* Control loop timer */
TIM_HandleTypeDef htim6; /* 1 kHz update interrupt */

/* CAN-FD */
FDCAN_HandleTypeDef hfdcan1;

/* UART for debug (optional) */
#ifdef DEBUG
UART_HandleTypeDef huart3;
#endif

/* ──────────────────── Global State ────────────────────────────── */

/** 4-wheel motor array — hardware config + runtime state. */
Motor_t g_motors[MOTOR_COUNT];

/** Global fault state — shared between control loop & main. */
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
static void Motor_HW_Config(void);
#ifdef DEBUG
static void MX_USART3_UART_Init(void);
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════ */

int main(void) {
  /* ── 1. HAL & System Clock ───────────────────────────────── */
  HAL_Init();
  SystemClock_Config();

  /* ── 2. Peripheral Init ──────────────────────────────────── */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
#ifdef DEBUG
  MX_USART3_UART_Init();
#endif

  /* ── 3. Module Init ──────────────────────────────────────── */
  Safety_Init();
  Fault_Init(&g_fault_state);

  /* Assign hardware descriptors to motor array */
  Motor_HW_Config();
  Motor_Init(g_motors);

  CAN_Comm_Init(&hfdcan1);
  Telemetry_Init();

  /* Start 1 kHz control loop (must be last — begins ISR firing) */
  ControlLoop_Init(&htim6, g_motors, &g_fault_state);

  /* ── 4. Enter READY state ────────────────────────────────── */
  Safety_SetReady();

  /* ═════════════════════════════════════════════════════════════
   *  SUPER-LOOP
   *  Only non-time-critical work runs here.
   * ═════════════════════════════════════════════════════════════ */
  while (1) {
    /* 50 ms periodic telemetry over CAN */
    Telemetry_Update(g_motors, &g_fault_state);

#ifdef DEBUG
    /* Optional: send debug info over UART (non-blocking) */
    /* Disabled in release to avoid timing interference.   */
#endif

    /* Idle until next interrupt — saves power */
    __WFI();
  }
}

/* ═══════════════════════════════════════════════════════════════════
 *  HAL CALLBACKS
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief  TIM period-elapsed callback — routes TIM6 to control loop.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  ControlLoop_TimerCallback(htim);
}

/**
 * @brief  FDCAN RX FIFO0 callback — routes to CAN comm module.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  CAN_Comm_RxCallback(hfdcan, RxFifo0ITs);
}

/**
 * @brief  EXTI callback — routes E-STOP pin to safety module.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ESTOP_GPIO_PIN) {
    Safety_ESTOP_IRQ(g_motors);
  }
}

/* ═══════════════════════════════════════════════════════════════════
 *  PERIPHERAL CONFIGURATION
 *  (In a CubeMX project, these are auto-generated.
 *   Provided here as reference implementations.)
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief  System Clock Configuration — 480 MHz via PLL1.
 *
 * HSE → PLL1 → SYSCLK = 480 MHz
 * AHB  = 240 MHz
 * APB1 = 120 MHz (timer clock = 240 MHz)
 * APB2 = 120 MHz (timer clock = 240 MHz)
 */
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Supply configuration — LDO */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* Voltage scaling for 480 MHz operation */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /* HSE Oscillator — 25 MHz crystal on Nucleo-144 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; /* ST-Link MCO */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;   /* 25 MHz / 5   = 5 MHz  */
  RCC_OscInitStruct.PLL.PLLN = 192; /* 5 MHz × 192  = 960 MHz */
  RCC_OscInitStruct.PLL.PLLP = 2;   /* 960 / 2      = 480 MHz */
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Clock tree */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2; /* 240 MHz */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider =
      RCC_APB1_DIV2; /* 120 MHz, TIM clk 240 MHz */
  RCC_ClkInitStruct.APB2CLKDivider =
      RCC_APB2_DIV2; /* 120 MHz, TIM clk 240 MHz */
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* ─────────────────── GPIO Init ────────────────────────────────── */

/**
 * @brief  Configure GPIOs: motor direction, enable, overcurrent,
 *         E-STOP (EXTI), contactor, debug output.
 *
 * Pin assignments (example — adjust to your PCB schematic):
 *
 *   Motor Direction:  PA0 (FL), PA1 (FR), PA2 (RL), PA3 (RR)
 *   Motor Enable:     PD0 (FL), PD1 (FR), PD2 (RL), PD3 (RR)
 *   Overcurrent In:   PE0 (FL), PE1 (FR), PE2 (RL), PE3 (RR) — active-low
 *   E-STOP:           PC13 — EXTI falling edge
 *   Contactor:        PD4 — push-pull output
 *   Debug:            PB0 — push-pull output
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Motor direction: PA0–PA3, push-pull output */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Motor enable: PD0–PD3, push-pull output */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Contactor relay: PD4, push-pull output */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); /* OFF by default */

  /* Debug GPIO: PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Overcurrent inputs: PE0–PE3, pull-up (active-low fault) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* E-STOP: PC13, EXTI falling-edge interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Enable EXTI interrupt for E-STOP (highest priority) */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ─────────────────── PWM Timer Init ───────────────────────────── */

/**
 * @brief  TIM1 — PWM channels 1 & 2 for front-left / front-right motors.
 *
 * Timer clock = 240 MHz (APB2 timer).
 * Prescaler = 0 → counter clock = 240 MHz.
 * ARR = 11999 → PWM freq = 240 MHz / (11999+1) = 20 kHz.
 */
static void MX_TIM1_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  /* Enable TIM1 main output (required for advanced timers) */
  __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
 * @brief  TIM8 — PWM channels 1 & 2 for rear-left / rear-right motors.
 *         Same config as TIM1.
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

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

  __HAL_TIM_MOE_ENABLE(&htim8);
}

/* ─────────────────── Encoder Timer Init ───────────────────────── */

/**
 * @brief Initialise a timer in encoder mode (quadrature, both edges).
 */
static void Encoder_Timer_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance) {
  htim->Instance = instance;
  htim->Init.Prescaler = 0U;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 0xFFFFFFFF; /* 32-bit for TIM2/5, 0xFFFF for TIM3/4 */
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef sConfig = {0};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0F; /* Digital filter to reject noise */
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0F;

  HAL_TIM_Encoder_Init(htim, &sConfig);
}

static void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();
  Encoder_Timer_Init(&htim2, TIM2);
}

static void MX_TIM3_Init(void) {
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  /* TIM3 is 16-bit — set period accordingly */
  htim3.Init.Period = 0xFFFF;
  Encoder_Timer_Init(&htim3, TIM3);
}

static void MX_TIM4_Init(void) {
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Period = 0xFFFF;
  Encoder_Timer_Init(&htim4, TIM4);
}

static void MX_TIM5_Init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();
  Encoder_Timer_Init(&htim5, TIM5);
}

/* ─────────────────── TIM6 (Control Loop Timer) ────────────────── */

/**
 * @brief  TIM6 — 1 kHz update interrupt for control loop.
 *
 * Timer clock = 240 MHz (APB1 timer).
 * Prescaler = 239 → counter clock = 1 MHz.
 * ARR = 999 → update rate = 1 MHz / 1000 = 1 kHz.
 */
static void MX_TIM6_Init(void) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 239U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim6);

  /* TIM6 interrupt — priority just below E-STOP */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ─────────────────── FDCAN1 Init ──────────────────────────────── */

/**
 * @brief  FDCAN1 — 1 Mbps nominal / 5 Mbps data phase.
 *
 * FDCAN kernel clock sourced from PLL1Q (= 240 MHz).
 *
 * Nominal: 240 MHz / ((NomPresc) × (1 + NomSeg1 + NomSeg2))
 *   = 240 / (3 × (1 + 63 + 16)) = 240 / (3 × 80) = 1 Mbps
 *
 * Data: 240 MHz / ((DataPresc) × (1 + DataSeg1 + DataSeg2))
 *   = 240 / (1 × (1 + 38 + 9)) = 240 / 48 = 5 Mbps
 */
static void MX_FDCAN1_Init(void) {
  __HAL_RCC_FDCAN_CLK_ENABLE();

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;

  /* Nominal bit timing: 1 Mbps */
  hfdcan1.Init.NominalPrescaler = 3U;
  hfdcan1.Init.NominalSyncJumpWidth = 16U;
  hfdcan1.Init.NominalTimeSeg1 = 63U;
  hfdcan1.Init.NominalTimeSeg2 = 16U;

  /* Data bit timing: 5 Mbps */
  hfdcan1.Init.DataPrescaler = 1U;
  hfdcan1.Init.DataSyncJumpWidth = 4U;
  hfdcan1.Init.DataTimeSeg1 = 38U;
  hfdcan1.Init.DataTimeSeg2 = 9U;

  hfdcan1.Init.MessageRAMOffset = 0U;
  hfdcan1.Init.StdFiltersNbr = 2U;
  hfdcan1.Init.ExtFiltersNbr = 0U;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8U;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 4U;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  /* FDCAN interrupt */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/* ─────────────────── Motor Hardware Config ────────────────────── */

/**
 * @brief  Assign HAL handles & GPIO pins to the motor array.
 *         Adjust pin assignments to match your PCB schematic.
 */
static void Motor_HW_Config(void) {
  /* Front-Left (index 0) */
  g_motors[0].hw.pwm_timer = &htim1;
  g_motors[0].hw.pwm_channel = TIM_CHANNEL_1;
  g_motors[0].hw.enc_timer = &htim2;
  g_motors[0].hw.dir_port = GPIOA;
  g_motors[0].hw.dir_pin = GPIO_PIN_0;
  g_motors[0].hw.en_port = GPIOD;
  g_motors[0].hw.en_pin = GPIO_PIN_0;
  g_motors[0].hw.oc_port = GPIOE;
  g_motors[0].hw.oc_pin = GPIO_PIN_0;

  /* Front-Right (index 1) */
  g_motors[1].hw.pwm_timer = &htim1;
  g_motors[1].hw.pwm_channel = TIM_CHANNEL_2;
  g_motors[1].hw.enc_timer = &htim3;
  g_motors[1].hw.dir_port = GPIOA;
  g_motors[1].hw.dir_pin = GPIO_PIN_1;
  g_motors[1].hw.en_port = GPIOD;
  g_motors[1].hw.en_pin = GPIO_PIN_1;
  g_motors[1].hw.oc_port = GPIOE;
  g_motors[1].hw.oc_pin = GPIO_PIN_1;

  /* Rear-Left (index 2) */
  g_motors[2].hw.pwm_timer = &htim8;
  g_motors[2].hw.pwm_channel = TIM_CHANNEL_1;
  g_motors[2].hw.enc_timer = &htim4;
  g_motors[2].hw.dir_port = GPIOA;
  g_motors[2].hw.dir_pin = GPIO_PIN_2;
  g_motors[2].hw.en_port = GPIOD;
  g_motors[2].hw.en_pin = GPIO_PIN_2;
  g_motors[2].hw.oc_port = GPIOE;
  g_motors[2].hw.oc_pin = GPIO_PIN_2;

  /* Rear-Right (index 3) */
  g_motors[3].hw.pwm_timer = &htim8;
  g_motors[3].hw.pwm_channel = TIM_CHANNEL_2;
  g_motors[3].hw.enc_timer = &htim5;
  g_motors[3].hw.dir_port = GPIOA;
  g_motors[3].hw.dir_pin = GPIO_PIN_3;
  g_motors[3].hw.en_port = GPIOD;
  g_motors[3].hw.en_pin = GPIO_PIN_3;
  g_motors[3].hw.oc_port = GPIOE;
  g_motors[3].hw.oc_pin = GPIO_PIN_3;
}

/* ─────────────────── UART Debug (optional) ────────────────────── */

#ifdef DEBUG
static void MX_USART3_UART_Init(void) {
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}
#endif

/* ─────────────────── Error Handler ────────────────────────────── */

/**
 * @brief  HAL error handler — enter infinite loop with motors disabled.
 *         In production, trigger a watchdog reset instead.
 */
void Error_Handler(void) {
  __disable_irq();
  /* Ensure motors are off */
  Motor_EmergencyStop(g_motors);
  Safety_DisableContactor();
  while (1) {
    /* Trap — watchdog will reset in production */
  }
}

/* ─────────────────── ISR Handlers (routing) ───────────────────── */

/**
 * @brief  TIM6 & DAC combined IRQ handler.
 */
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }

/**
 * @brief  FDCAN1 interrupt 0 handler.
 */
void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }

/**
 * @brief  EXTI lines 10–15 handler (includes E-STOP on PC13).
 */
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(ESTOP_GPIO_PIN); }
