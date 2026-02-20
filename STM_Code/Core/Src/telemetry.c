/**
 * @file    telemetry.c
 * @brief   Periodic telemetry transmission — expanded with odometry and
 * battery.
 *
 * Called from the main super-loop.
 * Uses tick-based timing (HAL_GetTick) — never blocks.
 *
 * Three staggered channels:
 *   0x300 Motor telemetry — every 50 ms
 *   0x310 Odometry        — every 100 ms
 *   0x320 Battery         — every 200 ms (ADC polling)
 */

#include "telemetry.h"
#include "can_comm.h"
#include "config.h"
#include "control_loop.h"
#include "stm32h7xx_hal.h"

/* ──────────────── Module-Level State ──────────────────────────── */

static uint32_t s_last_motor_tick = 0U;
static uint32_t s_last_odom_tick = 0U;
static uint32_t s_last_batt_tick = 0U;

static ADC_HandleTypeDef *s_hadc = NULL;

/* ──────────────── Private: ADC Helpers ────────────────────────── */

/**
 * @brief  Read one ADC channel (blocking, single conversion).
 * @param  channel  ADC channel number (ADC_CHANNEL_x).
 * @return Raw 12-bit ADC value, or 0 on timeout/error.
 *
 * Uses a short timeout (2 ms) to avoid blocking the main loop.
 * Called from main loop context only — never from ISR.
 */
static uint32_t adc_read_channel(uint32_t channel) {
  if (s_hadc == NULL)
    return 0U;

  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel = channel;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0U;

  if (HAL_ADC_ConfigChannel(s_hadc, &cfg) != HAL_OK)
    return 0U;

  HAL_ADC_Start(s_hadc);
  if (HAL_ADC_PollForConversion(s_hadc, 2U) != HAL_OK) {
    HAL_ADC_Stop(s_hadc);
    return 0U;
  }

  uint32_t raw = HAL_ADC_GetValue(s_hadc);
  HAL_ADC_Stop(s_hadc);

  return raw;
}

/**
 * @brief  Convert raw ADC value to battery voltage in millivolts.
 */
static uint16_t adc_to_voltage_mv(uint32_t raw) {
  /* V_batt = (raw / 4095) × 3300 × divider_ratio */
  return (uint16_t)((raw * ADC_VREF_MV * BATT_VDIV_RATIO) / ADC_RESOLUTION);
}

/**
 * @brief  Convert raw ADC value to battery current in milliamps.
 */
static int16_t adc_to_current_ma(uint32_t raw) {
  /* V_sense = (raw / 4095) × 3300 */
  int32_t v_sense_mv = (int32_t)((raw * ADC_VREF_MV) / ADC_RESOLUTION);
  /* I = (V_sense - V_zero) / sensitivity  →  mA */
  int32_t current_ma = ((v_sense_mv - (int32_t)BATT_ISENSE_ZERO_MV) * 1000) /
                       (int32_t)BATT_ISENSE_MV_PER_A;
  return (int16_t)current_ma;
}

/**
 * @brief  Convert raw ADC value to temperature in centidegrees Celsius.
 *         Assumes a linear sensor with 10 mV/°C, 500 mV at 0°C (LM35-style).
 */
static int16_t adc_to_temp_cdeg(uint32_t raw) {
  /* V_temp = (raw / 4095) × 3300  mV */
  int32_t v_temp_mv = (int32_t)((raw * ADC_VREF_MV) / ADC_RESOLUTION);
  /* T = (V - 500) / 10  → °C, then ×100 for centidegrees */
  int32_t temp_cdeg = ((v_temp_mv - 500) * 100) / 10;
  return (int16_t)temp_cdeg;
}

/* ──────────────── Public Functions ─────────────────────────────── */

void Telemetry_Init(ADC_HandleTypeDef *hadc) {
  uint32_t now = HAL_GetTick();
  s_last_motor_tick = now;
  s_last_odom_tick = now;
  s_last_batt_tick = now;
  s_hadc = hadc;
}

void Telemetry_Update(const Motor_t motors[MOTOR_COUNT],
                      const Fault_State_t *fault_state) {
  uint32_t now = HAL_GetTick();

  /* ── 0x300: Motor telemetry (50 ms) ──────────────────────── */
  if ((now - s_last_motor_tick) >= TELEMETRY_MOTOR_INTERVAL_MS) {
    s_last_motor_tick = now;

    CAN_Comm_SendTelemetry(
        motors[0].state.current_velocity, motors[1].state.current_velocity,
        motors[2].state.current_velocity, motors[3].state.current_velocity,
        (uint8_t)Safety_GetState(), Fault_GetActive(fault_state),
        (uint8_t)Config_GetActiveComp());
  }

  /* ── 0x310: Odometry (100 ms) ────────────────────────────── */
  if ((now - s_last_odom_tick) >= TELEMETRY_ODOM_INTERVAL_MS) {
    s_last_odom_tick = now;

    int32_t distance_mm = 0;
    int16_t heading_cdeg = 0;
    ControlLoop_GetOdometry(&distance_mm, &heading_cdeg);

    CAN_Comm_SendOdometry(distance_mm, heading_cdeg);
  }

  /* ── 0x320: Battery telemetry (200 ms) ───────────────────── */
  if ((now - s_last_batt_tick) >= TELEMETRY_BATT_INTERVAL_MS) {
    s_last_batt_tick = now;

    /* Read ADC channels: PA4 = voltage, PA5 = current, PA6 = temp */
    uint32_t raw_voltage = adc_read_channel(ADC_CHANNEL_4);
    uint32_t raw_current = adc_read_channel(ADC_CHANNEL_5);
    uint32_t raw_temp = adc_read_channel(ADC_CHANNEL_6);

    uint16_t voltage_mv = adc_to_voltage_mv(raw_voltage);
    int16_t current_ma = adc_to_current_ma(raw_current);
    int16_t temp_cdeg = adc_to_temp_cdeg(raw_temp);

    CAN_Comm_SendBattery(voltage_mv, current_ma, temp_cdeg);
  }
}
