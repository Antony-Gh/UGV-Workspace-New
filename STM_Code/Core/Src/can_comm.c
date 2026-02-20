/**
 * @file    can_comm.c
 * @brief   CAN-FD communication implementation — expanded for
 * multi-competition.
 *
 * FDCAN1 operates at 1 Mbps nominal / 5 Mbps data.
 * RX is handled via interrupt callback → parses velocity, mode, config,
 * reset, and heartbeat messages.
 * TX functions are non-blocking (post to TX FIFO).
 */

#include "can_comm.h"
#include "config.h"
#include "fault.h"
#include "lights.h"
#include "safety.h"
#include <string.h>

/* ──────────────── Module-Level State ──────────────────────────── */

/** FDCAN handle stored at init time. */
static FDCAN_HandleTypeDef *s_hfdcan = NULL;

/** Latest velocity command — written by ISR, read by control loop. */
volatile CAN_VelocityCmd_t g_can_velocity_cmd = {0};
volatile bool g_can_velocity_cmd_new = false;

/* External reference to fault state for heartbeat update.
 * Defined in main.c.                                             */
extern Fault_State_t g_fault_state;

/* ──────────────── Private Helpers ─────────────────────────────── */

/**
 * @brief  Transmit a CAN-FD frame.
 * @param  id         Standard 11-bit identifier.
 * @param  data       Pointer to payload bytes.
 * @param  length     HAL data-length code (e.g. FDCAN_DLC_BYTES_8).
 */
static void can_transmit(uint32_t id, const uint8_t *data, uint32_t length) {
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = length;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON; /* Use 5 Mbps data */
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  /* Non-blocking — if TX FIFO is full the frame is silently dropped.
   * In production, you could check the return value and set a fault flag. */
  HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &tx_header, (uint8_t *)data);
}

/* ──────────────── Public Functions ─────────────────────────────── */

void CAN_Comm_Init(FDCAN_HandleTypeDef *hfdcan) {
  s_hfdcan = hfdcan;

  /* ── Configure RX filters ──────────────────────────────────── */
  /*
   * Filter 0: 0x100 (Velocity) + 0x110 (Mode Select)
   * Filter 1: 0x120 (Config)   + 0x130 (Reset)
   * Filter 2: 0x200 (Heartbeat) — single ID in dual filter slot
   */

  FDCAN_FilterTypeDef filter;
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  /* Filter 0: velocity command + mode select */
  filter.FilterIndex = 0U;
  filter.FilterID1 = CAN_ID_VELOCITY_CMD; /* 0x100 */
  filter.FilterID2 = CAN_ID_MODE_SELECT;  /* 0x110 */
  HAL_FDCAN_ConfigFilter(hfdcan, &filter);

  /* Filter 1: config profile + reset command */
  filter.FilterIndex = 1U;
  filter.FilterID1 = CAN_ID_CONFIG_PROFILE; /* 0x120 */
  filter.FilterID2 = CAN_ID_RESET_CMD;      /* 0x130 */
  HAL_FDCAN_ConfigFilter(hfdcan, &filter);

  /* Filter 2: heartbeat (store same ID in both slots) */
  filter.FilterIndex = 2U;
  filter.FilterID1 = CAN_ID_HEARTBEAT; /* 0x200 */
  filter.FilterID2 = CAN_ID_HEARTBEAT; /* 0x200 */
  HAL_FDCAN_ConfigFilter(hfdcan, &filter);

  /* Reject all non-matching standard IDs */
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, /* Non-matching std */
                               FDCAN_REJECT,         /* Non-matching ext */
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  /* Start FDCAN */
  HAL_FDCAN_Start(hfdcan);

  /* Activate RX FIFO 0 new-message interrupt */
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

void CAN_Comm_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64]; /* CAN-FD max payload */

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    return;

  /* Read all available messages from FIFO 0 */
  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) ==
         HAL_OK) {
    switch (rx_header.Identifier) {

    /* ── 0x100: Velocity Command ────────────────────── */
    case CAN_ID_VELOCITY_CMD:
      if (rx_header.DataLength >= FDCAN_DLC_BYTES_8) {
        /* Copy 8 bytes into the shared struct (atomic on CM7) */
        memcpy((void *)&g_can_velocity_cmd, rx_data, sizeof(CAN_VelocityCmd_t));
        g_can_velocity_cmd_new = true;
      }
      break;

    /* ── 0x110: Mode Select ─────────────────────────── */
    case CAN_ID_MODE_SELECT: {
      CAN_ModeCmd_t mode_cmd;
      memcpy(&mode_cmd, rx_data, sizeof(CAN_ModeCmd_t));
      Safety_SetMode(mode_cmd.mode != 0U);
    } break;

    /* ── 0x120: Config Profile ──────────────────────── */
    case CAN_ID_CONFIG_PROFILE: {
      CAN_ConfigCmd_t config_cmd;
      memcpy(&config_cmd, rx_data, sizeof(CAN_ConfigCmd_t));
      Config_Apply((Competition_t)config_cmd.competition);
    } break;

    /* ── 0x130: Reset Command ───────────────────────── */
    case CAN_ID_RESET_CMD: {
      CAN_ResetCmd_t reset_cmd;
      memcpy(&reset_cmd, rx_data, sizeof(CAN_ResetCmd_t));
      if (reset_cmd.reset_key == CAN_RESET_KEY) {
        Safety_RequestReset();
      }
    } break;

    /* ── 0x200: Heartbeat ───────────────────────────── */
    case CAN_ID_HEARTBEAT:
      Fault_HeartbeatReceived(&g_fault_state, HAL_GetTick());
      break;

    default:
      /* Unknown ID — ignored (filtered out anyway) */
      break;
    }
  }

  /* Re-activate notification for next message */
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
}

void CAN_Comm_SendTelemetry(float vel_fl, float vel_fr, float vel_rl,
                            float vel_rr, uint8_t system_state,
                            uint8_t fault_flags, uint8_t comp_id) {
  CAN_Telemetry_t payload;

  payload.velocity_fl = (int16_t)(vel_fl * CAN_VELOCITY_SCALE);
  payload.velocity_fr = (int16_t)(vel_fr * CAN_VELOCITY_SCALE);
  payload.velocity_rl = (int16_t)(vel_rl * CAN_VELOCITY_SCALE);
  payload.velocity_rr = (int16_t)(vel_rr * CAN_VELOCITY_SCALE);
  payload.system_state = system_state;
  payload.fault_flags = fault_flags;
  payload.comp_id = comp_id;
  payload.reserved = 0U;

  can_transmit(CAN_ID_TELEMETRY, (const uint8_t *)&payload, FDCAN_DLC_BYTES_12);
}

void CAN_Comm_SendOdometry(int32_t distance_mm, int16_t heading_cdeg) {
  CAN_Odometry_t payload;

  payload.distance_mm = distance_mm;
  payload.heading_cdeg = heading_cdeg;
  payload.reserved = 0U;

  can_transmit(CAN_ID_ODOMETRY, (const uint8_t *)&payload, FDCAN_DLC_BYTES_8);
}

void CAN_Comm_SendBattery(uint16_t voltage_mv, int16_t current_ma,
                          int16_t temp_cdeg) {
  CAN_BatteryTelemetry_t payload;

  payload.voltage_mv = voltage_mv;
  payload.current_ma = current_ma;
  payload.temp_cdeg = temp_cdeg;
  payload.reserved = 0U;

  can_transmit(CAN_ID_BATTERY, (const uint8_t *)&payload, FDCAN_DLC_BYTES_8);
}

void CAN_Comm_SendFault(uint8_t fault_flags, uint8_t system_state) {
  CAN_FaultReport_t payload;

  payload.fault_flags = fault_flags;
  payload.system_state = system_state;

  can_transmit(CAN_ID_FAULT_REPORT, (const uint8_t *)&payload,
               FDCAN_DLC_BYTES_2);
}
