# STM32CubeIDE / CubeMX Configuration Notes

Reference configuration for the UGV firmware on **STM32H743ZI** (Nucleo-144).
Updated for multi-competition compliance (IGVC, IRC, UGVC, URC, ERC).

---

## Clock Tree

| Parameter     | Value                         |
| ------------- | ----------------------------- |
| HSE           | 8 MHz (Bypass — ST-Link MCO)  |
| PLL1 Source   | HSE                           |
| PLLM          | 1 (÷1 → 8 MHz)                |
| PLLN          | 120 (×120 → 960 MHz)          |
| PLLP          | 2 (÷2 → **480 MHz SYSCLK**)   |
| PLLQ          | 20 (÷20 → **48 MHz FDCAN**)   |
| PLLR          | 2 (÷2 → 480 MHz)              |
| HCLK (AHB)    | 240 MHz (÷2)                  |
| APB1          | 120 MHz (timer clock 240 MHz) |
| APB2          | 120 MHz (timer clock 240 MHz) |
| Voltage Scale | VOS0                          |
| Flash Latency | 4 WS                          |

---

## Timer Assignments

| Timer    | Function           | Prescaler | ARR             | Frequency |
| -------- | ------------------ | --------- | --------------- | --------- |
| TIM1 CH1 | PWM — Motor FL     | 0         | 11999           | 20 kHz    |
| TIM1 CH2 | PWM — Motor FR     | 0         | 11999           | 20 kHz    |
| TIM8 CH1 | PWM — Motor RL     | 0         | 11999           | 20 kHz    |
| TIM8 CH2 | PWM — Motor RR     | 0         | 11999           | 20 kHz    |
| TIM2     | Encoder — Motor FL | 0         | 0xFFFFFFFF      | —         |
| TIM3     | Encoder — Motor FR | 0         | 0xFFFF (16-bit) | —         |
| TIM4     | Encoder — Motor RL | 0         | 0xFFFF (16-bit) | —         |
| TIM5     | Encoder — Motor RR | 0         | 0xFFFFFFFF      | —         |
| TIM6     | Control loop IRQ   | 23        | 9999            | **1 kHz** |

### PWM Pins (CubeMX Alternate Function)
- **TIM1_CH1** → PE9 (AF1)
- **TIM1_CH2** → PE11 (AF1)
- **TIM8_CH1** → PC6 (AF3)
- **TIM8_CH2** → PC7 (AF3)

### Encoder Pins
- **TIM2_CH1/CH2** → PA15 / PB3 (AF1)
- **TIM3_CH1/CH2** → PB4 / PB5 (AF2) — *Remap needed: PB4/PB5 also used for OC input. Use alternate pins or share.*
- **TIM4_CH1/CH2** → PD12 / PD13 (AF2)
- **TIM5_CH1/CH2** → PA0 / PA1 (AF2) — *Conflicts with motor direction. Remap to alternate pins on PCB.*

> **Note:** Encoder pins use digital noise filter = 0x0F. Enable input capture filter in CubeMX.

---

## FDCAN1

| Parameter              | Value                         |
| ---------------------- | ----------------------------- |
| Mode                   | FD with Bit Rate Switch       |
| Nominal Prescaler      | 3                             |
| Nominal Seg1           | 13                            |
| Nominal Seg2           | 2                             |
| Nominal SJW            | 2                             |
| **Nominal Bitrate**    | **1 Mbps**                    |
| Data Prescaler         | 1                             |
| Data Seg1              | 7                             |
| Data Seg2              | 2                             |
| Data SJW               | 2                             |
| **Data Bitrate**       | **~4.8 Mbps** (48 MHz kernel) |
| Kernel Clock           | PLL1Q (48 MHz)                |
| TX Pin                 | PD1 (AF9)                     |
| RX Pin                 | PD0 (AF9)                     |
| RX FIFO 0 Elements     | 8                             |
| TX FIFO Queue Elements | 4                             |
| Element Size           | 64 bytes                      |
| **Std Filters**        | **4** (was 2)                 |

### CAN Filter Configuration

| Filter | Type | ID1 (hex) | ID2 (hex) | FIFO  | Purpose                    |
| ------ | ---- | --------- | --------- | ----- | -------------------------- |
| 0      | Dual | 0x100     | 0x110     | FIFO0 | Velocity cmd + Mode select |
| 1      | Dual | 0x120     | 0x130     | FIFO0 | Config profile + Reset cmd |
| 2      | Dual | 0x200     | 0x200     | FIFO0 | Heartbeat                  |

### CAN Message ID Summary

| ID    | Dir | DLC | Purpose                   |
| ----- | --- | --- | ------------------------- |
| 0x100 | RX  | 8B  | Velocity command          |
| 0x110 | RX  | 1B  | Mode select (TELEOP/AUTO) |
| 0x120 | RX  | 2B  | Competition profile       |
| 0x130 | RX  | 1B  | Reset (key = 0xA5)        |
| 0x200 | RX  | 1B  | Heartbeat                 |
| 0x300 | TX  | 12B | Motor telemetry           |
| 0x310 | TX  | 8B  | Odometry                  |
| 0x320 | TX  | 8B  | Battery telemetry         |
| 0x400 | TX  | 2B  | Fault report              |

---

## ADC1 — Battery Monitoring [NEW]

| Parameter       | Value              |
| --------------- | ------------------ |
| Clock Prescaler | Async ÷4 (~15 MHz) |
| Resolution      | 12-bit             |
| Scan Mode       | Disabled (single)  |
| Continuous Mode | Disabled           |
| Trigger         | Software Start     |

### ADC Channels

| Channel | Pin | Function              | Sensor / Notes                      |
| ------- | --- | --------------------- | ----------------------------------- |
| CH4     | PA4 | Battery voltage       | Via 1:11 voltage divider (36V→3.3V) |
| CH5     | PA5 | Battery current       | ACS712 (185 mV/A, 1650 mV zero)     |
| CH6     | PA6 | Motor/MCU temperature | LM35-style (10 mV/°C, 500 mV @ 0°C) |

> **Calibration:** ADC auto-calibration runs at startup (`HAL_ADCEx_Calibration_Start`).

---

## GPIO Pin Map

| Pin      | Function                    | Mode                            | Notes                  |
| -------- | --------------------------- | ------------------------------- | ---------------------- |
| PA0–PA3  | Motor direction             | Output PP                       | Active-high = forward  |
| PA4      | ADC1_CH4 (batt voltage)     | Analog                          | Voltage divider input  |
| PA5      | ADC1_CH5 (batt current)     | Analog                          | Current sensor input   |
| PA6      | ADC1_CH6 (temperature)      | Analog                          | Temp sensor input      |
| PA8–PA11 | Motor enable/relay          | Output PP                       | Active-high = enabled  |
| PB0      | Debug timing                | Output PP                       | Toggle in control loop |
| PB4–PB7  | Overcurrent fault           | Input, Pull-up                  | Active-low = fault     |
| PC13     | **Mechanical E-STOP**       | EXTI falling edge, Pull-up      | Priority (0,0)         |
| PD0      | FDCAN1 RX                   | AF9                             | CAN bus                |
| PD1      | FDCAN1 TX                   | AF9                             | CAN bus                |
| PD4      | Battery contactor relay     | Output PP                       | Active-high = closed   |
| **PD5**  | **Battery isolation relay** | **Output PP** [NEW]             | Active-high = closed   |
| PF0      | **LED Red**                 | **Output PP** [NEW]             | Fault/ESTOP indicator  |
| PF1      | **LED Green**               | **Output PP** [NEW]             | Waypoint indicator     |
| PF2      | **LED Blue**                | **Output PP** [NEW]             | Power/Teleop indicator |
| PF3      | **Laser pointer**           | **Output PP** [NEW]             | UGVC competition       |
| **PG0**  | **Wireless E-STOP**         | **EXTI falling, Pull-up** [NEW] | Priority (0,1)         |

---

## NVIC Priority Table

| IRQ            | Priority (Pre, Sub) | Function                        |
| -------------- | ------------------- | ------------------------------- |
| EXTI15_10      | **(0, 0)**          | **Mechanical E-STOP** — highest |
| **EXTI0**      | **(0, 1)**          | **Wireless E-STOP** [NEW]       |
| TIM6_DAC       | (1, 0)              | 1 kHz control loop              |
| FDCAN1_IT0     | (2, 0)              | CAN-FD RX                       |
| *ADC (polled)* | *N/A*               | Battery — polled in main loop   |

---

## CubeMX Checklist

1. **Project → Settings** → Toolchain = STM32CubeIDE
2. **RCC** → HSE = BYPASS Clock Source (8 MHz)
3. **Clock Configuration** → Match table above (PLL1Q = 48 MHz for FDCAN)
4. **TIM1** → PWM Generation CH1 + CH2, ARR = 11999
5. **TIM8** → PWM Generation CH1 + CH2, ARR = 11999
6. **TIM2** → Encoder Mode, Period = 0xFFFFFFFF
7. **TIM3** → Encoder Mode, Period = 0xFFFF
8. **TIM4** → Encoder Mode, Period = 0xFFFF
9. **TIM5** → Encoder Mode, Period = 0xFFFFFFFF
10. **TIM6** → Internal Clock, PSC = 23, ARR = 9999, enable Update IRQ
11. **FDCAN1** → FD + BRS, configure bit timing per table, **4 std filters**
12. **ADC1** → CH4 + CH5 + CH6, 12-bit, single conversion [NEW]
13. **GPIO** → Configure all pins per map above (including PD5, PF0–PF3, PG0)
14. **NVIC** → Set priorities per table above (add EXTI0 at 0,1)
15. **SYS** → Debug = Serial Wire, Timebase Source = SysTick
16. **USART3** (optional) → 115200 Baud, 8N1 — Nucleo virtual COM

---

## Build Flags

```
# Add to project C/C++ build settings → Preprocessor
DEBUG=1           # Enable UART debug prints (remove for release)
```
