# STM32CubeIDE / CubeMX Configuration Notes

Reference configuration for the UGV firmware on **STM32H743ZI** (Nucleo-144).

---

## Clock Tree

| Parameter     | Value                         |
| ------------- | ----------------------------- |
| HSE           | 25 MHz (Bypass — ST-Link MCO) |
| PLL1 Source   | HSE                           |
| PLLM          | 5 (÷5 → 5 MHz)                |
| PLLN          | 192 (×192 → 960 MHz)          |
| PLLP          | 2 (÷2 → **480 MHz SYSCLK**)   |
| PLLQ          | 4 (÷4 → 240 MHz FDCAN)        |
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
| TIM6     | Control loop IRQ   | 239       | 999             | **1 kHz** |

### PWM Pins (CubeMX Alternate Function)
- **TIM1_CH1** → PE9 (AF1) — *or your PCB pin*
- **TIM1_CH2** → PE11 (AF1)
- **TIM8_CH1** → PC6 (AF3)
- **TIM8_CH2** → PC7 (AF3)

### Encoder Pins
- **TIM2_CH1/CH2** → PA15 / PB3 (AF1)
- **TIM3_CH1/CH2** → PB4 / PB5 (AF2)
- **TIM4_CH1/CH2** → PD12 / PD13 (AF2)
- **TIM5_CH1/CH2** → PA0 / PA1 (AF2) — *may conflict; remap as needed*

> **Note:** Encoder pins use digital noise filter = 0x0F. Enable input capture filter in CubeMX.

---

## FDCAN1

| Parameter              | Value                   |
| ---------------------- | ----------------------- |
| Mode                   | FD with Bit Rate Switch |
| Nominal Prescaler      | 3                       |
| Nominal Seg1           | 63                      |
| Nominal Seg2           | 16                      |
| Nominal SJW            | 16                      |
| **Nominal Bitrate**    | **1 Mbps**              |
| Data Prescaler         | 1                       |
| Data Seg1              | 38                      |
| Data Seg2              | 9                       |
| Data SJW               | 4                       |
| **Data Bitrate**       | **5 Mbps**              |
| Kernel Clock           | PLL1Q (240 MHz)         |
| TX Pin                 | PD1 (AF9)               |
| RX Pin                 | PD0 (AF9)               |
| RX FIFO 0 Elements     | 8                       |
| TX FIFO Queue Elements | 4                       |
| Element Size           | 64 bytes                |
| Std Filters            | 2                       |

---

## GPIO Pin Map

| Pin     | Function           | Mode                       | Notes                  |
| ------- | ------------------ | -------------------------- | ---------------------- |
| PA0–PA3 | Motor direction    | Output PP                  | Active-high = forward  |
| PD0–PD3 | Motor enable/relay | Output PP                  | Active-high = enabled  |
| PD4     | Battery contactor  | Output PP                  | Active-high = closed   |
| PE0–PE3 | Overcurrent fault  | Input, Pull-up             | Active-low = fault     |
| PC13    | E-STOP             | EXTI falling edge, Pull-up | Highest priority (0,0) |
| PB0     | Debug timing       | Output PP                  | Toggle in control loop |

---

## NVIC Priority Table

| IRQ        | Priority (Pre, Sub) | Function             |
| ---------- | ------------------- | -------------------- |
| EXTI15_10  | (0, 0)              | **E-STOP** — highest |
| TIM6_DAC   | (1, 0)              | 1 kHz control loop   |
| FDCAN1_IT0 | (2, 0)              | CAN-FD RX            |

---

## CubeMX Checklist

1. **Project → Settings** → Toolchain = STM32CubeIDE
2. **RCC** → HSE = BYPASS Clock Source
3. **Clock Configuration** → Match table above
4. **TIM1** → PWM Generation CH1 + CH2, ARR = 11999
5. **TIM8** → PWM Generation CH1 + CH2, ARR = 11999
6. **TIM2** → Encoder Mode, Period = 0xFFFFFFFF
7. **TIM3** → Encoder Mode, Period = 0xFFFF
8. **TIM4** → Encoder Mode, Period = 0xFFFF
9. **TIM5** → Encoder Mode, Period = 0xFFFFFFFF
10. **TIM6** → Internal Clock, PSC = 239, ARR = 999, enable Update IRQ
11. **FDCAN1** → FD + BRS, configure bit timing per table
12. **GPIO** → Configure all pins per map above
13. **NVIC** → Set priorities per table above
14. **SYS** → Debug = Serial Wire, Timebase Source = SysTick
15. **USART3** (optional) → 115200 Baud, 8N1 — Nucleo virtual COM

---

## Build Flags

```
# Add to project C/C++ build settings → Preprocessor
DEBUG=1           # Enable UART debug prints (remove for release)
```
