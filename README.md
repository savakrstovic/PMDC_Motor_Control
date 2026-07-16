# PMDC Motor Speed Control (STM32G474)

Closed-loop speed control firmware for a **M4-4205D permanent-magnet DC motor**,
running on an **STM32G474RE** (NUCLEO-G474RE, Cortex-M4 @ 170 MHz). Speed is
sensed by a tachogenerator and regulated by a discrete MOSFET H-bridge under a
1 kHz PI control loop.

## Overview

- **Feedback:** bipolar tachogenerator signal, conditioned to a VDDA/2-centered
  0.21–3.09 V range on the ADC. ~1.8 rpm per 12-bit code, sign included — so a
  single ADC reading encodes both speed magnitude and direction.
- **Actuation:** discrete H-bridge of 4× IXFX300N20X3 MOSFETs driven by
  SI8273BBD-IS1 isolated gate drivers. Sign-magnitude PWM at 20 kHz.
- **Control:** 1 kHz control tick sampling the tach, converting to signed rpm,
  computing error against a setpoint, and running a PI controller with
  anti-windup and output saturation. Target loop bandwidth 50–100 Hz.
- **Protection:** onboard current/voltage protection asserts a hardware fault
  line into TIM1's break input (BKIN), which disables all bridge outputs in
  silicon. A dedicated GPIO pulses the protection latch clear.

## Pin map (NUCLEO-G474RE)

| Function            | Pin  | Peripheral      |
|---------------------|------|-----------------|
| Bridge leg A PWM    | PA8  | TIM1_CH1        |
| Bridge leg A PWM /  | PA7  | TIM1_CH1N       |
| Bridge leg B PWM    | PA9  | TIM1_CH2        |
| Bridge leg B PWM /  | PB0  | TIM1_CH2N       |
| Fault input (break) | PA6  | TIM1_BKIN       |
| Fault latch clear   | PB1  | GPIO output     |
| Tach analog input   | PA1  | ADC1_IN2        |
| Control tick        | —    | TIM6 @ 1 kHz    |
| Debug UART (VCP)    | PA2/PA3 | LPUART1      |

## Firmware structure

| Module                          | Responsibility                                   |
|---------------------------------|--------------------------------------------------|
| `Core/Src/tim.c`                | TIM1 PWM bridge (dead-time + break), TIM6 tick   |
| `Core/Src/adc.c`                | ADC1 tach channel                                |
| `Core/Src/motor_driver.c`       | Sign-magnitude drive, fault query & clear        |
| `Core/Src/tach_sensor.c`        | ADC code → signed rpm                            |
| `Core/Src/speed_control.c`      | 1 kHz PI loop with anti-windup                   |

## Build

Open `M4-4205D_Control.ioc` / the project in **STM32CubeIDE** and build, or use
the STM32CubeCLT toolchain (`arm-none-eabi-gcc`) against the provided
`STM32G474RETX_FLASH.ld`.

## Status & notes

- ⚠️ **PI gains** (`SPEED_KP`, `SPEED_KI` in `speed_control.c`) are placeholder
  values in the right order of magnitude — retune against the real motor/load.
- ⚠️ Two hardware-convention macros need confirmation against the board and are
  each a single-line flip:
  - `BreakPolarity` in `tim.c` — assumes an active-low fault line.
  - `FAULT_CLEAR_PULSE_ACTIVE_HIGH` in `motor_driver.c` — assumes idle-low,
    pulse-high to clear the latch.
