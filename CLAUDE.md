# PMDC_Motor_Control — STM32G474 closed-loop PMDC speed control

Closed-loop speed control firmware for an STM32G474RE (NUCLEO-G474RE) driving a
Callan M4-4205D PMDC motor with tachogenerator feedback, through a discrete
IXFX300N20X3 MOSFET H-bridge with SI8273BBD-IS1 gate drivers on the KLN-1001-A1
board (rev 1.2).

## Read these first, in this order

1. **`HANDOFF.md`** — the firmware architecture as built, per-commit history, the
   CubeMX/`.ioc` gotchas, open items.
2. **`PROTECTION.md`** — power-stage failure analysis, the current-limited braking
   law, board wiring plan, measurements still needed.

Then run `git log --oneline` and re-read the sources from disk. Do not rely on memory
of earlier sessions — the code has changed substantially between sessions before.

## Status

Builds clean. **Never run on hardware.** PROTECTION.md §5 lists the firmware gaps that
must close before anything is powered: the loop currently commands full reverse
(plugging) on any setpoint error above 500 rpm, and no hardware fault line reaches the
MCU yet.

## How this project is worked

- **Do not compile unless explicitly asked.** The user builds from STM32CubeIDE.
- **The CubeMX GUI is authoritative for peripheral configuration.** Regeneration
  overwrites hand edits outside `USER CODE` sections. Work in `USER CODE` sections and
  the standalone modules CubeMX never touches: `motor_driver.c`, `speed_control.c`,
  `tach_sensor.c`, `motor_cli.c`. For peripheral changes, give the user the GUI clicks.
  Hand-editing the `.ioc` is fragile — see the gotchas in HANDOFF.md.
- **Git:** Claude commits locally; the user pushes. Never push, never handle GitHub
  credentials.
- Repo root is `M4-4205D_Control/` (the CubeIDE project), branch `main`.

## Signal chain (summary — HANDOFF.md has the detail)

TIM6 at 1 kHz → TRGO → ADC1 (PA1, tach) → end-of-conversion interrupt runs the PI
loop → `MotorDriver_SetDuty()` → TIM1 20 kHz complementary PWM, 1 µs dead time,
sign-magnitude drive. Serial CLI on LPUART1 (ST-LINK VCP) at **209700 8N1**:
`set <rpm>`, `clear`, `status`, `stream <ms>`, `help`.

## Analog frontend

Bipolar tach signal conditioned to a VDDA/2-centred 0.21–3.09 V range, so the 12-bit
ADC code is signed speed with direction included. The source uses 1.8 rpm/code;
PROTECTION.md derives 1.696 from the 9.5 V/1000 rpm tacho and lists the change as
pending.
