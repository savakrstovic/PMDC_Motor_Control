# Handoff — Serial CLI + triggered-ADC control loop

Written 2026-07-26. Covers the four commits `c1dfd7e..ee21431`, all merged to `main`.

**Status: builds clean, never run on hardware.** Nothing in here has been tested on
the physical bridge. Read "Open items" before powering anything.

---

## Architecture as built

Signal chain, 1 kHz:

```
TIM6 (1kHz, no IRQ) ──TRGO──▶ ADC1 conversion (hardware, no CPU)
                                  └── EOC IRQ ──▶ TachSensor_OnConversionComplete()
                                                  SpeedControl_Task()  ← PI runs here
```

TIM6 is a pure hardware pacer now; its update interrupt is retired. The control loop
runs from ADC end-of-conversion with the sample already in hand (~2.5 µs old), so
there is no polling anywhere in the tick and it is hard-bounded.

Interrupt priorities (NVIC_PRIORITYGROUP_4 → 4 preempt bits, 0 = most urgent):

| IRQ | Preempt | Role |
|---|---|---|
| SysTick | 0 | HAL tick (`TICK_INT_PRIORITY`) |
| ADC1_2 | 1 | control loop |
| LPUART1 | 3 | serial CLI |

The CLI sits strictly below the control loop: a character arriving mid-tick waits;
the tick never waits.

Power stage: TIM1, 20 kHz (ARR 8499 @ 170 MHz), complementary PWM with 1.000 µs
dead time, sign-magnitude drive via `motor_driver.c`.

Console: LPUART1 on PA2/PA3 → ST-LINK VCP, **209700 baud 8N1** (non-standard — your
terminal needs a custom-baud field). Commands: `set <rpm>`, `clear`, `status`,
`stream <ms>` / `stream off`, `help`.

---

## What changed, by commit

**`c1dfd7e` — LPUART1 CLI + control loop moved to triggered ADC**

- New `motor_cli.c` / `motor_cli.h`. Interrupt-driven, single-producer/single-consumer
  ring buffers both directions (RX 128 B, TX 512 B). The ISR moves one byte and never
  blocks — a full ring drops rather than spins. All parsing/formatting is thread-level
  in `MotorCli_Task()`, called from the main loop.
- Drives the LPUART registers directly rather than `HAL_UART_Transmit/Receive_IT`,
  whose state machine can't express free-running RX without a byte-loss window between
  the completion callback and re-arm. `LPUART1_IRQHandler` lives in `stm32g4xx_it.c` in
  CubeMX-generated form and calls `MotorCli_IrqHandler()`, so regeneration can't create
  a duplicate symbol. The generated `HAL_UART_IRQHandler()` call below it is vestigial.
- Control loop moved from the TIM6 update ISR to the ADC EOC ISR.
- `TachSensor_ReadRpm()` → `TachSensor_LatestRpm()` (non-blocking); added
  `TachSensor_Init()` and `TachSensor_OnConversionComplete()`.
- `SpeedControl_GetTelemetry()` publishes a consistent setpoint/measured/duty snapshot
  under a short critical section.
- `SpeedControl_WatchdogTask()` added — see Open items #4.
- ADC `Overrun` → `DATA_OVERWRITTEN` (HAL treats it as non-fatal without DMA).

**`d584ed8` — TIM1 dead time 1 µs**

`DeadTime = 149` (`DTG = 0x95`). DTG is piecewise, not linear (RM0440 §28.4.18). With
CKD=DIV1 and TIM1 on 170 MHz, t_DTS = 5.882 ns, so range `0xx` tops out at 747 ns and
1 µs needs range `10x`: `(64 + 21) × 2 × t_DTS = 85/85 MHz = 1.000 µs` exactly.

**`2bcf2f4` — TIM1 Output Compare → PWM Generation, 20 kHz**

TIM1's channels were in `TIM_OCMODE_TIMING` (frozen) — the CCR/CNT comparison had no
effect on the outputs, so `MotorDriver_SetDuty()` was writing values that did nothing.
Now `TIM_OCMODE_PWM1` via `HAL_TIM_PWM_Init` / `HAL_TIM_PWM_ConfigChannel`. Period
65535 → 8499, which matches the `PWM_COUNT_FULL_SCALE = 8500` that `motor_driver.c`
already assumed (at the old ARR a full-scale command produced only 13% duty).

**`ee21431` — `.ioc` synced with the TIM1 PWM configuration** (written from the GUI).

---

## CubeMX / `.ioc` gotchas learned the hard way

- **TIM1's period key is `PeriodNoDither`, not `Period`.** The G4 advanced timers
  support dithering and CubeMX uses a different key for them. TIM6 still uses plain
  `Period`. Hand-editing `TIM1.Period=` silently does nothing.
- **A channel's mode string appears in SIX places.** Changing Output Compare → PWM
  Generation means editing all of: `PA7.Mode`, `PB0.Mode`, `SH.S_TIM1_CH1.0`,
  `SH.S_TIM1_CH2.0`, the two escaped `TIM1.Channel-...` keys, and `TIM1.IPParameters`.
  Miss any one and CubeMX can't resolve the peripheral and **drops TIM1 from the
  project entirely**. Prefer the GUI for this change.
- **`NVIC.*` values are nine colon-separated fields.** Fields 1–3 are
  enabled : preempt : sub. Fields 4–9 are undocumented code-generation flags — copy
  them verbatim from an existing peripheral-IRQ line rather than inventing values.
  The current entries have been round-tripped through CubeMX unchanged, so they're
  known good.
- Dead Time, channel mode, ARR, ADC trigger and NVIC priority are all first-class
  CubeMX parameters — set them in the GUI and regeneration preserves them. No
  `USER CODE` guard needed for any of them.
- **HAL dispatches the Msp callback by which Init function you call.** Switching
  `HAL_TIM_OC_Init` → `HAL_TIM_PWM_Init` required renaming `HAL_TIM_OC_MspInit` /
  `MspDeInit` → `HAL_TIM_PWM_*`. Miss that and TIM1 comes up with no clock and no
  BKIN pin, with no build error.
- HAL only tears down the ADC EOC interrupt when the trigger source is *software*
  start. With a timer trigger, `HAL_ADC_Start_IT` arms it once and it free-runs — no
  re-arm window where a trigger can be lost.

---

## Open items

**1. Dead time is unmeasured.** 1.000 µs is arithmetic, not a measurement. The correct
value is roughly `t_d(off)max − t_d(on)min + gate-driver propagation skew + margin`.
For IXFX300N20X3 devices (300 A, large Qg) 1 µs may be marginal rather than generous.
**Scope V_GS at the FET pins — not the driver inputs — on one leg, at low bus voltage,
and confirm real non-overlap before applying full bus.** Also unverified: whether the
SI8273BBD-IS1 has its own interlock / DT-pin dead time, in which case the larger of the
two governs.

**2. PI gains are untuned placeholders.** `SPEED_KP = 2.0f`, `SPEED_KI = 40.0f` in
`speed_control.c`, targeting 50–100 Hz bandwidth per `CLAUDE.md`. Retune on hardware.

**3. Nothing has been tested on hardware.** The CLI has never echoed a character and
the loop has never closed. First bench session should be: CLI responds → `status`
reads plausible RPM → tiny setpoint at low bus voltage → verify sign/direction.

**4. Zero duty brakes; a fault coasts.** At CCR=0 in PWM1, OCxREF is never active, so
both *low-side* FETs are on and the motor is shorted — that's the state at power-up,
at `set 0`, and when `SpeedControl_WatchdogTask()` trips `MotorDriver_Stop()`. A
break-input fault instead clears MOE with OSSR disabled, releasing outputs to Hi-Z so
the motor coasts. Both are defensible; just don't assume they're the same.

**5. The watchdog is unrequested scope.** `SpeedControl_WatchdogTask()` was added
because moving the loop onto the ADC introduced a new failure mode: a dead sampling
chain stops the loop and the bridge would hold its last duty forever. It checks every
50 ms that the tick counter advanced and calls `MotorDriver_Stop()` if not; `status`
shows `loop=STALLED`. It is *not* a substitute for the TIM1 break input. Delete the
function and its one call site in `main.c` if unwanted.

**6. Stale comment.** `motor_cli.c:372` still says "the TIM6 tick preempts thread
mode" — that moved to the ADC ISR in `c1dfd7e`. Documentation only.

**7. `TIM1 LockLevel = OFF`.** TIM1's LOCK bits are write-once after reset and can make
DTG/BKE/BKP/OSSR immutable for the power cycle — reasonable hardening for a value whose
corruption destroys the bridge. Left off because it blocks reconfiguration during
bring-up.

---

## Verification status

| Check | Result |
|---|---|
| Build, `-Wall -Wextra -O2` | clean, zero warnings |
| Flash / RAM | 61,504 B of 512 K (11.7%) / 2,740 B of 128 K (2.1%) |
| Vector table (decoded from ELF) | ADC1_2 → `ADC1_2_IRQHandler`, LPUART1 → `LPUART1_IRQHandler`, TIM6_DAC → `Default_Handler` (correctly retired) |
| `HAL_TIM_PWM_MspInit` | strong symbol in project space — the Msp rename took |
| Blocking calls in ISR paths | none; only `HAL_Delay` is in `MotorDriver_ClearFault`, thread-level only |
| Hardware | **none whatsoever** |

Toolchain used: `arm-none-eabi-gcc` from `I:\ST\STM32CubeCLT_1.21.0`.

---

## Working preferences (Claude's memory doesn't sync between machines)

- **Don't compile unless explicitly asked.** Write the code and stop; the build is
  driven from CubeIDE.
- `PMDC_Motor_Control` is the project. If `LwIP_HTTP_Server_Raw` is also attached to
  the session it is unrelated — leave it alone, separate repo and branch.
- Commits: Claude commits, the user pushes from CubeIDE.

---

## Prompt to resume in a new session

**First make sure the session is actually pointed at this repo.** In Claude Desktop's
Code tab the folder picker selects the project, not the prompt — start a session
against the wrong directory and a prompt like "continue work on X" will be read as
"create X". Clone the repo and open *that* folder before typing anything:

```
git clone https://github.com/savakrstovic/PMDC_Motor_Control.git
```

Then paste this. It verifies before it touches anything, so a wrong folder produces a
question rather than a new project:

> Before anything else: run `git log --oneline -3` and `ls` in the current working
> directory. I expect the PMDC_Motor_Control repo, with `HANDOFF.md` in the root and
> commit `41247a4` at or near HEAD.
>
> **If you don't see that, stop and tell me. Do not create, scaffold, or initialize
> anything** — it means the session is pointed at the wrong folder and I'll fix that
> before we start.
>
> If it is there: read `HANDOFF.md`, then wait for instructions. Short version —
> STM32G474 closed-loop PMDC speed control; the 1 kHz loop runs from the ADC
> end-of-conversion interrupt paced by TIM6 TRGO; serial CLI on LPUART1 at 209700 baud;
> TIM1 drives the bridge at 20 kHz with 1 µs dead time. It builds clean but has never
> been run on hardware.
>
> Don't compile unless I ask — I build from CubeIDE.

Then say what you actually want, e.g.:

- "I've done the first bench test, here's what happened: ..."
- "Help me tune the PI gains — here's a step response I captured."
- "Fix open item #6, the stale comment in motor_cli.c."
- "Walk me through what to check on the scope before I apply bus voltage."
