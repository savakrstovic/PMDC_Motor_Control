# Protection, braking and power-stage interface

Written 2026-07-27. Companion to [HANDOFF.md](HANDOFF.md), which covers the firmware
architecture. This one covers the power stage: how the drive can destroy itself, the
braking law that prevents it, and what still needs wiring and measuring.

**Nothing here has been tested on hardware.** Numbers marked "estimated" are derived
from datasheet and brief values, not measurement.

---

## 1. The failure this project must not repeat

On an earlier prototype — different board (IGBT bridge), different MCU (an old PIC),
control algorithm unknown and unavailable — the following happened:

- Motor accelerated to 2400 rpm, **no load on the shaft**, only rotor inertia
- A switch was closed simulating the "end of movement" signal
- After roughly 1–2 seconds, one transistor failed destructively

### Diagnosis

The 1–2 second delay is the most informative detail. It rules out shoot-through
(microseconds) and rules out voltage breakdown (effectively instant). A delay of that
order is a **thermal failure**: a device conducted far too much current until its
junction reached destructive temperature.

The most likely cause is **plugging** — the controller reacting to a large speed error
by commanding voltage opposing the rotation. Applied voltage and back-EMF then add
across the armature instead of subtracting:

```
normal running :  I = (V − E) / Ra     V and E nearly cancel, current is small
plugging       :  I = (V + E) / Ra     they add, current is roughly 2x stall
```

Confidence: moderate. Two other mechanisms fit the same evidence — sustained dynamic
braking on a marginal device, or regenerative bus pumping until something broke down.
The old algorithm is unavailable, so this cannot be settled. **It does not need to be:
the mitigations in section 3 cover all three.**

Note also that the failed device may well have been an antiparallel **diode** rather
than the transistor itself. Under plugging and regeneration the motor is a source and
current flows back into the bridge through the freewheeling diodes, which in IGBT
co-packs are routinely rated below the IGBT.

### Why the test bench is the dangerous configuration

In the real system the motor drives a self-locking gearbox, which absorbs the rotor's
kinetic energy mechanically and cannot back-drive. **On the bench there is no such
load**, so all of that energy has to leave electrically. The bench is the more
hazardous setup, and it is the one being used for driver bring-up.

---

## 2. Motor electrical model

From the system brief (Callan M4-4205D):

| Parameter | Value | Source |
|---|---|---|
| DC bus | 115 V | brief |
| Nominal current | 46 A | brief |
| Stall current | 120 A | brief |
| Max speed | 3200 rpm | brief |
| Tacho | 9.5 V / 1000 rpm | brief |
| **Ra (armature)** | **≈ 0.96 Ω** | estimated, 115 V / 120 A |
| **Ke** | **0.022–0.036 V/rpm** | estimated — see below |

`Ke` is not pinned down because the brief does not say whether 3200 rpm is the loaded
or no-load figure. That ambiguity propagates into every current estimate below as a
roughly ±25% band. **Measuring it is the single highest-value bench measurement** — see
section 6.

### Current for each stopping method, at 2400 rpm

Back-EMF at 2400 rpm is somewhere in 53–86 V given the `Ke` uncertainty.

| Method | Applied V | Current | Verdict |
|---|---|---|---|
| Coast (all devices off) | — | 0 A | safe, slow |
| Electrical coast | `V = E` | 0 A | no braking |
| Dynamic brake (both low-side on) | `V = 0` | **55–90 A** | at the 90 A board limit |
| **Plugging (full reverse)** | `V = −115 V` | **175–210 A** | destroys hardware |

Two conclusions:

1. Plugging must be made **structurally impossible**, not merely discouraged.
2. Even ordinary dynamic braking from full speed sits at the board's 90 A limit. On
   this hardware, uncontrolled shorting of the motor is not a safe stop either.

---

## 3. The braking law

The construction team requires a **sharp stop** — a deceleration profile is not
acceptable, and there is a mechanical hard-stop downstream of the switch. So the
problem is: stop as fast as the hardware physically allows, and no faster.

### Reverse voltage is never needed

Braking only requires the applied voltage to fall **below** the back-EMF, not to go
negative:

```
I = (V − E) / Ra        braking is simply V < E
```

Shorting the motor (`V = 0`) already produces braking current at the board's limit.
Reversing does not brake better — it piles current on top of a brake that is already
saturated. **Reverse has no useful role in stopping this motor.**

### Current-limited braking

Braking torque is proportional to current, and current is what destroys devices. So the
fastest safe stop holds the maximum permitted braking current *constant* all the way
down: constant current → constant torque → constant deceleration → linear speed ramp.

Solving for the voltage that produces exactly `−I_limit`:

```
V_command = E − (I_limit × Ra)
          = (Ke × measured_rpm) − (I_limit × Ra)
```

**This needs no current sensor.** The tachometer already provides `E` every millisecond.
It is a feedforward current limit built from feedback the loop already has.

The law sequences itself correctly with no mode switching:

| Speed | Commands roughly | Physical state |
|---|---|---|
| 2400 rpm | −5 V to +28 V | reduced forward voltage |
| mid range | passing through 0 | both low-side on — dynamic braking |
| near 0 rpm | ≈ −58 V | slight reverse, holding torque as E vanishes |

Current stays flat at `I_limit` throughout, instead of spiking to 190 A.

### How sharp the stop can be

Bounded by `I_limit` and rotor inertia. At `I_limit` = 60 A the braking torque is about
30% higher than the motor produces accelerating at its 46 A rating, so the stop takes
somewhat less time than the spin-up. **That is a hard physical bound** — a faster stop
requires more current than the hardware survives. If it is not sharp enough, that is an
inertia and current-rating problem, not a firmware one.

To estimate it: time the spin-up to 2400 rpm. Stop time will be within ~1.3x of that.

### Apply the clamp to acceleration too

The same expression bounds current in the driving direction. Applying it to *all*
output, not just stopping, also fixes the `set 0` hazard described in section 5.

---

## 4. Board interface — KLN-1001-A1 (rev 1.2)

Read from the schematic PDF via text extraction, not a rendered drawing, so net names
and connector assignments are reliable but **topology and polarity need visual
confirmation**.

### Already available on connectors — no board modification

| Signal | Where | What it is |
|---|---|---|
| `LSUP`, `LSDN` | J5 | left leg gate commands (high, low) |
| `RSUP`, `RSDN` | J6 | right leg gate commands (high, low) |
| **`FO1`** | J5 | **latched fault output** |
| **`FO2`** | J6 | latched fault output |
| `SENSE_I_AC` | J4 | **motor current** (ACS772ECB-200B) |
| `SENSE_I_DC` | J4 | DC-link current (ACS772ECB-200B) |
| `SENSE_V_DC` | J4 | bus voltage (Si8920 + TLV9061) |

`FO1`/`FO2` come from the board's fault flip-flop (U6), which latches on any of:
DC-link overvoltage, DC-link undervoltage, DC-link overcurrent, motor overcurrent, and
four temperature channels. The comparator protection this project needs **already exists
in hardware** — it simply is not connected to the MCU.

### Needs a small modification

`FLT_CLR` is driven only by on-board pushbutton **S1**. It is not on any connector.
Consequently `MotorDriver_ClearFault()` currently pulses PB1 into thin air and the CLI's
`clear` command does nothing to the board. Fix: parallel PB1 (as open-drain) across S1.

### Wiring plan

1. **`FO1` → PA6 (`M_BKIN`)** — one wire, biggest single win. Turns the board's existing
   latched protection into a hardware PWM kill via `TIM1_BRK`, and makes
   `MotorDriver_IsFaulted()` meaningful.
2. **PB1 across S1** — so `clear` works.
3. **`SENSE_I_AC` → spare ADC channel** (later) — closes the loop on the feedforward
   current limit and enables real stall detection.

### Before wiring — three checks

- **`FO` polarity and drive type.** Firmware assumes active-low (idle high, pulled low
  on trip), matching `TIM_BREAKPOLARITY_LOW`. If `FO` is the opposite, flip that and
  `FAULT_CLEAR_PULSE_ACTIVE_HIGH` in `motor_driver.c`.
- **PA6 is probably not 5 V tolerant.** `FO` is `+5V_D` logic. PA6 carries an ADC
  function on the G474, and analog-capable pins are typically 3.6 V max, not FT. Check
  the datasheet pin table; a divider or series resistor plus clamp may be required.
- **Analog signals need attenuation.** ACS772 on 5 V sits at 2.5 V for zero current and
  swings roughly 0.5–4.5 V. Must be scaled to 0–3.3 V. Being ratiometric, once divided
  the sensor supply and ADC reference no longer track — so sample the zero-current point
  at startup rather than assuming mid-scale.

---

## 5. Firmware gaps

### Not yet implemented — required before bench work

| Item | Why |
|---|---|
| **`GPIO_PULLUP` on PA6** | break input is currently **floating** on an active-low break — undefined behaviour next to a 115 V bridge. Also set `BreakFilter` to ~4–8. |
| **Current-limited output clamp** | section 3. Prevents plugging structurally. |
| **Duty slew rate limiting** | limits di/dt; covers the stall-release transient |
| **Stall detection** | the mechanical hard-stop leaves the rotor stopped while the controller may still command voltage. Stall current is `V/Ra` regardless of how well the braking went. |
| `TACH_RPM_PER_CODE` → `1.696f` | currently `1.8f` — reads 6.1% high, so the loop settles ~6% below every setpoint. Derived from 9.5 V/1000 rpm × 0.05 attenuation ÷ 0.8057 mV/code. |
| `DeadTime` → `203` | 2.024 µs, matching the ~2 µs measured on this driver with a previous MCU. Current value 149 (1 µs) leaves only ~1.4x margin at Rg = 10 Ω. |
| CLI setpoint limit → ±3200 | currently ±3600; motor max is 3200 rpm |

### The `set 0` hazard

With `SPEED_KP = 2.0f`, commanding `set 0` while spinning at 2400 rpm produces:

```
error   = −2400 rpm
p_term  = 2.0 × −2400 = −4800  →  saturates to −1000‰
output  = 100% duty in reverse   ← this is plugging
```

The P term alone saturates for any error above 500 rpm, so **any large setpoint change
commands full reverse voltage** — precisely the condition that destroyed the IGBT. The
existing conditional anti-windup does not help; this is proportional, not integral.

The current-limited clamp fixes this as a side effect.

### Deliberate divergences from the design brief

- Brief specifies continuous ADC DMA + FMAC filtering. Built instead as TIM6 TRGO →
  single conversion → EOC interrupt, because a hardware-triggered sample at a
  deterministic instant is worth more to a control loop, and it makes the tick
  hard-bounded. For brush-noise rejection, use the G4's **ADC hardware oversampler**
  (8–16x, currently `OversamplingMode = DISABLE`) rather than DMA + FMAC.
- Brief specifies a 0.25 mΩ shunt + internal PGA. The board instead has two ACS772
  Hall sensors with conditioned outputs — isolated, no shunt dissipation, and **no PGA
  needed**. The protection chain becomes ACS772 → divider → COMP → `TIM1_BRK`.
- Brief specifies break state = both low-side ON (active braking). Currently configured
  as coast (`OSSI_DISABLE`, both idle states `RESET`). Changing this needs care: on a
  break event the outputs jump to their idle state with the dead-time generator
  bypassed, so a high-side that was conducting turns off while the low-side turns on —
  at Rg = 10 Ω that is ~645 ns of genuine overlap, during an overcurrent event. Safer
  to leave break → Hi-Z and have the break ISR command both low-sides on after a delay.

---

## 6. Measurements still needed

| Measure | How | Why it matters |
|---|---|---|
| **Ke** | spin at known rpm, measure terminal voltage | removes the ±25% band from every current estimate in this document |
| **Ra** | four-wire measurement at the motor terminals | same; also drifts +0.4%/°C, so allow margin |
| Spin-up time to 2400 rpm | stopwatch | gives the achievable stop time to within ~1.3x |
| Dead time at the gates | scope V_GS at the FET pins, both devices of one leg | confirms 2 µs is adequate at Rg = 10 Ω |
| `FO` polarity / drive | schematic or meter | sets break polarity in firmware |
| PA6 5 V tolerance | G474 datasheet pin table | determines whether `FO` can be wired directly |

Until `Ke` and `Ra` are measured, run `I_limit` conservatively — 40 A rather than 60 A.
Feedforward has no way to notice it is wrong.
