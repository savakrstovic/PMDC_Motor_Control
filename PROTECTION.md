# Protection, braking and power-stage interface

Written 2026-07-27. **Revised 2026-11-19** against the Callan M4-420X electrical
specification (datasheet) and the user's answers about the target system. Companion to
[HANDOFF.md](HANDOFF.md), which covers the firmware architecture.

**Nothing here has been tested on hardware.** Numbers marked "derived" are computed from
datasheet values; numbers marked "estimated" are not from any document.

### What the 2026-11-19 revision changed

- **The armature resistance estimate was wrong by ~18x.** The first version took
  Ra = 115 V / 120 A = 0.96 Ω. The datasheet gives **0.052 Ω** for the M4-4205D: the
  120 A figure is the motor's *peak current rating*, not V/R. Every current figure in the
  first version was low by roughly that factor.
- **Consequently the tacho-only feedforward current clamp (old §3) does not work on
  these motors** and has been replaced by a cascaded speed → current loop (§3). A
  current sensor is now mandatory, not "later".
- **Shorting the motor (both low-sides on) is not a safe stop** — it is >1000 A from
  speed. The safe state is outputs off (Hi-Z, coast).
- **Tacho is 9 V/krpm**, not 9.5. The existing `TACH_RPM_PER_CODE = 1.8f` is
  essentially right (1.790); the pending change to 1.696 was wrong and is withdrawn.
- **The driver must support three motor types**, one of them as a pair on one bridge —
  parameters move from compile-time constants to per-motor profiles (§2, §5).
- **The 90 A "board limit" is not a board rating** — it is the measured worst-case
  current in the twin-4209D application (§4).

---

## 1. The failure this project must not repeat

On an earlier prototype — different board (IGBT bridge), different MCU (an old PIC),
control algorithm unknown and unavailable — the following happened:

- Motor accelerated to 2400 rpm, **no load on the shaft**, only rotor inertia
- A switch was closed simulating the "end of movement" signal
- After roughly 1–2 seconds, one transistor failed destructively

### Diagnosis

The 1–2 second delay rules out shoot-through (microseconds) and instantaneous voltage
breakdown, so this was most likely a **thermal or energy-accumulation** failure. With the
datasheet resistance, three candidate mechanisms are all far worse than first estimated:

| Mechanism | What happens on an M4-4205D from 2400 rpm |
|---|---|
| Plugging (reverse voltage) | `(V + E) / R` = (115 + 85) / 0.052 ≈ 3800 A, inductance-limited only for the first ms |
| Dynamic braking (motor shorted) | `E / R` = 85 / 0.052 ≈ 1600 A |
| Regenerative bus pumping | ~400 J of rotor energy pushed back into the DC link (§3) |

It cannot be settled which one it was, and it does not need to be: the design in §3
(current loop + coast as the safe state + braking energy dumped into a resistor) covers
all three. The failed device may also have been an antiparallel **diode** — under
braking and regeneration the motor is the source and current returns through them.

### Why the test bench is the dangerous configuration

In the real system each motor drives a **self-locking gearbox, ratio roughly
8000–9000:1, efficiency roughly 0.16** (user's recollection, to be confirmed). Two
consequences:

- **Reflected load inertia is negligible** (÷ ratio², ~10⁻⁸), so the rotor's own
  inertia is essentially all the stored energy in both bench and field.
- In the field, gearbox friction absorbs that energy when drive is removed, so a
  coasting motor stops quickly and little energy returns to the bus. For example, a
  4209D running at 40 A carries ~24 Nm of load torque and would coast from 1800 rpm to
  rest in roughly 0.2 s (estimated — depends on the real running current).

**On the bench there is no gearbox**: a coasting 4205D takes ~4–5 s to stop (motor friction
only), and any active braking sends almost all the rotor energy into the bus. The bench
is the more hazardous setup, and it is the one being used for driver bring-up.

---

## 2. Motors

The driver must run three motor types. Values are from the M4-420X electrical
specification; the 4208 winding is not yet known, so all four are listed.

| Parameter | M4-4205D | M4-4209D | M4-4208 A / B / C / D |
|---|---|---|---|
| Back-EMF Ke (V/krpm) | 35.5 | 63.9 | 142.0 / 113.1 / 85.2 / 56.8 |
| Torque const. Kt (Nm/A) | 0.339 | 0.61 | 1.36 / 1.08 / 0.81 / 0.54 |
| **Resistance R (Ω)** | **0.052** | **0.079** | 0.453 / 0.287 / 0.172 / 0.072 |
| **Inductance L (mH)** | **0.275** | **0.49** | 2.73 / 1.75 / 0.98 / 0.44 |
| Cont. current Io (A) | 46.0 | 44.4 | 20.0 / 25.0 / 31.7 / 44.9 |
| Peak current Ip (A) | 120 | 120 | 120 |
| Cont. / peak torque (Nm) | 15.6 / 38 | 27 / 69 | 27/142, 27/119, 26/92, 24/62 |
| Rated max speed (rpm) | 3200 | 2100 | 975 / 1200 / 1600 / 2400 |
| Max terminal voltage (V) | 140 | 140 | 140 |
| Rotor inertia JM (kg·m²) | 0.014 | 0.0241 | 0.0217 |
| Static friction (Nm) | 0.54 | 0.95 | 0.81 |
| Viscous damping (Nm/krpm) | 0.16 | 0.30 | 0.26 |
| Tacho | 9 V/krpm (custom) | 9 V/krpm (assumed) | 9 V/krpm (assumed) |

Notes:

- **Tacho 9 V/krpm** for the 4205D comes from a separate order document (custom
  variant; the datasheet's standard TGF2030-147 options are 19/38/47/57 V/krpm). It is
  *assumed* for the 4209D and 4208 until their order documents are checked. Tacho
  inertia (0.64 kg·cm²) is negligible.
- **The 4205D in practice runs at ~2500 rpm or less**, never its 3200 rpm rating (user).
- Ke and Kt are mutually consistent in SI (35.5 V/krpm = 0.339 V·s/rad), which is a good
  sign the table was transcribed correctly.
- Datasheet typos not affecting these motors: 4209**A** inductance reads "308" (surely
  3.08 mH); 4207 inertia reads "0.186" (surely 0.0186).

### Twin 4209D on one bridge

In one application **a single driver runs two 4209D motors in parallel**. Seen from the
bridge that is one equivalent motor:

| | Value |
|---|---|
| R | 0.0395 Ω |
| L | 0.245 mH |
| Ke | 63.9 V/krpm (unchanged) |
| Continuous current | 88.8 A total |
| Rotor inertia | 0.0482 kg·m² |
| Measured worst-case current | **≤ 90 A total** (user, already measured) |

The bridge current sensor sees only the **total**; how it splits between the two motors
is invisible to the firmware. Paralleled low-resistance motors share current poorly — if
they run at the same speed, a 1% Ke mismatch at 1800 rpm is ~1.15 V driving ~7 A
through the two windings in series, so one motor carries ~7 A above the average and the
other ~7 A below. A per-motor current limit cannot be enforced, only a total one.

### Derived quantities that drive the design

| | 4205D | 4209D | Twin 4209D | 4208A … 4208D |
|---|---|---|---|---|
| Electrical time const. L/R | 5.3 ms | 6.2 ms | 6.2 ms | 6.0 … 6.1 ms |
| Max no-load speed on 115 V bus | ≈ 3200* | **≈ 1800** | ≈ 1800 | ≈ 810 … 2020 |
| Stall current at 115 V (winding R only) | 2200 A | 1460 A | 2900 A | 254 … 1600 A |
| di/dt with full bus across L | 420 A/ms | 235 A/ms | 470 A/ms | 42 … 260 A/ms |
| Rotor energy at practical max speed | 480 J @ 2500 | 430 J @ 1800 | 860 J | ≈ 80 … 480 J |
| PWM ripple, 20 kHz, 115 V, worst case | 5.2 A pk-pk | 2.9 A | 5.9 A | 0.5 … 3.3 A |

\* minus I·R, bridge drop and dead-time loss — in practice below 3200.

- **The 115 V bus caps speed below rating** for the 4209D and every 4208 winding; they
  are rated near the 140 V terminal maximum. Rated speed would need a higher bus (the
  200 V MOSFETs allow up to roughly 140 V).
- **Inductance spans 10x across the set** (0.245–2.73 mH). Current-loop gains scale
  with L, so one gain set cannot serve all motors (§3).
- **J/Kt is ~0.040 for all three D windings** (0.041 / 0.040 / 0.040), so the speed-loop
  gains carry over between them unloaded; the gearbox load changes this in the field.

---

## 3. Control and braking

### Why the original feedforward clamp fails

The first version proposed limiting current with no current sensor:
`V_command = Ke × rpm ± I_limit × Ra`. The allowed voltage window is `I_limit × R`, and
with the real resistance it is tiny:

| Motor | Window at 40 A | Share of 115 V bus |
|---|---|---|
| 4205D | ±2.1 V | ±1.8% duty |
| 4208A (best case) | ±9 V at 20 A | ±7.9% duty |

Errors that are routine in an open-loop voltage estimate exceed it:

- **Dead time**: 1 µs of a 50 µs period ≈ 2% of bus ≈ 2.3 V — alone larger than the
  4205D window.
- **Ke tolerance**: 1% error at 2400 rpm ≈ 0.85 V ≈ 16 A on the 4205D.
- **Bus voltage**: assumed, not measured — 1% ≈ 1.15 V ≈ 22 A.

**A current sensor and a real current loop are mandatory.**

### Architecture: cascaded speed → current control

```
1 kHz   tach ──▶ speed PI ──▶ I_ref, clamped to ±I_limit (per motor)
20 kHz  SENSE_I_AC ──▶ current PI ──▶ duty ──▶ TIM1
        board fault latch (FO) ──▶ TIM1_BKIN ──▶ outputs Hi-Z   (hardware, µs)
```

- **The current loop must run at the PWM rate** (10–20 kHz). With the full bus across
  the winding the current moves 235–470 A per millisecond; a 1 kHz loop sees the
  overcurrent only after it has happened.
- **Sample current synchronously with the PWM**, triggered from TIM1 (TRGO/TRGO2 or a
  spare compare channel) at the point where the instantaneous value equals the average
  (mid on-time for edge-aligned, counter peak/valley for centre-aligned). Ripple is up to
  ~6 A pk-pk, so unsynchronised sampling gives a noisy loop.
- **Current-loop gains are per motor.** For a target bandwidth ωc the standard
  choice is `Kp = L·ωc`, `Ki = R·ωc` (V/A, then ÷ V_bus for duty). At 1 kHz:
  Kp ≈ 1.5 V/A (twin 4209D) … 1.7 (4205D) … 3.1 (4209D) … 17 (4208A).
- **The speed PI outputs current, not duty.** `set 0` from speed then commands
  `−I_limit` until the speed is gone — braking at a controlled current, never plugging —
  because the current loop, not the speed loop, decides the voltage. This replaces the
  old `set 0` hazard fix.
- **The hardware trip is a prerequisite, not a backstop.** From 90 A to the ACS772's
  200 A range takes ~0.25 ms on the twin 4209D. Only the board's comparator → `FO` →
  `TIM1_BKIN` path is fast enough to catch a current-loop failure.

### The safe state is coast

Outputs off (MOE cleared, Hi-Z) is the only state that is safe at any speed: the
winding current decays through the body diodes into the bus, carrying only the
inductive energy (~1–2 J at 90 A). Both low-sides on — today's `MotorDriver_Stop()`, the
power-up state and the watchdog trip — shorts the motor and is >1000 A from speed.

So: **stop, watchdog trip, fault, stall and power-up all go to Hi-Z.** Active braking is
only ever done *through the current loop*, never by shorting.

### Where the braking energy goes

A current-limited stop dissipates very little in the winding (I²R at 40 A is only 83 W
on the 4205D), so **nearly all rotor energy returns to the DC link** — on the bench,
roughly 400–450 J per motor from practical top speed, ~860 J for the twin 4209D.
Absorbing 430 J in capacitance while holding the bus below 170 V would need ~55 mF.

- **On the bench**, active braking from speed therefore needs a brake (dump) resistor,
  or a supply/battery that can sink current. Without one, brake at low current or let
  it coast — coasting puts nothing into the bus.
- **In the field** the gearbox absorbs most of it (§1), so coast may already give the
  required sharp stop. Measure before adding braking logic for the field case.
- **The board redesign includes braking resistors** (user, 2026-11-19). Not yet known:
  whether the chopper is hardware-autonomous (comparator on bus voltage) or needs an MCU
  output plus bus-voltage measurement (`SENSE_V_DC`). That decides pin allocation.

### Stop times

Bounded by current limit and rotor inertia, bench, rotor only (derived):

| | 40 A | 60 A | 90 A |
|---|---|---|---|
| 4205D from 2500 rpm | 0.25 s | 0.17 s | 0.12 s |
| 4209D from 1800 rpm | 0.18 s | 0.12 s | 0.08 s |

A sharp stop is physically available at modest current — the constraint is where the
energy goes, not the current needed.

### Stall

With the current loop in place, stalling against the mechanical hard stop holds the
current at `I_limit` instead of `V/R`. That is a **thermal** problem with seconds of
margin (83 W in the 4205D winding at 40 A, below its 46 A continuous rating), not an
instantaneous one. Stall detection becomes: speed below a threshold while `I_ref` sits at
its limit for longer than a timeout → coast, plus an I²t estimate per motor.

---

## 4. Board interface — KLN-1001-A1 (rev 1.2)

**A redesigned board is on its way** (with braking resistors). Everything in this
section is for rev 1.2 and must be re-checked against the new schematic — connector
pinout, `FO` polarity, `FLT_CLR` access and the overcurrent threshold may all change.

Read from the rev 1.2 schematic PDF via text extraction, not a rendered drawing, so net
names and connector assignments are reliable but **topology and polarity need visual
confirmation**.

### Already available on connectors — no board modification

| Signal | Where | What it is |
|---|---|---|
| `LSUP`, `LSDN` | J5 | left leg gate commands (high, low) |
| `RSUP`, `RSDN` | J6 | right leg gate commands (high, low) |
| **`FO1`** | J5 | **latched fault output** |
| **`FO2`** | J6 | latched fault output |
| `SENSE_I_AC` | J4 | **motor current** (ACS772ECB-200B, ±200 A) |
| `SENSE_I_DC` | J4 | DC-link current (ACS772ECB-200B) |
| `SENSE_V_DC` | J4 | bus voltage (Si8920 + TLV9061) |

`FO1`/`FO2` come from the board's fault flip-flop (U6), which latches on any of:
DC-link overvoltage, DC-link undervoltage, DC-link overcurrent, motor overcurrent, and
four temperature channels. The comparator protection this project needs **already exists
in hardware** — it simply is not connected to the MCU.

### Current rating

**90 A is not the board's rating.** It is the measured worst-case total current with
two 4209D motors on one driver, and the real figure is believed to be lower. The board's
actual limit is whatever its motor-overcurrent comparator trips at — **unknown, needs
reading from the schematic** (rev 1.2 and the redesign). The MOSFETs (IXFX300N20X3,
300 A) and the ACS772 (±200 A) are not the binding limits. The per-motor `I_limit` must
sit below that trip point with margin.

### Needs a small modification

`FLT_CLR` is driven only by on-board pushbutton **S1**. It is not on any connector.
Consequently `MotorDriver_ClearFault()` currently pulses PB1 into thin air and the CLI's
`clear` command does nothing to the board. Fix: parallel PB1 (as open-drain) across S1 —
or route `FLT_CLR` to a connector in the redesign.

### Wiring plan

1. **`FO1` → PA6 (`M_BKIN`)** — turns the board's latched protection into a hardware
   PWM kill via `TIM1_BRK`. **Prerequisite for powered work** (§3).
2. **`SENSE_I_AC` → spare ADC channel**, scaled to 0–3.3 V — **prerequisite for the
   current loop** (§3), no longer "later".
3. **PB1 across S1** — so `clear` works.
4. **`SENSE_V_DC` → spare ADC channel** — needed if the brake chopper is MCU-driven, and
   for converting voltage commands to duty correctly.

### Before wiring — three checks

- **`FO` polarity and drive type.** Firmware assumes active-low (idle high, pulled low
  on trip), matching `TIM_BREAKPOLARITY_LOW`. If `FO` is the opposite, flip that and
  `FAULT_CLEAR_PULSE_ACTIVE_HIGH` in `motor_driver.c`.
- **PA6 is probably not 5 V tolerant.** `FO` is `+5V_D` logic. PA6 carries an ADC
  function on the G474, and analog-capable pins are typically 3.6 V max, not FT. Check
  the datasheet pin table; a divider or series resistor plus clamp may be required.
- **Analog signals need attenuation.** ACS772 on 5 V sits at 2.5 V for zero current and
  swings roughly 0.5–4.5 V (10 mV/A). Must be scaled to 0–3.3 V. Being ratiometric, once
  divided the sensor supply and ADC reference no longer track — so sample the
  zero-current point at startup (outputs off) rather than assuming mid-scale.

---

## 5. Firmware gaps

### Required before any powered work

| Item | Why |
|---|---|
| **Safe state = coast** | `MotorDriver_Stop()`, the watchdog trip and power-up currently short the motor (both low-sides on) — >1000 A from speed. Make all of them clear MOE (Hi-Z); start with outputs disabled until explicitly enabled. |
| **`GPIO_PULLUP` on PA6** | break input is currently **floating** on an active-low break. Also set `BreakFilter` to ~4–8. (CubeMX GUI) |
| **Tacho sign, configurable** | sign convention between duty and tacho is undocumented. If it is inverted, the speed loop is positive feedback and runs away. Make it a per-motor setting and verify open-loop on the bench. |
| **Open-loop bench command** | a CLI command for a small fixed duty, so tacho sign and direction can be checked before the loop is closed. At standstill 1% duty on a 115 V bus is ~22 A in a 4205D — only at low bus voltage, from a current-limited supply. |
| `DeadTime` → `203` | 2.024 µs, matching the ~2 µs measured on this driver with a previous MCU. Current value 149 (1 µs). (CubeMX GUI) |

### Required before closed-loop running

| Item | Why |
|---|---|
| **Current measurement** | `SENSE_I_AC` on an ADC channel, triggered from TIM1, zero-offset calibrated at startup. Needs CubeMX changes. |
| **Current loop at PWM rate** | §3. Replaces the feedforward clamp, which is withdrawn. |
| **Speed PI outputs current** | §3. Removes plugging structurally. Current `SPEED_KP = 2.0f` saturates to full reverse duty for any error above 500 rpm. |
| **Per-motor profiles** | Ke, R, L, I_limit (continuous and peak), tacho constant and sign, max rpm, current-loop gains. A `MotorProfile` struct selected at runtime. 4205D, 4209D, twin 4209D, 4208 (winding TBD). |
| **Motor selection that can't silently be wrong** | jumper/DIP, ID resistor, or CLI + flash. Wrong profile = wrong current limit and gains. Undecided. |
| **Stall detection** | speed < threshold while `I_ref` is at its limit for > timeout → coast; plus I²t per motor (§3). |
| **End-of-movement switch input** | the stop source in the field (platform can't be changed). Pin, NO/NC and debounce undecided; it should command the stop directly in firmware, not via the CLI. |
| **Duty cap below 100%** | if the SI8273 high sides are bootstrap-supplied, 100% duty can't be held. Matters most for the 4205D, which needs nearly the full bus. |
| `TACH_RPM_PER_CODE` → `1.790f` | 9 V/krpm × 0.05 attenuation ÷ 0.8057 mV/code. Current `1.8f` is 0.56% high — minor. (The earlier proposed `1.696f` assumed 9.5 V/krpm and is **withdrawn**.) The front end fits ±28.8 V = 3200 rpm at 9 V/krpm, so all three motors fit. |
| ~~CLI setpoint limit → ±3200~~ | **done 2026-09-03**. Should become per-motor max rpm (4205D ~2500 in practice). |

### Deliberate divergences from the design brief

- Brief specifies continuous ADC DMA + FMAC filtering. Built instead as TIM6 TRGO →
  single conversion → EOC interrupt, because a hardware-triggered sample at a
  deterministic instant is worth more to a control loop, and it makes the tick
  hard-bounded. For brush-noise rejection, use the G4's **ADC hardware oversampler**
  (8–16x, currently `OversamplingMode = DISABLE`) rather than DMA + FMAC.
- Brief specifies a 0.25 mΩ shunt + internal PGA. The board instead has two ACS772
  Hall sensors with conditioned outputs — isolated, no shunt dissipation, and **no PGA
  needed**. The protection chain becomes ACS772 → divider → COMP → `TIM1_BRK`.
- Brief specifies break state = both low-side ON (active braking). **Rejected**: from
  speed that is a >1000 A short (§3), and on a break event the outputs jump to idle with
  the dead-time generator bypassed, adding ~645 ns of real overlap at Rg = 10 Ω during an
  overcurrent. Break → Hi-Z (current config: `OSSI_DISABLE`, idle states `RESET`) stays.

---

## 6. Open questions and measurements

| Item | Status / how | Why it matters |
|---|---|---|
| **4208 winding** (A/B/C/D) | user to find in documents / nameplate | R, L, Ke, current limits differ up to 10x |
| **Tacho constant for 4209D and 4208** | assumed 9 V/krpm; check order documents | wrong value = wrong speed, possible ADC saturation |
| **Tacho sign vs drive direction** | possibly in documentation; otherwise verify open-loop on the bench | inverted sign = runaway |
| **Twin 4209D: which tacho feeds the loop?** | one tach ADC channel today | the other motor's speed is unobserved |
| **Board overcurrent trip threshold** | rev 1.2 schematic, then redesign | sets the real ceiling for `I_limit` |
| **Brake chopper: hardware or MCU-driven?** | redesign schematic | pin allocation, need for `SENSE_V_DC` |
| **End-of-movement switch** | pin, NO/NC, who reads it | the field stop path |
| **SI8273 high-side supply** | bootstrap or isolated? | max sustainable duty |
| Gearbox ratio / efficiency | user recollection ~8000–9000:1, η ≈ 0.16 | field stop time, field regen |
| Running current per motor in the field | measure | field load torque → coast-down time |
| `FO` polarity / drive | schematic or meter | sets break polarity in firmware |
| PA6 5 V tolerance | G474 datasheet pin table | whether `FO` can be wired directly |
| Dead time at the gates | scope V_GS at the FET pins, both devices of one leg | confirms 2 µs is adequate at Rg = 10 Ω |
| Tacho constant, actual | spin shaft by drill, bridge unpowered: `K_tach = 35.5 × V_tach / V_motor` (4205D) | calibrates the whole tach front end against the datasheet Ke |

Ke and R no longer need measuring for the design — the datasheet gives them. A quick
four-wire R check is still worthwhile to catch cabling and connector resistance, which at
these values is not negligible.
