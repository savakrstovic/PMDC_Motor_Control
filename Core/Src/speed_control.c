#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "speed_control.h"
#include "motor_driver.h"
#include "tach_sensor.h"

/* 1kHz sample rate. Placeholder gains for ~50-100Hz closed-loop bandwidth
 * per CLAUDE.md -- retune Kp/Ki against the actual motor/load on hardware. */
#define CONTROL_PERIOD_S 0.001f
/* How often the thread-level stall check runs, and how many ticks may be
 * missed before the bridge is stopped. 50ms is ~50 ticks -- long enough that
 * it never trips on jitter, short enough to matter mechanically. */
#define WATCHDOG_PERIOD_MS   50U
#define WATCHDOG_MIN_TICKS   1U
#define SPEED_KP         2.0f
#define SPEED_KI         40.0f
#define OUTPUT_MIN       (-(float)MOTOR_DUTY_MAX)
#define OUTPUT_MAX       ((float)MOTOR_DUTY_MAX)

static float s_setpoint_rpm = 0.0f;
static float s_integrator = 0.0f;

/* Published for telemetry consumers (the CLI). Written only by
 * SpeedControl_Task in the TIM6 ISR, read only under the critical section in
 * SpeedControl_GetTelemetry. */
static float   s_measured_rpm = 0.0f;
static int16_t s_output_duty = 0;

/* Incremented every tick; the watchdog watches it advance. */
static volatile uint32_t s_tick_count = 0U;
static volatile bool     s_loop_stalled = false;

void SpeedControl_Init(void)
{
  s_setpoint_rpm = 0.0f;
  s_integrator = 0.0f;
  s_measured_rpm = 0.0f;
  s_output_duty = 0;
  s_tick_count = 0U;
  s_loop_stalled = false;
  MotorDriver_Init();
}

void SpeedControl_SetSetpointRpm(float rpm)
{
  s_setpoint_rpm = rpm;
}

void SpeedControl_GetTelemetry(SpeedControl_Telemetry *telemetry)
{
  if (telemetry == NULL)
  {
    return;
  }

  /* The three fields must describe the same tick, so they are copied with
   * the tick locked out. Costs well under a microsecond of added jitter. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  telemetry->setpoint_rpm = s_setpoint_rpm;
  telemetry->measured_rpm = s_measured_rpm;
  telemetry->output_duty_permille = s_output_duty;
  __set_PRIMASK(primask);
}

bool SpeedControl_IsLoopStalled(void)
{
  return s_loop_stalled;
}

void SpeedControl_WatchdogTask(void)
{
  static uint32_t last_check_ms = 0U;
  static uint32_t last_tick_count = 0U;

  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_check_ms) < WATCHDOG_PERIOD_MS)
  {
    return;
  }
  last_check_ms = now_ms;

  uint32_t ticks = s_tick_count;

  /* The loop is clocked by ADC end-of-conversion, so a sampling chain that
   * dies silently would leave the bridge holding its last duty forever. This
   * is the backstop for that -- it is not a replacement for the TIM1 break
   * input, which is still what catches real hardware faults. */
  if ((uint32_t)(ticks - last_tick_count) < WATCHDOG_MIN_TICKS)
  {
    if (!s_loop_stalled)
    {
      s_loop_stalled = true;
      MotorDriver_Stop();
    }
  }
  else
  {
    s_loop_stalled = false;
  }

  last_tick_count = ticks;
}

void SpeedControl_Task(void)
{
  s_tick_count++;

  /* Sampled before the fault check so telemetry keeps showing the real
   * coast-down speed while the bridge is latched off. */
  float measured_rpm = TachSensor_LatestRpm();

  if (MotorDriver_IsFaulted())
  {
    s_integrator = 0.0f;
    s_measured_rpm = measured_rpm;
    s_output_duty = 0;
    return;
  }

  float error = s_setpoint_rpm - measured_rpm;

  float p_term = SPEED_KP * error;
  float integrator_tentative = s_integrator + SPEED_KI * error * CONTROL_PERIOD_S;
  float unsaturated = p_term + integrator_tentative;

  bool saturated_high = (unsaturated > OUTPUT_MAX);
  bool saturated_low = (unsaturated < OUTPUT_MIN);

  float output;
  if (saturated_high)
  {
    output = OUTPUT_MAX;
  }
  else if (saturated_low)
  {
    output = OUTPUT_MIN;
  }
  else
  {
    output = unsaturated;
  }

  /* Conditional-integration anti-windup: freeze the integrator only when
   * saturating AND the error would push further into the same rail. */
  if (!((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f)))
  {
    s_integrator = integrator_tentative;
  }

  MotorDriver_SetDuty((int16_t)output);

  s_measured_rpm = measured_rpm;
  s_output_duty = (int16_t)output;
}
