#include <stdbool.h>
#include "speed_control.h"
#include "motor_driver.h"
#include "tach_sensor.h"

/* 1kHz sample rate. Placeholder gains for ~50-100Hz closed-loop bandwidth
 * per CLAUDE.md -- retune Kp/Ki against the actual motor/load on hardware. */
#define CONTROL_PERIOD_S 0.001f
#define SPEED_KP         2.0f
#define SPEED_KI         40.0f
#define OUTPUT_MIN       (-(float)MOTOR_DUTY_MAX)
#define OUTPUT_MAX       ((float)MOTOR_DUTY_MAX)

static float s_setpoint_rpm = 0.0f;
static float s_integrator = 0.0f;

void SpeedControl_Init(void)
{
  s_setpoint_rpm = 0.0f;
  s_integrator = 0.0f;
  MotorDriver_Init();
}

void SpeedControl_SetSetpointRpm(float rpm)
{
  s_setpoint_rpm = rpm;
}

void SpeedControl_Task(void)
{
  if (MotorDriver_IsFaulted())
  {
    s_integrator = 0.0f;
    return;
  }

  float measured_rpm = TachSensor_ReadRpm();
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
}
