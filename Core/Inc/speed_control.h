#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Snapshot of what the last 1kHz tick acted on. */
typedef struct
{
  float   setpoint_rpm;
  float   measured_rpm;
  int16_t output_duty_permille; /* as passed to MotorDriver_SetDuty() */
} SpeedControl_Telemetry;

void SpeedControl_Init(void);
void SpeedControl_SetSetpointRpm(float rpm);

/* Consistent copy of the three loop variables. Takes a few-cycle critical
 * section, so call it from thread mode -- not from the tick itself. */
void SpeedControl_GetTelemetry(SpeedControl_Telemetry *telemetry);

/* Runs one PI update; call from the TIM6 1kHz tick. */
void SpeedControl_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CONTROL_H */
