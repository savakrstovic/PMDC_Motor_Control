#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void SpeedControl_Init(void);
void SpeedControl_SetSetpointRpm(float rpm);

/* Runs one PI update; call from the TIM6 1kHz tick. */
void SpeedControl_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CONTROL_H */
