#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Full-scale magnitude for MotorDriver_SetDuty(); -1000..1000 maps to
 * -100.0%..+100.0% commanded bridge voltage. */
#define MOTOR_DUTY_MAX 1000

void MotorDriver_Init(void);
void MotorDriver_SetDuty(int16_t duty_permille);
void MotorDriver_Stop(void);
bool MotorDriver_IsFaulted(void);
void MotorDriver_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
