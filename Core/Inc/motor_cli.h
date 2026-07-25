#ifndef MOTOR_CLI_H
#define MOTOR_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Serial console on LPUART1 (PA2/PA3 -> ST-LINK VCP), 209700 8N1.
 *
 * Commands, one per line, terminated by CR and/or LF:
 *   set <rpm>      signed target speed, e.g. "set 500", "set -500", "set 0"
 *   clear          pulse the hardware fault-clear line and re-enable the bridge
 *   status         one-shot Setpoint / Measured / Duty / Fault line
 *   stream <ms>    repeat the status line every <ms> (50..10000)
 *   stream off     stop repeating
 *   help           command list
 *
 * Call MotorCli_Init() once after MX_LPUART1_UART_Init() and
 * SpeedControl_Init(), then MotorCli_Task() from the main loop. */

void MotorCli_Init(void);

/* Parses buffered input and emits queued output. Thread mode only --
 * never call from an ISR (it can reach HAL_Delay via MotorDriver_ClearFault). */
void MotorCli_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CLI_H */
