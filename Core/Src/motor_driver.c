#include "motor_driver.h"
#include "main.h"
#include "tim.h"

/* Board's protection fault line is assumed active-low (idle high, pulled low
 * on trip) -- the common convention for comparator/gate-driver FAULT outputs.
 * TIM1_BKIN polarity in tim.c (TIM_BREAKPOLARITY_LOW) must match this. */
#define FAULT_CLEAR_PULSE_ACTIVE_HIGH 1

/* TIM1 period (ARR) + 1: full PWM count range used as the CCR full-scale. */
#define PWM_COUNT_FULL_SCALE 8500U

static uint32_t DutyToCounts(int16_t magnitude_permille)
{
  return ((uint32_t)magnitude_permille * PWM_COUNT_FULL_SCALE) / MOTOR_DUTY_MAX;
}

void MotorDriver_Init(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

/* Sign-magnitude drive: the leg on the commanded side of zero PWMs at
 * |duty|, the other leg is held static low. Both legs always run through
 * TIM1's own hardware dead-time/break logic, so this never needs to reason
 * about shoot-through directly -- only which leg is active. */
void MotorDriver_SetDuty(int16_t duty_permille)
{
  if (duty_permille > MOTOR_DUTY_MAX)
  {
    duty_permille = MOTOR_DUTY_MAX;
  }
  else if (duty_permille < -MOTOR_DUTY_MAX)
  {
    duty_permille = -MOTOR_DUTY_MAX;
  }

  uint32_t counts = DutyToCounts(duty_permille < 0 ? (int16_t)-duty_permille : duty_permille);

  if (duty_permille >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, counts);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, counts);
  }
}

void MotorDriver_Stop(void)
{
  MotorDriver_SetDuty(0);
}

bool MotorDriver_IsFaulted(void)
{
  /* MOE reads back 0 whenever TIM1's break input has tripped and outputs
   * are hardware-disabled -- the authoritative "is the bridge live" bit. */
  return (htim1.Instance->BDTR & TIM_BDTR_MOE) == 0U;
}

void MotorDriver_ClearFault(void)
{
  MotorDriver_Stop();

#if FAULT_CLEAR_PULSE_ACTIVE_HIGH
  HAL_GPIO_WritePin(FAULT_CLEAR_GPIO_Port, FAULT_CLEAR_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(FAULT_CLEAR_GPIO_Port, FAULT_CLEAR_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(FAULT_CLEAR_GPIO_Port, FAULT_CLEAR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(FAULT_CLEAR_GPIO_Port, FAULT_CLEAR_Pin, GPIO_PIN_SET);
#endif

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
  __HAL_TIM_MOE_ENABLE(&htim1);
  /* BKIN is level-sensitive: if the fault condition is still asserted this
   * immediately re-trips the break and MotorDriver_IsFaulted() stays true. */
}
