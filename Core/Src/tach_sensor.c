#include "tach_sensor.h"
#include "adc.h"
#include "main.h"

/* Analog frontend centers zero speed at ADC mid-scale (VDDA/2 into a 12-bit
 * ADC) and scales ~1.8 rpm per code, sign included -- see CLAUDE.md. */
#define TACH_ADC_MIDSCALE_CODE 2048
#define TACH_RPM_PER_CODE      1.8f

/* Written by the ADC EOC interrupt, read by the control loop in that same
 * interrupt and by the CLI at thread level. A 32-bit load/store is single-copy
 * atomic on Cortex-M4, so a reader can never see a torn value. */
static volatile float s_latest_rpm = 0.0f;

void TachSensor_Init(void)
{
  s_latest_rpm = 0.0f;

  /* ExternalTrigConv is TIM6_TRGO, so this arms the ADC once and every
   * subsequent trigger converts in hardware. HAL only tears the EOC interrupt
   * back down when the trigger source is software start (see the
   * LL_ADC_REG_IsTriggerSourceSWStart branch in HAL_ADC_IRQHandler), so with a
   * timer trigger the interrupt stays armed indefinitely -- no re-arming, and
   * no window where a trigger could be missed. */
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

void TachSensor_OnConversionComplete(void)
{
  int32_t signed_code = (int32_t)HAL_ADC_GetValue(&hadc1) - TACH_ADC_MIDSCALE_CODE;
  s_latest_rpm = (float)signed_code * TACH_RPM_PER_CODE;
}

float TachSensor_LatestRpm(void)
{
  return s_latest_rpm;
}
