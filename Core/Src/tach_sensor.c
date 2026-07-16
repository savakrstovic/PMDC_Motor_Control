#include "tach_sensor.h"
#include "adc.h"

/* Analog frontend centers zero speed at ADC mid-scale (VDDA/2 into a 12-bit
 * ADC) and scales ~1.8 rpm per code, sign included -- see CLAUDE.md. */
#define TACH_ADC_MIDSCALE_CODE 2048
#define TACH_RPM_PER_CODE      1.8f
#define TACH_ADC_TIMEOUT_MS    2U

float TachSensor_ReadRpm(void)
{
  uint32_t raw = TACH_ADC_MIDSCALE_CODE;

  if (HAL_ADC_Start(&hadc1) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc1, TACH_ADC_TIMEOUT_MS) == HAL_OK)
    {
      raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
  }

  int32_t signed_code = (int32_t)raw - TACH_ADC_MIDSCALE_CODE;
  return (float)signed_code * TACH_RPM_PER_CODE;
}
