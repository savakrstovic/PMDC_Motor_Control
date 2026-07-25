#ifndef TACH_SENSOR_H
#define TACH_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Arms hardware-triggered sampling. TIM6's update event drives TRGO, which
 * starts one ADC1 conversion per 1kHz tick with no CPU involvement; the
 * end-of-conversion interrupt latches the result. Call once, after
 * MX_ADC1_Init (which calibrates) and before TIM6 is started. */
void TachSensor_Init(void);

/* Latches one completed conversion. ADC end-of-conversion interrupt only. */
void TachSensor_OnConversionComplete(void);

/* Most recently latched sample as signed rpm -- positive/negative encodes
 * direction per the tach's bipolar analog frontend. Non-blocking: returns the
 * value the sampling chain already produced, it never waits on the ADC. */
float TachSensor_LatestRpm(void);

#ifdef __cplusplus
}
#endif

#endif /* TACH_SENSOR_H */
