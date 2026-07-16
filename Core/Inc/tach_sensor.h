#ifndef TACH_SENSOR_H
#define TACH_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Signed rpm, positive/negative encodes direction per the tach's bipolar
 * analog frontend. */
float TachSensor_ReadRpm(void);

#ifdef __cplusplus
}
#endif

#endif /* TACH_SENSOR_H */
