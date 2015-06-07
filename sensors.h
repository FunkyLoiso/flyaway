#ifndef SENSORS_H_INCLUDED
#define SENSORS_H_INCLUDED

/* PCA9685   16-channel 12-bit PWM controller
 * ADXL345   3 axis digital accelerometer
 * ITG-3200  3-axis MEMS gyroscope
 * HMC5883   3-axis compass
 * BMP085    pressure + temperature
 * nRF24L01+ somwe radio                      */

#include <math.h>

#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883.h"
#include "BMP085.h"

#include "vector.h"

static const int32_t std_sealevel_pressure = 101325;

typedef struct {
  vector_double_3d acc_data;  /* ADXL345 accelerations, m/s^2 */
  vector_double_3d avel_data; /* ITG3200 angular velocicites, deg/s */
  double itg3200_temp;        /* ITG3200 temperature, deg c */
  vector_double_3d mag_data;  /* HMC5883 magnetic inductions, gauss  */
  int32_t bmp085_temp;        /* BMP085 temperature, deg c */
  double altitude;            /* BMP085 altitude, m */
} SENSOR_DATA;

double pressure_to_altitude(int32_t sealevel_pressure, int32_t pressure);
int init_sensors();
int read_sensors(SENSOR_DATA* data);

#endif
