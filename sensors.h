#ifndef SENSORS_H_INCLUDED
#define SENSORS_H_INCLUDED

/* PCA9685   16-channel 12-bit PWM controller
 * ADXL345   3 axis digital accelerometer
 * ITG-3200  3-axis MEMS gyroscope
 * HMC5883   3-axis compass
 * BMP085    pressure + temperature
 * nRF24L01+ some radio                      */

#include <math.h>

#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883.h"
#include "BMP085.h"

#include "vector.h"

static const int32_t std_sealevel_pressure = 101325;

typedef struct {
  sensor_sample_3d acc_data;  /* ADXL345 accelerations, g */
  sensor_sample_3d avel_data; /* ITG3200 angular velocities, deg/s */
  sensor_sample itg3200_temp; /* ITG3200 temperature, deg c */
  sensor_sample_3d mag_data;  /* HMC5883 magnetic inductions, gauss  */
  sensor_sample bmp085_temp;  /* BMP085 temperature, deg c */
  sensor_sample altitude;     /* BMP085 altitude, m */
} sensor_data;

double pressure_to_altitude(int32_t sealevel_pressure, int32_t pressure);
int init_sensors();
int read_sensors(sensor_data* data);

#endif
