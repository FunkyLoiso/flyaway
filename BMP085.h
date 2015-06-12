#ifndef BMP085_H_INCLUDED
#define BMP085_H_INCLUDED

#include <stdint.h>
#include "vector.h"

typedef enum {
  BMP085_1_INT_SAMPLE = 0,  /* Ultra low power, 3 + 1.5 = 4.5 ms delay */
  BMP085_2_INT_SAMPLES = 1, /* Standard, 2*3 + 1.5 = 7.5 ms delay */
  BMP085_4_INT_SAMPLES = 2, /* High resolution, 4*3 + 1.5 = 13.5 ms delay */
  BMP085_8_INT_SAMPLES = 3  /* Ultra high resolution, 8*3 + 1.5 = 25.5 ms delay */
} BMP085_OVERSAMPLING_SETTING;

int BMP085_init(unsigned char adr, BMP085_OVERSAMPLING_SETTING oss);

int BMP085_schedule_press_update();
int BMP085_schedule_press_temp_update(); /* blocks for 4.5 ms  */

void BMP085_read_temp(sensor_sample* temperature, int* is_new_value); /* Doesn't block, writes 1 to is_new_value if the value has changed since last BMP085_schedule_press_temp_update call */
int BMP085_read_press(sensor_sample_int32* pressure, int* is_new_value); /* Doesn't block, writes 1 to is_new_value if the value has changed since last BMP085_schedule_press_updatek call */

#endif
