#ifndef ADXL345_H_INCLUDED
#define ADXL345_H_INCLUDED

#include "vector.h"

typedef enum {
  ADXL_RANGE_2g = 0,
  ADXL_RANGE_4g = 1,
  ADXL_RANGE_8g = 2,
  ADXL_RANGE_16g = 3
} ADXL_RANGE_MODE;

typedef enum {
  ADXL_DATA_RATE_0_1 = 0,
  ADXL_DATA_RATE_0_2 = 1,
  ADXL_DATA_RATE_0_39 = 3,
  ADXL_DATA_RATE_0_78 = 4,
  ADXL_DATA_RATE_1_56 = 5,
  ADXL_DATA_RATE_3_13 = 6,
  ADXL_DATA_RATE_6_25 = 7,
  ADXL_DATA_RATE_12_5 = 8,
  ADXL_DATA_RATE_25 = 9,
  ADXL_DATA_RATE_50 = 10,
  ADXL_DATA_RATE_100 = 11,
  ADXL_DATA_RATE_200 = 12,
  ADXL_DATA_RATE_400 = 13,
  ADXL_DATA_RATE_800 = 14,
  ADXL_DATA_RATE_1600 = 15,
  ADXL_DATA_RATE_3200 = 16
} ADXL_DATA_RATE;

int ADXL345_init(unsigned char adr, ADXL_RANGE_MODE mode, ADXL_DATA_RATE data_rate);
int ADXL345_set_zero_level();
int ADXL345_read(sensor_sample_3d* accs);

#endif
