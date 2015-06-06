#ifndef HMC5883_H_INCLUDED
#define HMC5883_H_INCLUDED

#include "vector.h"

const int HMC5883_OVERFLOW = -1000; // returned by HMC5883_read in case of axis overflow

typedef enum {
  HMC5883_DATA_RATE_0_75HZ = (0x00),
  HMC5883_DATA_RATE_1_5HZ = (0x04),
  HMC5883_DATA_RATE_3HZ = (0x08),
  HMC5883_DATA_RATE_7_5HZ = (0x0c),
  HMC5883_DATA_RATE_15HZ = (0x10),
  HMC5883_DATA_RATE_30HZ = (0x14),
  HMC5883_DATA_RATE_75HZ = (0x18)
} HMC5883_DATA_RATE;

typedef enum {
HMC5883_GAIN_0_9GA = (0x00),
HMC5883_GAIN_1_2GA = (0x20),
HMC5883_GAIN_1_9GA = (0x40),
HMC5883_GAIN_2_5GA = (0x60),
HMC5883_GAIN_4_0GA = (0x80),
HMC5883_GAIN_4_6GA = (0xa0),
HMC5883_GAIN_5_5GA = (0xc0),
HMC5883_GAIN_7_9GA = (0xe0)
} HMC5883_GAIN;

typedef struct {
  vector_int_3d offset;
  vector_double_3d sensitivity;
} HMC5883_CALIBRATION_DATA;

int HMC5883_init(unsigned char adr, HMC5883_DATA_RATE data_rate, HMC5883_GAIN gain);
int HMC5883_read(vector_double_3d* mag);

int HMC5883_calibrate();
void HMC5883_get_calibration_data(HMC5883_CALIBRATION_DATA* data);
void HMC5883_set_calibration_data(HMC5883_CALIBRATION_DATA* data);

#endif
