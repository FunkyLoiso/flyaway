#ifndef ITG3200_H_INCLUDED
#define ITG3200_H_INCLUDED

#include <vector.h>

typedef enum {
  ITG3200_FILTER_BANDWIDTH_MIN = 0,
  ITG3200_FILTER_BANDWIDTH_256 = 0,
  ITG3200_FILTER_BANDWIDTH_188 = 1,
  ITG3200_FILTER_BANDWIDTH_98 = 2,
  ITG3200_FILTER_BANDWIDTH_42 = 3,
  ITG3200_FILTER_BANDWIDTH_20 = 4,
  ITG3200_FILTER_BANDWIDTH_10 = 5,
  ITG3200_FILTER_BANDWIDTH_5 = 6,
  ITG3200_FILTER_BANDWIDTH_MAX = ITG3200_FILTER_BANDWIDTH_5
} ITG3200_FILTER_BANDWIDTH;

typedef unsigned char ITG3200_SAMPLERATE_DIVIDER;

int ITG3200_init(unsigned char adr, ITG3200_FILTER_BANDWIDTH bandwidth, ITG3200_SAMPLERATE_DIVIDER divider);
int ITG3200_get_samplerate();
int ITG3200_read(sensor_sample_3d* avels, sensor_sample* temp);
void ITG3200_set_callibration_curves(vector_double_3d offsets, vector_double_3d slopes);

#endif
