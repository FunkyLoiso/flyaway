#ifndef sensor_fusion_h__
#define sensor_fusion_h__

#include "vector.h"

typedef struct {
  vector_double_4d attitude;
  vector_double_3d avel;
  vector_double_3d lin_acc;
  double altitude;
  long long ts;
  double time_s;
} fused_sensor_data;

void fuse_sensor_data(sensor_data* data, fused_sensor_data* out_fused_data);

#endif // sensor_fusion_h__
