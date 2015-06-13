#ifndef VECTOR_H_INCLUDED
#define VECTOR_H_INCLUDED

#include <stdint.h>

typedef struct {
  int x, y, z;
} vector_int_3d;

typedef struct {
  double x, y, z;
} vector_double_3d;

typedef struct {
  double q0, q1, q2, q3;
} vector_double_4d;

typedef struct {
  vector_double_3d cols[3];
} matrix_double_3x3;

static vector_double_3d cross_product_double_3d(vector_double_3d v1, vector_double_3d v2) {
  vector_double_3d ret = {v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
  return ret;
}

/* data with timestamp */
typedef struct {
  double val;
  long long ts;
} sensor_sample;

typedef struct {
	int32_t val;
	long long ts;
} sensor_sample_int32;

typedef struct {
  vector_double_3d data;
  long long ts;
} sensor_sample_3d;

typedef struct {
	vector_int_3d data;
	long long ts;
} sensor_sample_int_3d;

#endif
