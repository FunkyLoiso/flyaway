#ifndef VECTOR_H_INCLUDED
#define VECTOR_H_INCLUDED

#include <stdint.h>

typedef struct {
  int x, y, z;
} vector_int_3d;

typedef struct {
  double x, y, z;
} vector_double_3d;


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
