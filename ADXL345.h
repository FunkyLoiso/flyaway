#ifndef ADXL345_H_INCLUDED
#define ADXL345_H_INCLUDED

typedef enum {
  ADXL_RANGE_2g,
  ADXL_RANGE_4g,
  ADXL_RANGE_8g,
  ADXL_RANGE_16g
} ADXL_RANGE_MODE;

int ADXL345_init(unsigned char adr, ADXL_RANGE_MODE mode);
int ADXL345_read(double* acc_x, double* acc_y, double* acc_z);

#endif
