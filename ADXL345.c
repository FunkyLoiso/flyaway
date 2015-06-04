/*
based on http://www.raspberrypi.org/forums/viewtopic.php?t=55834
*/

#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>

#include "ADXL345.h"
#include "ADXL345_registers.h"
#include "logging.h"

int adxl345_fd = -1;
unsigned char adxl345_adr = 0x00;
ADXL_RANGE_MODE adxl345_range_mode = -1;

/* private */
int select_device() {
  return ioctl(adxl345_fd, I2C_SLAVE, adxl345_adr);
}

int write_device(char* buf, int len) {
  return write(adxl345_fd, buf, len);
}

int read_device(char* buf, int len) {
  return read(adxl345_fd, buf, len);
}

/* public */
int ADXL345_init(unsigned char adr, ADXL_RANGE_MODE range_mode) {
  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -1;
  }
  adxl345_fd = fd;
  adxl345_adr = adr;
  adxl345_range_mode = range_mode;

  char buf[2];
//put into measure mode
  buf[0] = POWER_CTL;
  buf[1] = PCTL_MEASURE;

  int rc = write_device(buf, 2);
  if (2 != rc) return -2;

  int range_bits = 0;
  switch (range_mode)
  {
  case ADXL_RANGE_2g:
    range_bits = RANGE_PM_2g;
    break;
  case ADXL_RANGE_4g:
    range_bits = RANGE_PM_4g;
    break;
  case ADXL_RANGE_8g:
    range_bits = RANGE_PM_8g;
    break;
  case ADXL_RANGE_16g:
    range_bits = RANGE_PM_16g;
    break;
  default:
    LOG_ERROR("Invalid ADXL range mode: %d", range_mode);
    return -3;
    break;
  }

  //set resolution
  buf[0] = DATA_FORMAT;
  buf[1] = FULL_RES | range_bits;
  rc = write_device(buf, 2);
  if (2 != rc) return -4;

  LOG_DEBUG("ADXL345 on adr 0x%x init OK\n", adr);
  return 0;
}

int ADXL345_read(double* acc_x, double* acc_y, double* acc_z) {
  int rc = select_device();
  if (rc) return rc;

  char buf[6];
  buf[0] = DATAX0;

  rc = write_device(buf, 1);
  if (1 != rc) return -1;

  //in highres mode the output is 0.004g per digit
  rc = read_device(buf, 6);
  if (6 != rc) return -2;

  double k = 0.004; // others say(90.0 / 256.0)
  *acc_x = k * ((buf[1] << 8) | buf[0]);
  *acc_y = k * ((buf[3] << 8) | buf[2]);
  *acc_z = k * ((buf[5] << 8) | buf[4]);
}
