#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>

#include "ITG3200.h"
#include "ITG3200_registers.h"
#include "logging.h"

int ITG3200_fd = -1;
unsigned char ITG3200_adr = 0x00;
ITG3200_FILTER_BANDWIDTH ITG3200_bandwidth = -1;
ITG3200_SAMPLERATE_DIVIDER ITG3200_divider = -1;

/* private */
int select_device() {
  return ioctl(ITG3200_fd, I2C_SLAVE, ITG3200_adr);
}

int write_device(char* buf, int len) {
  return write(ITG3200_fd, buf, len);
}

int read_device(char* buf, int len) {
  return read(ITG3200_fd, buf, len);
}

/* public */
int ITG3200_init(unsigned char adr, ITG3200_FILTER_BANDWIDTH bandwidth, ITG3200_SAMPLERATE_DIVIDER divider) {
  if (bandwidth < ITG3200_FILTER_BANDWIDTH_MIN || bandwidth > ITG3200_FILTER_BANDWIDTH_MAX) {
    LOG_ERROR("Invalid bandwidth value %d", bandwidth);
    return -1;
  }

  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -2;
  }
  ITG3200_fd = fd;
  ITG3200_adr = adr;
  ITG3200_bandwidth = bandwidth;
  ITG3200_divider = divider;

  char buf[2];

  //test read "who am I" register
  buf[0] = WHO_AM_I;
  int rc = write_device(buf, 2);
  if (2 != rc) return -3;

  int rc = read_device(buf, 1);
  if (1 != rc) return -4;

  if (buf[0] & 0x7E != 0x68) {
    LOG_DEBUG("ITG3200 'who am I' is 0x%x, expected 0x%x", buf[0], 0x68);
    return -5;
  }

  //sample rate divider
  buf[0] = SMPL_RATE_DIVIDER;
  buf[1] = divider;
  int rc = write_device(buf, 2);
  if (2 != rc) return -6;

  //scale and filter bandwidth
  buf[0] = DIG_LOWPASS_FLTR;
  buf[1] = (3 << 3) + bandwidth;
  int rc = write_device(buf, 2);
  if (2 != rc) return -7;

  //set X Gyro oscilator as the clock reference. Recomended by the sheet
  buf[0] = POWER_MGMNT;
  buf[1] = 1;
  int rc = write_device(buf, 2);
  if (2 != rc) return -8;

  LOG_DEBUG("ITG3200 on adr 0x%x init OK\n", adr);
  return 0;
}

int ADXL345_read(double* acc_x, double* acc_y, double* acc_z) {
  int rc = select_device();
  if (rc) return rc;

  char buf[6];
  buf[0] = DATAX0;

  rc = write_device(buf, 2); // Not 1?
  if (2 != rc) return -1;

  //in highres mode the output is 0.004g per digit
  rc = read_device(buf, 6);
  if (6 != rc) return -2;

  double k = 0.004;
  *acc_x = k * ((buf[1] << 8) | buf[0]);
  *acc_y = k * ((buf[3] << 8) | buf[2]);
  *acc_z = k * ((buf[5] << 8) | buf[4]);
}
