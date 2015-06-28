/*
based on http://www.raspberrypi.org/forums/viewtopic.php?t=55834
*/

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>

#include <wiringPiI2C.h>
#include <wiringPi.h>

#include "ADXL345.h"
#include "ADXL345_registers.h"
#include "logging.h"
#include "twos_complement.h"
#include "cpu_cycles.h"

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
int ADXL345_init(unsigned char adr, ADXL_RANGE_MODE range_mode, ADXL_DATA_RATE data_rate) {
  if(range_mode > ADXL_RANGE_16g) {
    LOG_ERROR("Wrong range mode for ADXL345: %d", range_mode);
    return -1;
  }

  if(data_rate > ADXL_DATA_RATE_3200) {
    LOG_ERROR("Wrong data rate value for ADXL345: %d", data_rate);
    return -5;
  }
  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -10;
  }
  adxl345_fd = fd;
  adxl345_adr = adr;
  adxl345_range_mode = range_mode;

  char buf[2];
  /* check device id */
  buf[0] = DEVID;
  int rc = write_device(buf, 1);
  if(1 != rc) return -11;
  rc = read_device(buf, 1);
  if(1 != rc) return -12;
  if(((uint8_t)buf[0]) != ID_ADXL345) {
    LOG_ERROR("ADXL345 id expected 0x%x, got 0x%x", ID_ADXL345, buf[0]);
    return -13;
  }

  /*set data rate*/
  buf[0] = BW_RATE;
  buf[1] = data_rate;
  rc = write_device(buf, 2);
  if(2 != rc) return -13;

  /* put into measure mode*/
  buf[0] = POWER_CTL;
  buf[1] = PCTL_MEASURE;

  rc = write_device(buf, 2);
  if (2 != rc) return -2;

  //set resolution
  buf[0] = DATA_FORMAT;
  buf[1] = FULL_RES | range_mode;
  rc = write_device(buf, 2);
  if (2 != rc) return -4;

  LOG_DEBUG("ADXL345 on adr 0x%x init OK", adr);
  return 0;
}

int ADXL345_read(sensor_sample_3d* accs) {
  int rc = select_device();
  if (rc) return rc;

  char buf[6];
  buf[0] = DATAX0;
  buf[2] = DATAY0;
  buf[4] = DATAZ0;

  rc = write_device(buf, 1);
  if (1 != rc) return -10;

//  rc = read_device(buf, 6);
//  if (6 != rc) return -20;

  rc = read_device(buf, 2);
  if (2 != rc) return -20;

  rc = write_device(buf+2, 1);
  if (1 != rc) return -10;
  rc = read_device(buf+2, 2);
  if (2 != rc) return -21;

  rc = write_device(buf+4, 1);
  if (1 != rc) return -10;
  rc = read_device(buf+4, 2);
  if (2 != rc) return -22;
  accs->ts = cpu_cycles();

  static const double k = 4.0/1024.0; /* in highres mode the output is 0.00390625g per digit */
  const uint8_t sig_bits = 8 + 2 + adxl345_range_mode; /* number of significant bits depends on range mode */

  accs->data.x = k * from_bytes16_limited(buf[0], buf[1], sig_bits); /* LSB then MSB */
  accs->data.y = k * from_bytes16_limited(buf[2], buf[3], sig_bits);
  accs->data.z = k * from_bytes16_limited(buf[4], buf[5], sig_bits);

  return 0;
}

int ADXL345_set_zero_level() {
  int rc = select_device();
  if(rc) return rc;

  sensor_sample_3d cur_data;
  vector_double_3d avg_data;

  static const int count = 20;
  int i;
  for(i = 0; i < count; ++i) {
    rc = ADXL345_read(&cur_data);
    if(rc) return rc;
    avg_data.x += cur_data.data.x;
    avg_data.y += cur_data.data.y;
    avg_data.z += cur_data.data.z;

    delay(10);
  }

  avg_data.x /= (double) count;
  avg_data.y /= (double) count;
  avg_data.z /= (double) count;

  char buf[4];
  buf[0] = OFSX;
  buf[1] =  -rint( avg_data.x / 0.0156 );
  buf[2] =  -rint( avg_data.y / 0.0156 );
  buf[3] =  -rint( avg_data.z / 0.0156 );

  rc = write_device(buf, 4);
  if(4 != rc) return -100;

  LOG_DEBUG("ADXL345 zero level set. Errors: %f %f %f, offsets written: %f %f %f",
            -avg_data.x, -avg_data.y, -avg_data.z,
            0.0156 * buf[1], 0.0156 * buf[2], 0.0156 * buf[3]);
  return 0;
}

