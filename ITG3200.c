#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <wiringPiI2C.h>

#include "ITG3200.h"
#include "ITG3200_registers.h"
#include "logging.h"
#include "twos_complement.h"
#include "cpu_cycles.h"

int ITG3200_fd = -1;
unsigned char ITG3200_adr = 0x00;
ITG3200_FILTER_BANDWIDTH ITG3200_filter_bandwidth = -1;
ITG3200_SAMPLERATE_DIVIDER ITG3200_divider = -1;

/* private */
static int select_device() {
  return ioctl(ITG3200_fd, I2C_SLAVE, ITG3200_adr);
}

static int write_device(char* buf, int len) {
  return write(ITG3200_fd, buf, len);
}

static int read_device(char* buf, int len) {
  return read(ITG3200_fd, buf, len);
}

/* public */
int ITG3200_init(unsigned char adr, ITG3200_FILTER_BANDWIDTH bandwidth, ITG3200_SAMPLERATE_DIVIDER divider) {
  if (bandwidth > ITG3200_FILTER_BANDWIDTH_MAX) {
    LOG_ERROR("Invalid bandwidth value %d", bandwidth);
    return -1;
  }

  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -2;
  }
  ITG3200_fd = fd;
  ITG3200_adr = adr;
  ITG3200_filter_bandwidth = bandwidth;
  ITG3200_divider = divider;

  char buf[2];

  //test read "who am I" register
  buf[0] = ITG3200_WHOAMI;
  int rc = write_device(buf, 1);
  if (1 != rc) return -3;

  rc = read_device(buf, 1);
  if (1 != rc) return -4;

  if ((buf[0] & 0x7E) != ITG3200_ID_VAL) {
    LOG_DEBUG("ITG3200 'who am I' is 0x%x, expected 0x%x", buf[0], ITG3200_ID_VAL);
    return -5;
  }

  //sample rate divider
  buf[0] = ITG3200_SMPLRT_DIV;
  buf[1] = divider;
  rc = write_device(buf, 2);
  if (2 != rc) return -6;

  //scale and filter bandwidth
  buf[0] = ITG3200_DLPF_FS;
  buf[1] = FS_SEL_2000 | bandwidth; //FULL_SCALE in bits 3-4
  rc = write_device(buf, 2);
  if (2 != rc) return -7;

  //set X Gyro oscilator as the clock reference. Recomended by the sheet
  buf[0] = ITG3200_PWR_MGM;
  buf[1] = PWR_MGM_CLK_SEL_X;
  rc = write_device(buf, 2);
  if (2 != rc) return -8;

  LOG_DEBUG("ITG3200 on adr 0x%x init OK", adr);
  return 0;
}

int ITG3200_get_samplerate() {
  int internal_freq = ITG3200_filter_bandwidth == ITG3200_FILTER_BANDWIDTH_256 ? 8000 : 1000; // page 24
  return internal_freq / (ITG3200_divider + 1); // page 23
}

int ITG3200_read_temp(sensor_sample* temp) {
  int rc = select_device();
  if (rc) return rc;

  char buf[2];
  buf[0] = ITG3200_TEMP_OUT_H;

  rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 2);
  if (2 != rc) return -2;
  temp->ts = cpu_cycles();

  /* based on ITG3200 driver from Atmel. They use be16_to_cpu. */
  temp->val = from_bytes16(buf[1], buf[0]) - TEMP_OFFSET; /* MSB then LSB */
  temp->val /= TEMP_COUNTS_PER_DEG_C;
  temp->val += TEMP_REF_DEG;

  return 0;
}

int ITG3200_read_angular_vel(sensor_sample_3d *avels) {
  int rc = select_device();
  if (rc) return rc;

  char buf[6];
  buf[0] = ITG3200_GYRO_XOUT_H;

  rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 6);
  if (6 != rc) return -2;
  avels->ts = cpu_cycles();

  avels->data.x = ((double)from_bytes16(buf[1], buf[0])) / SCALE_LSB_PER_DPS;
  avels->data.y = ((double)from_bytes16(buf[3], buf[2])) / SCALE_LSB_PER_DPS;
  avels->data.z = ((double)from_bytes16(buf[5], buf[4])) / SCALE_LSB_PER_DPS;

  return 0;
}
