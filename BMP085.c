#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h> // for memcpy
#include <limits.h> // for UINT_MAX

#include <wiringPiI2C.h>
#include <wiringPi.h> // for delay

#include "BMP085.h"
#include "BMP085_registers.h"
#include "logging.h"
#include "twos_complement.h"
#include "cpu_cycles.h"

int BMP085_fd = -1;
unsigned char BMP085_adr = 0x00;
BMP085_OVERSAMPLING_SETTING BMP085_oss = -1;
struct {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;

  int16_t b1;
  int16_t b2;

  int16_t mb;
  int16_t mc;
  int16_t md;
} cal = {0};

int32_t uncompensated_temperature = -1;
int32_t real_temperature = -1;
long long temperature_ts = 0;
int new_temperature_flag = 0;

int32_t uncompensated_pressure = -1;
int32_t real_pressure = -1;
long long pressure_ts = 0;
unsigned int last_pressure_schedule_mcs = 0;
int check_pressure_flag = 0;

/* private */
static int select_device() {
  return ioctl(BMP085_fd, I2C_SLAVE, BMP085_adr);
}

static int write_device(char* buf, int len) {
  return write(BMP085_fd, buf, len);
}

static int read_device(char* buf, int len) {
  return read(BMP085_fd, buf, len);
}

/* public */
int BMP085_init(unsigned char adr, BMP085_OVERSAMPLING_SETTING oss) {
  if (oss < BMP085_1_INT_SAMPLE || oss > BMP085_8_INT_SAMPLES) {
    LOG_ERROR("Invalid BMP085_OVERSAMPLING_SETTING value: %d", oss);
    return -1;
  }

  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -10;
  }

  BMP085_fd = fd;
  BMP085_adr = adr;
  BMP085_oss = oss;

  char buf[BMP_EEPROM_SIZE_BYTES] = {0};

  /* check id */
  buf[0] = BMP_CHIP_ID;
  int rc = write_device(buf, 1);
  if (1 != rc) return -11;

  rc = read_device(buf, 1);
  if(1 != rc) return -20;

  if (buf[0] != BMP085_ID_VAL) {
    LOG_ERROR("BMP085 id check failed. Expected 0x%x, got 0x%x", BMP085_ID_VAL, buf[0]);
    return -30;
  }

  /* get calibration parameters */
  buf[0] = BMP_EEPROM_ADDR;
  rc = write_device(buf, 1);
  if (1 != rc) return -40;
  rc = read_device(buf, BMP_EEPROM_SIZE_BYTES);
  if (BMP_EEPROM_SIZE_BYTES != rc) return -50;

  cal.ac1 = from_bytes16(buf[ 1], buf[ 0]); /* MSB then LSB */
  cal.ac2 = from_bytes16(buf[ 3], buf[ 2]);
  cal.ac3 = from_bytes16(buf[ 5], buf[ 4]);
  cal.ac4 = from_bytes16(buf[ 7], buf[ 6]);
  cal.ac5 = from_bytes16(buf[ 9], buf[ 8]);
  cal.ac6 = from_bytes16(buf[11], buf[10]);

  cal.b1  = from_bytes16(buf[13], buf[12]);
  cal.b2  = from_bytes16(buf[15], buf[14]);

  cal.mb  = from_bytes16(buf[17], buf[16]);
  cal.mc  = from_bytes16(buf[19], buf[18]);
  cal.md  = from_bytes16(buf[21], buf[20]);

  LOG_DEBUG("BMP085 on adr 0x%x init OK", adr);
  return 0;
}

int BMP085_schedule_press_update() {
  int rc = select_device();
  if (rc) return rc;

  char buf[2];
  buf[0] = BMP_CONTROL;
  buf[1] = BMP_PRESS_READ + (BMP085_oss << 6);
  rc = write_device(buf, 2);
  if (2 != rc) return -10;
  last_pressure_schedule_mcs = micros();
  check_pressure_flag = 1;
  return 0;
}

int BMP085_schedule_press_temp_update() { /* blocks for 4.5 ms  */
  int rc = select_device();
  if (rc) return rc;

  char buf[2];
  buf[0] = BMP_CONTROL;
  buf[1] = BMP_TEMP_READ;
  rc = write_device(buf, 2);
  if (2 != rc) return -10;

  delayMicroseconds(4500); /*wait for temp result*/

  buf[0] = BMP_DATA_MSB;
  rc = write_device(buf, 1);
  if (1 != rc) return -20;
  rc = read_device(buf, 2);
  if (2 != rc) return -30;

  uncompensated_temperature = from_bytes16(buf[1], buf[0]); /* MSB then LSB */
  temperature_ts = cpu_cycles();
  new_temperature_flag = 1;

  /*now schedule pressure update and return*/
  rc = BMP085_schedule_press_update();
  if (rc) return 10 * rc;
  return 0;
}

void BMP085_read_temp(sensor_sample* temperature, int* is_new_value)
{
  if (new_temperature_flag) {
    int32_t const x1 = ((uncompensated_temperature - cal.ac6) * cal.ac5) >> 15;
    int32_t const x2 = ((int32_t)cal.mc << 11) / (x1 + cal.md);

    real_temperature = ((double)( ((x1 + x2) + 8) >> 4 )) / 10.0; /* temperature output is in 0.1 of deg c */

    new_temperature_flag = 0;
    *is_new_value = 1;
  }
  else {
    *is_new_value = 0;
  }

  temperature->val = real_temperature;
  temperature->ts = temperature_ts;

}
int BMP085_read_press(sensor_sample_int32* pressure, int* is_new_value)
{
  unsigned int delay = 1500 + (3000 << BMP085_oss); /* max delay is 1.5 + 3*<num_of_samples> ms */
  if ( check_pressure_flag && ((micros() - last_pressure_schedule_mcs) >= delay) ) {
    /* new value ready */
    char buf[3];
    buf[0] = BMP_DATA_MSB;
    int rc = write_device(buf, 1);
    if (1 != rc) return -10;
    rc = read_device(buf, 3);
    if (3 != rc) return -20;

    uncompensated_pressure = from_bytes24(buf[2], buf[1], buf[0]); /* MSB then LSB then XLSB */
	pressure_ts = cpu_cycles();
    check_pressure_flag = 0;

    /* The compensated pressure in pascal (Pa) units. */
    int32_t const UT = uncompensated_temperature;

    int32_t const x1 = ((UT - cal.ac6) * cal.ac5) >> 15;
    int32_t const x2 = ((int32_t)cal.mc << 11) / (x1 + cal.md);

    int32_t const B5 = (x1 + x2);
    int32_t const B6 = B5 - 4000L;

    int32_t X1 = (cal.b2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = (cal.ac2 * B6) >> 11;
    int32_t X3 = X1 + X2;

    int32_t const B3 = ((((int32_t)cal.ac1 * 4 + X3) << BMP085_oss) + 2) >> 2;

    X1 = (cal.ac3 * B6) >> 13;
    X2 = (cal.b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    uint32_t const B4 = (cal.ac4 * (uint32_t)(X3 + 32768L)) >> 15;
    uint32_t const B7 = (uncompensated_pressure - B3) * (50000L >> BMP085_oss);

    int32_t P = (B7 < 0x80000000UL) ? ((B7 << 1) / B4) : ((B7 / B4) << 1);

    X1 = (P >> 8);
    X1 = (X1 * X1 * 3038L) >> 16;
    X2 = (-7357L * P) >> 16;

    P += ((X1 + X2 + 3791L) >> 4);

    real_pressure = P;
    *is_new_value = 1;
  }
  else {
    /* new value not yet ready */
    *is_new_value = 0;
  }
  pressure->val = real_pressure;
  pressure->ts = pressure_ts;


  return 0;
}
