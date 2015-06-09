#include <errno.h>
#include <string.h>
#include <wiringPi.h>

#include "sensors.h"
#include "logging.h"

//#define CALLIBRATE_HMC5883

static int calibrate_HMC5883() {
  printf("HMC5883 calibration procedure start\n");

  printf("(1 of 3) Please, hold sensor still and press Enter..");
  getchar();
  int rc = HMC5883_calibrate(0);
  if(rc) {
    printf("failed\n");
    return rc;
  }

  printf("OK\n(2 of 3) Now rotate the sensor 90 deg around Z axis, hold it still and press Enter..");
  getchar();
  rc = HMC5883_calibrate(1);
  if(rc) {
    printf("failed\n");
    return rc;
  }

  printf("OK\n(3 of 3) Now rotate the sensor 90 deg around X or Y axis, hold it still and press Enter..");
  rc = HMC5883_calibrate(2);
  if(rc) {
    printf("failed\n");
    return rc;
  }
  printf("OK\n HMC5883 calibration successfully compleated. Thank you for your patience.\n");

  return 0;
}

double pressure_to_altitude(int32_t sealevel_pressure, int32_t pressure) {
  return 44330 * (1.0 - pow(pressure /sealevel_pressure,0.1903));
}

int init_sensors()
{
  int rc = ADXL345_init(0x53, ADXL_RANGE_4g, ADXL_DATA_RATE_400); /* Our accelerometer detects on the "alternative address" 0x53 */
  if(rc) {
    LOG_ERROR("Error during ADXL345 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  rc = ITG3200_init(0x68, ITG3200_FILTER_BANDWIDTH_20, 3); /* 20 Hz lopass filter, 1000/(3+1) = 250Hz update */
  if(rc) {
    LOG_ERROR("Error during ITG3200 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  rc = HMC5883_init(0x1E, HMC5883_DATA_RATE_75HZ, HMC5883_GAIN_7_9GA); /* 75 Hz update, +-0.9gauss value range (Earth field is 0.31 - 0.58 gauss) */
  if(rc) {
    LOG_ERROR("Error during HMC5883 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }
  /* HMC5883 calibration can be done here */
#ifdef CALLIBRATE_HMC5883
  rc = callibrate_HMC5883();
  if(rc) {
    LOG_ERROR("Error during HMC5883 calibration. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  /* @TODO: save callibration data to file and load it later instead of calibrating every time */
#endif

  rc = BMP085_init(0x77, BMP085_2_INT_SAMPLES); /* 1,5 + 2*3 = 7,5 ms interval => 133 Hz update */
  if(rc) {
    LOG_ERROR("Error during BMP085 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  rc = BMP085_schedule_press_temp_update();
  if(rc) {
    LOG_ERROR("Error during HMC5883 pressure&temp update scheduling. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  return 0;
}


int read_sensors(SENSOR_DATA *data)
{
  /* @TODO: do result code checking */
  int rc = ADXL345_read(&data->acc_data);
  rc = ITG3200_read_temp(&data->itg3200_temp);
  rc = ITG3200_read_angular_vel(&data->avel_data);
  rc = HMC5883_read(&data->mag_data);

  int is_value_new;
  int pressure;
  BMP085_read_temp(&data->bmp085_temp, &is_value_new);
  BMP085_read_press(&pressure, &is_value_new);
  data->altitude = pressure_to_altitude(std_sealevel_pressure, pressure);

  /* schedule a BMP085 update */
  static const unsigned int BMP085_temp_update_interval_ms = 1000;
  static unsigned int BMP085_last_temp_update = 0;
  if( is_value_new && (millis() - BMP085_last_temp_update) < BMP085_temp_update_interval_ms ) {
    /* too early for temp update */
    rc = BMP085_schedule_press_update();
  }
  else {
    rc = BMP085_schedule_press_temp_update(); /* this blocks for 4.5 ms */
    BMP085_last_temp_update = millis();
  }

  return 0;
}
