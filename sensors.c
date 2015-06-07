#include <errno.h>
#include <string.h>

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
}


int init_sensors()
{
  int rc = ADXL345_init(0x1D, ADXL_RANGE_4g);
  if(rc) {
    LOG_ERROR("Error during ADXL345 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  rc = ITG3200_init(0x34, ITG3200_FILTER_BANDWIDTH_20, 3); /* 20 Hz lopass filter, 1000/(3+1) = 250Hz update */
  if(rc) {
    LOG_ERROR("Error during ITG3200 initialization. Internal code: %d, errno: %d\nstrerror: \"%s\"", rc, errno, strerror(errno));
    return rc;
  }

  rc = HMC5883_init(0x1E, HMC5883_DATA_RATE_75HZ, HMC5883_GAIN_0_9GA); /* 75 Hz update, +-0.9gauss value range (Earth field is 0.31 - 0.58 gauss) */
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
}


int read_sensors(SENSOR_DATA *data)
{

}