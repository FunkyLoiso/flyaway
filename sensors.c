#include <errno.h>
#include <string.h>
#include <wiringPi.h>

#include "sensors.h"
#include "logging.h"

//#define CALLIBRATE_HMC5883

static double zero_altitude_abs; /* altitude which should be considered zero */

#ifdef CALLIBRATE_HMC5883
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
#endif

double pressure_to_altitude(double sealevel_pressure, double pressure) {
  return 44330 * (1.0 - pow(pressure /sealevel_pressure,0.1903));
}

int init_sensors()
{
  int rc = ADXL345_init(0x53, ADXL_RANGE_16g, ADXL_DATA_RATE_400); /* Our accelerometer detects on the "alternative address" 0x53. 400 Hz update */
  if(rc) {
    LOG_ERROR_ERRNO("Error during ADXL345 initialization. Internal code: %d,", rc);
    return rc;
  }

//  rc = ADXL345_set_zero_level();
//  if (rc) {
//    LOG_ERROR("Error during ADXL345 zero level setting. Internal code: %d,", rc);
//    return rc;
//  }

  rc = ITG3200_init(0x68, ITG3200_FILTER_BANDWIDTH_20, 3); /* 20 Hz lowpass filter, 1000/(3+1) = 250Hz update */
  if(rc) {
    LOG_ERROR("Error during ITG3200 initialization. Internal code: %d,", rc);
    return rc;
  }

  rc = HMC5883_init(0x1E, HMC5883_DATA_RATE_75HZ, HMC5883_GAIN_0_9GA); /* 75 Hz update, +-0.9gauss value range (Earth field is 0.31 - 0.58 gauss) */
  if(rc) {
    LOG_ERROR("Error during HMC5883 initialization. Internal code: %d,", rc);
    return rc;
  }
  /* HMC5883 calibration can be done here */
#ifdef CALLIBRATE_HMC5883
  rc = callibrate_HMC5883();
  if(rc) {
    LOG_ERROR("Error during HMC5883 calibration. Internal code: %d,", rc);
    return rc;
  }

  /* @TODO: save callibration data to file and load it later instead of calibrating every time */
#endif

  rc = BMP085_init(0x77, BMP085_2_INT_SAMPLES); /* 1,5 + 2*3 = 7,5 ms interval => 133 Hz update */
  if(rc) {
    LOG_ERROR("Error during BMP085 initialization. Internal code: %d,", rc);
    return rc;
  }

  rc = BMP085_schedule_press_temp_update();
  if(rc) {
    LOG_ERROR("Error during HMC5883 pressure&temp update scheduling. Internal code: %d,", rc);
    return rc;
  }

  return 0;
}


int read_sensors(sensor_data *data)
{
  int rc = 0;

  /* schedule a BMP085 update */
  static const unsigned int BMP085_temp_update_interval_ms = 1000;
  static unsigned int BMP085_last_temp_update = 0;
  if( (millis() - BMP085_last_temp_update) < BMP085_temp_update_interval_ms ) {
    /* too early for temp update */
    rc = BMP085_schedule_press_update();
    if(rc) {
      LOG_ERROR_ERRNO("Error scheduling BMP085 pressure update, code %d", rc);
      return rc;
    }
  }
  else {
    rc = BMP085_schedule_press_temp_update(); /* this blocks for 4.5 ms */
    if(rc) {
      LOG_ERROR_ERRNO("Error scheduling BMP085 temperature update, code %d", rc);
      return rc;
    }
    BMP085_last_temp_update = millis();
  }

  rc = ADXL345_read(&data->acc_data);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading ADXL345 data, code %d", rc);
//    return rc;
  }
  rc = ITG3200_read_temp(&data->itg3200_temp);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading ITG3200 temperature data, code %d", rc);
//    return rc;
  }
  rc = ITG3200_read_angular_vel(&data->avel_data);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading ITG3200 angular velocity data, code %d", rc);
    return rc;
  }
  rc = HMC5883_read(&data->mag_data);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading HMC5883 magnetic data, code %d", rc);
    return rc;
  }

  int is_temp_new, is_press_new;
  sensor_sample_int32 pressure;
  BMP085_read_temp(&data->bmp085_temp, &is_temp_new);
  rc = BMP085_read_press(&pressure, &is_press_new);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading BMP085 pressure, code %d", rc);
    return rc;
  }
  data->altitude.val = pressure_to_altitude(std_sealevel_pressure, pressure.val);
  data->altitude.ts = pressure.ts;

  return 0;
}

int zero_altitude()
{
  int is_value_new;
  sensor_sample_int32 pressure;
  int rc = BMP085_read_press(&pressure, &is_value_new);
  if(rc) {
    LOG_ERROR_ERRNO("Error reading BMP085 pressure to zero altitude, code %d", rc);
    return rc;
  }

  zero_altitude_abs = pressure_to_altitude(std_sealevel_pressure, pressure.val);
  return 0;
}

/* calibration */
int itg3200_callibration_curve(unsigned int interval_ms, unsigned int time_ms, FILE* out_file) {
  int rc = 0;
  sensor_sample temp = {};
  sensor_sample_3d avels = {};

  rc = fprintf(out_file, "temp, avelx, avely, avelz\n");
  if(rc < 0) return rc;

  unsigned int stop_time_ms = millis() + time_ms;
  do {
    rc = ITG3200_read_temp(&temp);
    if(rc) return rc;
    rc = ITG3200_read_angular_vel(&avels);
    if(rc) return rc;

    rc = fprintf(out_file, "%f, %f, %f, %f\n", temp.val, avels.data.x, avels.data.y, avels.data.z);
    if(rc < 0) return rc;

    delay(interval_ms);
    printf("%f, %f, %f, %f\n", temp.val, avels.data.x, avels.data.y, avels.data.z);
    fflush(stdout);
  } while (millis() < stop_time_ms);

  return 0;
}
