#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>
#include <wiringPi.h> // for delay

#include "HMC5883.h"
#include "HMC5883_registers.h"
#include "logging.h"

int HMC5883_fd = -1;
unsigned char HMC5883_adr = 0x00;
HMC5883_DATA_RATE HMC5883_data_rate = -1;
HMC5883_GAIN HMC5883_gain = -1;
HMC5883_CALLIBRATION_DATA HMC5883_callibration_data = {0};

/* private */
int select_device() {
  //maybe we need to do HMC5883_adr << 1 for write and (HMC5883_adr << 1) | 1 for read?
  return ioctl(HMC5883_fd, I2C_SLAVE, HMC5883_adr);
}

int write_device(char* buf, int len) {
  return write(HMC5883_fd, buf, len);
}

int read_device(char* buf, int len) {
  return read(HMC5883_fd, buf, len);
}

int gain_to_scale(HMC5883_GAIN gain) {
  switch (gain) {
  case GAIN_0_9GA: return SCALE_0_9GA;
  case GAIN_1_2GA: return SCALE_1_2GA;
  case GAIN_1_9GA: return SCALE_1_9GA;
  case GAIN_2_5GA: return SCALE_2_5GA;
  case GAIN_4_0GA: return SCALE_4_0GA;
  case GAIN_4_6GA: return SCALE_4_6GA;
  case GAIN_5_5GA: return SCALE_5_5GA;
  case GAIN_7_9GA: return SCALE_7_9GA;
  default: return -1;
  }
}

int read_mag_raw(vector_int_3d* raw_data) {
  char buf[6];
  buf[0] = HMC5883_MAG_X_HI;

  int rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 6);
  if (6 != rc) return -2;

  // sensor regizer order is X Z Y
  raw_data->x = (buf[1] << 8) | buf[0];
  raw_data->z = (buf[3] << 8) | buf[2];
  raw_data->y = (buf[5] << 8) | buf[4];

  if (raw_data->x == DATA_OUTPUT_OVERFLOW |
    raw_data->y == DATA_OUTPUT_OVERFLOW |
    raw_data->z == DATA_OUTPUT_OVERFLOW) {
    return HMC5883_OVERFLOW;
  }

  return 0;
}

int selftest(vector_int_3d *data)
{
  static const int SELF_TEST_DELAY_MS = 250;
  //int count;
  //uint8_t meas_mode;          /* test measurement mode */
  //vector_int_3d data;
  //int status = 0;
  int return_code = 0;

  struct {
    uint8_t config_reg_a;
    uint8_t config_reg_b;
    uint8_t mode_reg;
  }
  reg_set;

  int rc = select_device();
  if (rc) return rc;

  char buf[2];

  /* Save register values */
  buf[0] = HMC5883_CONFIG_REG_A;
  rc = write_device(buf, 1);
  if (1 != rc) return -10;
  rc = read_device((char *)&reg_set, sizeof(reg_set));
  if (sizeof(reg_set) != rc) return -11;

  /* Set range (sensitivity) */
  buf[0] = HMC5883_CONFIG_REG_B;
  buf[1] = HMC5883_TEST_GAIN;
  rc = write_device(buf, 2);
  if (2 != rc) {
    return_code = -20;
    goto exit;
  }

  /* Set test mode */
  buf[0] = HMC5883_CONFIG_REG_A;
  buf[1] = DATA_RATE_15HZ | MEAS_MODE_POS;
  rc = write_device(buf, 2);
  if (2 != rc) {
    return_code = -30;
    goto exit;
  }

  delay(SELF_TEST_DELAY_MS);

  /* Perform test measurement & check results */
  buf[0] = HMC5883_MODE_REG;
  buf[1] = MODE_SINGLE; /* put into single-measurement mode to start self-test */
  rc = write_device(buf, 2);
  if (2 != rc) {
    return_code = -40;
    goto exit;
  }

  delay(SELF_TEST_DELAY_MS);

  vector_int_3d raw_data;
  int rc = read_mag_raw(&raw_data);
  if (rc) {
    return_code = rc;
    goto exit;
  }

  if (arg != NULL) {
    ((sensor_data_t *)arg)->scaled = false; /* only raw values */
    ((sensor_data_t *)arg)->timestamp = 0;  /* no timestamp */

    ((sensor_data_t *)arg)->axis.x = (int32_t)data.x; /* copy values */
    ((sensor_data_t *)arg)->axis.y = (int32_t)data.y;
    ((sensor_data_t *)arg)->axis.z = (int32_t)data.z;
  }

  /* Check range of readings */
  if ((HMC5883L_TEST_X_MIN > data.x) ||
    (data.x > HMC5883L_TEST_X_MAX) ||
    (HMC5883L_TEST_Y_MIN > data.y) ||
    (data.y > HMC5883L_TEST_Y_MAX) ||
    (HMC5883L_TEST_Z_MIN > data.z) ||
    (data.z > HMC5883L_TEST_Z_MAX)) {
    *test_code = SENSOR_TEST_ERR_RANGE;
    status = false; /* value out of range */
  }

exit:
  /* Restore registers */
  count = sensor_bus_write(hal, HMC5883L_CONFIG_REG_A,
    (uint8_t *)&reg_set, sizeof(reg_set));

  if (count != sizeof(reg_set)) {
    *test_code = SENSOR_TEST_ERR_WRITE;
    status = false;
  }

  return status;
}

/* public */
int HMC5883_init(unsigned char adr, HMC5883_DATA_RATE data_rate, HMC5883_GAIN gain) {
  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -1;
  }

  HMC5883_fd = fd;
  HMC5883_adr = adr;
  HMC5883_data_rate = data_rate;
  HMC5883_gain = gain;


  char buf[3];

  //check id
  buf[0] << HMC5883_ID_REG_A;
  int rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 3);
  if(3 != rc) return -2;

  if (buf[0] != ID_A_DEFAULT |
    buf[1] != ID_B_DEFAULT |
    buf[2] != ID_C_DEFAULT) {
    LOG_ERROR("HMC5883 id check failed. Expected 0x%x%x%x, got 0x%x%x%x", ID_A_DEFAULT, ID_B_DEFAULT, ID_C_DEFAULT, buf[0], buf[1], buf[2]);
    return -3;
  }

  //set data output rate and measure mode
  buf[0] = HMC5883_CONFIG_REG_A;
  buf[1] = data_rate | MEAS_MODE_NORM;

  int rc = write_device(buf, 2);
  if (2 != rc) return -4;

  //set gain
  buf[0] = HMC5883_CONFIG_REG_B;
  buf[1] = gain;

  int rc = write_device(buf, 2);
  if (2 != rc) return -5;

  //set continuous-measurement mode
  buf[0] = HMC5883_MODE_REG;
  buf[1] = MODE_CONTIN;
  rc = write_device(buf, 2);
  if (2 != rc) return -6;

  LOG_DEBUG("HMC5883 on adr 0x%x init OK\n", adr);
  return 0;
}

int HMC5883_read(vector_double_3d* mag) {
  int rc = select_device();
  if (rc) return rc;

  vector_int_3d raw_data;
  rc = read_mag_raw(&raw_data);
  if (rc) return rc;

  double counts_per_Gauss = gain_to_scale(HMC5883_gain);
  mag->x = ((double)raw_data.x) / counts_per_Gauss;
  mag->y = ((double)raw_data.y) / counts_per_Gauss;
  mag->z = ((double)raw_data.z) / counts_per_Gauss;
}

int HMC5883_callibare(int step) {
  static vector_int_3d step_data[3];  /* sensor readings during calibration */
  vector_int_3d dummy_data;           /* data from first sensor read (ignored) */
  vector_int_3d read_back;            /* data read back from nvram to validate */
  sensor_data_t test_data;            /* readings during self test */
  int test_code;                      /* self-test code & result */

  /* Validate the supported calibration types and step number. */
  if ((step < 1) || (step > 3)) {
    return -1;
  }

  /* During first pass, use self-test to determine sensitivity scaling */
  if (step == 1) {
    /* Run internal self test with known bias field */
    test_code = SENSOR_TEST_BIAS_POS;

    if ((hmc5883l_selftest(sensor, &test_code,
      &test_data) == false) ||
      (test_code != SENSOR_TEST_ERR_NONE)) {
      return false;
    }

    /* Calculate & store sensitivity adjustment values */
    cal_data.sensitivity.x
      = ((scalar_t)HMC5883L_TEST_X_NORM / test_data.axis.x);
    cal_data.sensitivity.z
      = ((scalar_t)HMC5883L_TEST_Z_NORM / test_data.axis.z);
    cal_data.sensitivity.y
      = ((scalar_t)HMC5883L_TEST_Y_NORM / test_data.axis.y);

    nvram_write(CAL_SENSITIVITY_ADDR, &cal_data.sensitivity,
      sizeof(cal_data.sensitivity));

    /* Read back data and confirm it was written correctly */
    nvram_read(CAL_SENSITIVITY_ADDR, &read_back, sizeof(vector3_t));

    if (memcmp(&cal_data.sensitivity, &read_back,
      sizeof(vector3_t))) {
      sensor->err = SENSOR_ERR_IO;
      return false;
    }
  }

  /* Read sensor data and test for data overflow.
  *   Note: Sensor must be read twice - the first reading may
  *         contain stale data from previous orientation.
  */
  if (hmc5883l_get_data(hal, &dummy_data) != true) {
    return false;
  }

  delay_ms(READ_DELAY_MSEC);

  if (hmc5883l_get_data(hal, &(step_data[step - 1])) != true) {
    return false;
  }

  /* Apply sensitivity scaling factors */
  hmc5883l_apply_sensitivity(&(step_data[step - 1]));

  switch (step) {
    /* There's nothing more to do on the first two passes. */
  case 1:
  case 2:
    break;

    /* Calculate & store the offsets on the final pass. */
  case 3:
    cal_data.offsets.x = (step_data[0].x + step_data[1].x) / 2;
    cal_data.offsets.y = (step_data[0].y + step_data[1].y) / 2;
    cal_data.offsets.z = (step_data[1].z + step_data[2].z) / 2;

    nvram_write(CAL_OFFSETS_ADDR, &cal_data.offsets,
      sizeof(cal_data.offsets));

    /* Read back data and confirm it was written correctly */
    nvram_read(0, &read_back, sizeof(vector3_t));

    if (memcmp(&cal_data.offsets, &read_back, sizeof(vector3_t))) {
      sensor->err = SENSOR_ERR_IO;
      return false;
    }

    break;

  default:
    return false;   /* bad step value */
  }

  return true;

}

void HMC5883_get_callibration_data(HMC5883_CALLIBRATION_DATA* data) {
  memcpy(data, &HMC5883_callibration_data, sizeof(HMC5883_CALLIBRATION_DATA));
}

void HMC5883_set_callibration_data(HMC5883_CALLIBRATION_DATA* data) {
  memcpy(&HMC5883_callibration_data, data, sizeof(HMC5883_CALLIBRATION_DATA));
}

#if 0
/**
* @brief Calibrate magnetometer
*
* This function measures the magnetometer output if 3 different device
* orientations, calculates average offset values, and stores these offsets
* in non-volatile memory.  The offsets will later be used during normal
* measurements, to compensate for fixed magnetic effects.
*
* This routine must be called 3 times total, with the "step" parameter
* indicating what stage of the calibration is being performed.  This
* multi-step mechanism allows the application to prompt for physical
* placement of the sensor device before this routine is called.
*
* @param sensor    Address of an initialized sensor descriptor.
* @param data      The address of a vector storing sensor axis data.
* @param step      The calibration stage number (1 to 3).
* @param info      Unimplemented (ignored) parameter.
* @return bool     true if the call succeeds, else false is returned.
*/
bool hmc5883l_calibrate(sensor_t *sensor, sensor_calibration_t calib_type,
  int step, void *info)
{
  static vector3_t step_data[3]; /* sensor readings during calibration */
  vector3_t dummy_data;          /* data from first sensor read (ignored) */
  vector3_t read_back;           /* data read back from nvram to validate */
  sensor_data_t test_data;       /* readings during self test */
  int test_code;                 /* self-test code & result */
  sensor_hal_t *const hal = sensor->hal;

  /* Validate the supported calibration types and step number. */
  if ((calib_type != MANUAL_CALIBRATE) || ((step < 1) || (step > 3))) {
    return false;
  }

  /* During first pass, use self-test to determine sensitivity scaling */
  if (step == 1) {
    /* Run internal self test with known bias field */
    test_code = SENSOR_TEST_BIAS_POS;

    if ((hmc5883l_selftest(sensor, &test_code,
      &test_data) == false) ||
      (test_code != SENSOR_TEST_ERR_NONE)) {
      return false;
    }

    /* Calculate & store sensitivity adjustment values */
    cal_data.sensitivity.x
      = ((scalar_t)HMC5883L_TEST_X_NORM / test_data.axis.x);
    cal_data.sensitivity.z
      = ((scalar_t)HMC5883L_TEST_Z_NORM / test_data.axis.z);
    cal_data.sensitivity.y
      = ((scalar_t)HMC5883L_TEST_Y_NORM / test_data.axis.y);

    nvram_write(CAL_SENSITIVITY_ADDR, &cal_data.sensitivity,
      sizeof(cal_data.sensitivity));

    /* Read back data and confirm it was written correctly */
    nvram_read(CAL_SENSITIVITY_ADDR, &read_back, sizeof(vector3_t));

    if (memcmp(&cal_data.sensitivity, &read_back,
      sizeof(vector3_t))) {
      sensor->err = SENSOR_ERR_IO;
      return false;
    }
  }

  /* Read sensor data and test for data overflow.
  *   Note: Sensor must be read twice - the first reading may
  *         contain stale data from previous orientation.
  */
  if (hmc5883l_get_data(hal, &dummy_data) != true) {
    return false;
  }

  delay_ms(READ_DELAY_MSEC);

  if (hmc5883l_get_data(hal, &(step_data[step - 1])) != true) {
    return false;
  }

  /* Apply sensitivity scaling factors */
  hmc5883l_apply_sensitivity(&(step_data[step - 1]));

  switch (step) {
    /* There's nothing more to do on the first two passes. */
  case 1:
  case 2:
    break;

    /* Calculate & store the offsets on the final pass. */
  case 3:
    cal_data.offsets.x = (step_data[0].x + step_data[1].x) / 2;
    cal_data.offsets.y = (step_data[0].y + step_data[1].y) / 2;
    cal_data.offsets.z = (step_data[1].z + step_data[2].z) / 2;

    nvram_write(CAL_OFFSETS_ADDR, &cal_data.offsets,
      sizeof(cal_data.offsets));

    /* Read back data and confirm it was written correctly */
    nvram_read(0, &read_back, sizeof(vector3_t));

    if (memcmp(&cal_data.offsets, &read_back, sizeof(vector3_t))) {
      sensor->err = SENSOR_ERR_IO;
      return false;
    }

    break;

  default:
    return false;   /* bad step value */
  }

  return true;
}
#endif
