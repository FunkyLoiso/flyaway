#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h> // for memcpy

#include <wiringPiI2C.h>
#include <wiringPi.h> // for delay

#include "HMC5883.h"
#include "HMC5883_registers.h"
#include "logging.h"
#include "twos_complement.h"
#include "cpu_cycles.h"

int HMC5883_fd = -1;
unsigned char HMC5883_adr = 0x00;
HMC5883_DATA_RATE HMC5883_data_rate = -1;
HMC5883_GAIN HMC5883_gain = -1;
HMC5883_CALIBRATION_DATA HMC5883_calibration_data = { .offset = {0, 0, 0}, .sensitivity = {1.0, 1.0, 1.0} };

/* private */
static int select_device() {
  //maybe we need to do HMC5883_adr << 1 for write and (HMC5883_adr << 1) | 1 for read?
  return ioctl(HMC5883_fd, I2C_SLAVE, HMC5883_adr);
}

static int write_device(char* buf, int len) {
  return write(HMC5883_fd, buf, len);
}

static int read_device(char* buf, int len) {
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

int read_mag_raw(sensor_sample_int_3d* raw_data) {
  char buf[6];
  buf[0] = HMC5883_MAG_X_HI;

  int rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 6);
  if (6 != rc) return -2;
  raw_data->ts = cpu_cycles();

  // sensor register order is X Z Y
  raw_data->data.x = from_bytes16(buf[1], buf[0]); /* MSB then LSB */
  raw_data->data.z = from_bytes16(buf[3], buf[2]);
  raw_data->data.y = from_bytes16(buf[5], buf[4]);

  if ((raw_data->data.x == DATA_OUTPUT_OVERFLOW) |
      (raw_data->data.y == DATA_OUTPUT_OVERFLOW) |
      (raw_data->data.z == DATA_OUTPUT_OVERFLOW)) {
    return HMC5883_OVERFLOW;
  }

  return 0;
}

int selftest(sensor_sample_int_3d *out_data)
{
  static const int SELF_TEST_DELAY_MS = 250;
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

  rc = read_mag_raw(out_data);
  if (rc) {
    return_code = rc;
    goto exit;
  }

#ifdef check_range_of_readings
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
#endif

exit:
  /* Restore registers */
  buf[0] = HMC5883_CONFIG_REG_A;
  rc = write_device(buf, 1);
  if (1 != rc) return -50;
  rc = write_device((char *)&reg_set, sizeof(reg_set));
  if (sizeof(reg_set) != rc) {
    return -60;
  }

  return return_code;
}

void apply_sensitivity(vector_int_3d* raw_data) {
  raw_data->x *= HMC5883_calibration_data.sensitivity.x;
  raw_data->y *= HMC5883_calibration_data.sensitivity.y;
  raw_data->z *= HMC5883_calibration_data.sensitivity.z;
}

void apply_offset(vector_int_3d* sensitivity_scaled_raw_data) {
  sensitivity_scaled_raw_data->x -= HMC5883_calibration_data.offset.x;
  sensitivity_scaled_raw_data->y -= HMC5883_calibration_data.offset.y;
  sensitivity_scaled_raw_data->z -= HMC5883_calibration_data.offset.z;
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
  buf[0] = HMC5883_ID_REG_A;
  int rc = write_device(buf, 1);
  if (1 != rc) return -1;

  rc = read_device(buf, 3);
  if(3 != rc) return -2;

  if ((buf[0] != ID_A_DEFAULT) |
      (buf[1] != ID_B_DEFAULT) |
      (buf[2] != ID_C_DEFAULT)) {
    LOG_ERROR("HMC5883 id check failed. Expected 0x%x 0x%x 0x%x, got 0x%x 0x%x 0x%x", ID_A_DEFAULT, ID_B_DEFAULT, ID_C_DEFAULT, buf[0], buf[1], buf[2]);
    return -3;
  }

  //set data output rate and measure mode
  buf[0] = HMC5883_CONFIG_REG_A;
  buf[1] = data_rate | MEAS_MODE_NORM;

  rc = write_device(buf, 2);
  if (2 != rc) return -4;

  //set gain
  buf[0] = HMC5883_CONFIG_REG_B;
  buf[1] = gain;

  rc = write_device(buf, 2);
  if (2 != rc) return -5;

  //set continuous-measurement mode
  buf[0] = HMC5883_MODE_REG;
  buf[1] = MODE_CONTIN;
  rc = write_device(buf, 2);
  if (2 != rc) return -6;

  LOG_DEBUG("HMC5883 on adr 0x%x init OK", adr);
  return 0;
}

int HMC5883_read(sensor_sample_3d* mag)
{
  int rc = select_device();
  if (rc) return rc;

  sensor_sample_int_3d raw_data;
  rc = read_mag_raw(&raw_data);
  if (rc) return rc;

  apply_sensitivity(&raw_data.data);
  apply_offset(&raw_data.data);

  double counts_per_Gauss = gain_to_scale(HMC5883_gain);
  mag->data.x = ((double)raw_data.data.x) / counts_per_Gauss;
  mag->data.y = ((double)raw_data.data.y) / counts_per_Gauss;
  mag->data.z = ((double)raw_data.data.z) / counts_per_Gauss;
  mag->ts = raw_data.ts;

  return 0;
}

int HMC5883_calibrate(int step) {
  static sensor_sample_int_3d step_data[3];  /* sensor readings during calibration */
  sensor_sample_int_3d dummy_data;           /* data from first sensor read (ignored) */
  sensor_sample_int_3d test_data;            /* readings during self test */
  int rc;

  /* Validate the supported calibration types and step number. */
  if ((step < 1) || (step > 3)) {
    return -1;
  }

  /* During first pass, use self-test to determine sensitivity scaling */
  if (step == 1) {
    /* Run internal self test with known bias field */
    rc = selftest(&test_data);
    if (rc) return -100 + rc;

    /* Calculate & store sensitivity adjustment values */
    HMC5883_calibration_data.sensitivity.x = ((double)HMC5883_TEST_X_NORM) / test_data.data.x;
    HMC5883_calibration_data.sensitivity.y = ((double)HMC5883_TEST_Y_NORM) / test_data.data.y;
    HMC5883_calibration_data.sensitivity.z = ((double)HMC5883_TEST_Z_NORM) / test_data.data.z;
  }

  /* Read sensor data and test for data overflow.
  *   Note: Sensor must be read twice - the first reading may
  *         contain stale data from previous orientation.
  */
  rc = read_mag_raw(&dummy_data);
  if (rc) return -200 + rc;

  delay(100);

  rc = read_mag_raw(&(step_data[step - 1]));
  if (rc) return -300 + rc;

  /* Apply sensitivity scaling factors */
  apply_sensitivity(&(step_data[step - 1]).data);

  switch (step) {
    /* There's nothing more to do on the first two passes. */
  case 1:
  case 2:
    break;

    /* Calculate & store the offsets on the final pass. */
  case 3:
    /*@TODO: I don't understand this shit. x offset is average between 1 and 2 step? Should the callibration rotations be exactly 90 degrees?*/
    HMC5883_calibration_data.offset.x = (step_data[0].data.x + step_data[1].data.x) / 2;
    HMC5883_calibration_data.offset.y = (step_data[0].data.y + step_data[1].data.y) / 2;
    HMC5883_calibration_data.offset.z = (step_data[1].data.z + step_data[2].data.z) / 2;
    break;

  default:
    return -400;
  }

  return 0;
}

void HMC5883_get_calibration_data(HMC5883_CALIBRATION_DATA* data) {
  memcpy(data, &HMC5883_calibration_data, sizeof(HMC5883_CALIBRATION_DATA));
}

void HMC5883_set_calibration_data(HMC5883_CALIBRATION_DATA* data) {
  memcpy(&HMC5883_calibration_data, data, sizeof(HMC5883_CALIBRATION_DATA));
}
