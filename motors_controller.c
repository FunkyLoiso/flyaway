#include "motors_controller.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>

#include "logging.h"
#include "PCA9685_registers.h"

static const double clock_freq = 25000000.0;

int PCA9685_fd = -1;
unsigned char PCA9685_adr = 0x00;

struct {
 int front, tail, left, right;
}channels;

struct {
  double min, width;
}duty_cycle_limits;

/* private */
static int select_device() {
  return ioctl(PCA9685_fd, I2C_SLAVE, PCA9685_adr);
}

static int write_device(char* buf, int len) {
  return write(PCA9685_fd, buf, len);
}

static int read_device(char* buf, int len) {
  return read(PCA9685_fd, buf, len);
}

static int map_throttle(double throttle) {
  return rint( (throttle * duty_cycle_limits.width + duty_cycle_limits.min) * 4096 );
}

/* public */
int init_motors_controller(unsigned char adr, int front_ch, int tail_ch, int left_ch, int right_ch, double min_duty_cycle, double max_duty_cycle, int pwm_freq)
{
  if( (front_ch < 0 || front_ch > 15) ||
      (tail_ch < 0 || tail_ch > 15) ||
      (left_ch < 0 || left_ch > 15) ||
      (right_ch < 0 || right_ch > 15) ) {
    LOG_ERROR("Channel numbers must be between 0 and 15");
    return -10;
  }

  if( (min_duty_cycle < 0.0 || min_duty_cycle > 1.0) ||
      (max_duty_cycle < 0.0 || max_duty_cycle > 1.0)  ) {
    LOG_ERROR("Duty cycle limits must be between 0.0 and 1.0");
    return -20;
  }

  if(pwm_freq < MOTORS_CONTROLLER_MIN_PWM_FREQ || pwm_freq > MOTORS_CONTROLLER_MAX_PWM_FREQ) {
    LOG_ERROR("PWM frequency must be between %d and %d", MOTORS_CONTROLLER_MIN_PWM_FREQ, MOTORS_CONTROLLER_MAX_PWM_FREQ);
    return -30;
  }

  int fd = wiringPiI2CSetup(adr);
  if (fd < 0)  {
    return -40;
  }

  PCA9685_fd = fd;
  PCA9685_adr = adr;

  channels.front = front_ch;
  channels.tail = tail_ch;
  channels.left = left_ch;
  channels.right = right_ch;

  duty_cycle_limits.min = min_duty_cycle;
  duty_cycle_limits.width = max_duty_cycle - min_duty_cycle;

  uint8_t prescale_val = rint(clock_freq / 4096 / pwm_freq) - 1;

  char buf[2];
  /* set mode to sleep, set frequency, restart */
  buf[0] = PCA9685_MODE1;
  buf[1] = MODE1_SLEEP;
  int rc = write_device(buf, 2);
  if(2 != rc) return -50;
  delay(2); /* wait a bit for the device to go to sleep */
  
  buf[0] = PCA9685_PRESCALE;
  buf[1] = prescale_val;
  rc = write_device(buf, 2);
  if(2 != rc) return -60;

  /* set auto increment mode and remove sleep bit */
  buf[0] = PCA9685_MODE1;
  buf[1] = MODE1_AUTO_INCREMENT;
  int rc = write_device(buf, 2);
  if(2 != rc) return -70;

  return 0;
}

int set_motors_throttles(motors_throttles throttles)
{
  int head_off = map_throttle(throttles.m_head);
  int tail_off = map_throttle(throttles.m_tail);
  int left_off = map_throttle(throttles.m_left);
  int right_off = map_throttle(throttles.m_right);

  int rc = select_device();
  if(rc) return rc;

  char buf[3];
  buf[0] = LED_N_OFF_H(channels.front);
  buf[1] = head_off >> 8;
  buf[2] = head_off & 0xff;
  rc = write_device(buf, 3);
  if(3 != rc) return -10;

  buf[0] = LED_N_OFF_H(channels.tail);
  buf[1] = tail_off >> 8;
  buf[2] = tail_off & 0xff;
  rc = write_device(buf, 3);
  if(3 != rc) return -20;

  buf[0] = LED_N_OFF_H(channels.left);
  buf[1] = left_off >> 8;
  buf[2] = left_off & 0xff;
  rc = write_device(buf, 3);
  if(3 != rc) return -30;

  buf[0] = LED_N_OFF_H(channels.right);
  buf[1] = right_off >> 8;
  buf[2] = right_off & 0xff;
  rc = write_device(buf, 3);
  if(3 != rc) return -40;

  return 0;
}
