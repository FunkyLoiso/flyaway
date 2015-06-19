#ifndef motors_controller_h__
#define motors_controller_h__

#include "throttle_mixing.h"

static const int MOTORS_CONTROLLER_MIN_PWM_FREQ = 24;
static const int MOTORS_CONTROLLER_MAX_PWM_FREQ = 1536;

int init_motors_controller(unsigned char adr, int front_ch, int back_ch, int left_ch, int right_ch, double min_duty_cycle, double max_duty_cycle, int pwm_freq);
int set_motors_throttles(motors_throttles throttles);

#endif // motors_controller_h__
