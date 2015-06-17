#include "throttle_mixing.h"

void mix_throttles(throttle_correction* cor, motors_throttles* thr)
{
  thr->m_head = cor->d_throttle_alt - cor->d_throttle_yaw - cor->d_throttle_pitch; /* head motor is CCW, positive yaw is CCW, positive pitch is head down */
  thr->m_tail = cor->d_throttle_alt - cor->d_throttle_yaw + cor->d_throttle_pitch; /* tail motor is also CCW */
  thr->m_left = cor->d_throttle_alt + cor->d_throttle_yaw + cor->d_throttle_roll; /* left motor is CW, positive roll is left up */
  thr->m_right = cor->d_throttle_alt + cor->d_throttle_yaw - cor->d_throttle_roll; /* right motor is also CW */
}
