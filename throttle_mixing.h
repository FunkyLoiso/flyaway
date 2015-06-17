#ifndef throttle_mixing_h__
#define throttle_mixing_h__

typedef struct {
  double d_throttle_roll, d_throttle_pitch, d_throttle_yaw, d_throttle_alt;
}throttle_correction;

typedef struct {
 double m_head, m_tail, m_left, m_right;
} motors_throttles;

void mix_throttles(throttle_correction* correction, motors_throttles* out_motors_throttles);

#endif // throttle_mixing_h__
