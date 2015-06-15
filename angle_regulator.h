#ifndef angle_regulator_h__
#define angle_regulator_h__

typedef void* angle_regulator_context;

angle_regulator_context create_angle_regulator(double Kp, double Ki, double Kd, double integrator_limit_rad);
void destroy_angle_regulator(angle_regulator_context ctx);

/* returns throttle correction 
 *
 *                       +------------------->(*Kp)------v
 *                       |                              (+)
 * desired_ang ---->(-)--+-->(integrator)---->(*Ki)---->(+)---->throttle correction out
 *                   ^                                  (-)
 * real_ang ---------+                                   ^
 *                                                       |
 * angular_vel ------------------------------>(*Kd)------+
 */
double regulate_angle(angle_regulator_context ctx, double desired_ang, double real_ang, double angular_vel, double time_s);

#endif // angle_regulator_h__
