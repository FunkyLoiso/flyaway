#ifndef linear_velocity_regulator_h__
#define linear_velocity_regulator_h__

typedef void* lin_vel_regulator_context;

lin_vel_regulator_context create_lin_vel_regulator(double Kp, double Kd, double angle_limit);
void destroy_lin_vel_regulator(lin_vel_regulator_context ctx);

/* returns angle command in radians
 *
 * des_vel ---->(-)---->(*Kp)---->(-)---->(limit)----> angle_out
 *               ^                 ^
 *               |                 |             
 *          (integrator)           |
 *               |                 |
 * lin_acc ------+----->(*Kd)------+                                           
 */
double regulate_lin_vel(lin_vel_regulator_context ctx, double desired_vel, double lin_acc, double time_s);

#endif // linear_velocity_regulator_h__
