#ifndef altitude_regulator_h__
#define altitude_regulator_h__

#include "differentiaton.h"

typedef void* altitude_regulator_context;

altitude_regulator_context create_altitude_regulator(double Kp, double Ki, double Kd, double gravity_offset, double int_limit, differentiation_mode diff_mode);
void destroy_altitude_regulator(altitude_regulator_context ctx);

/* returns throttle correction 
 * gravity_offset ------------------------------------------+
 *                       +------------------->(*Kp)------v  v
 *                       |                              (+)(+)
 * desired_alt ---->(-)--+-->(integrator)---->(*Ki)---->(+)---->throttle correction out
 *                   ^                                  (-)
 *                   |                                   ^
 * real_alt ---------+----(differentiator)--->(*Kd)------+
 */
double regulate_altitude(altitude_regulator_context ctx, double desired_alt, double real_alt, double time_s);

#endif // altitude_regulator_h__
