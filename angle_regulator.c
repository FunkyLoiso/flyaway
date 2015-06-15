#include "angle_regulator.h"

#include <math.h>   /* fmin, fmax */
#include <stdlib.h> /* malloc, free */

#include "integration.h"

typedef struct {
  double Kp;
  double Ki;
  double Kd;

  integration_context ictx;
}context;

angle_regulator_context create_angle_regulator(double Kp, double Ki, double Kd, double integrator_limit_rad)
{
  context* ctx = (context*)malloc(sizeof(context));
  ctx->Kp = Kp;
  ctx->Ki = Ki;
  ctx->Kd = Kd;
  ctx->ictx = create_integrator(integrator_limit_rad);

  return ctx;
}

void destroy_angle_regulator(angle_regulator_context ctx)
{
  destroy_integrator( ((context*)ctx)->ictx );
  free(ctx);
}

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
double regulate_angle(angle_regulator_context ctx, double desired_ang, double real_ang, double angular_vel, double time_s)
{
  context* ctx_ = (context*)ctx;
  double ang_error = desired_ang - real_ang;
  return  (ang_error * ctx_->Kp) +
          (integrate(ctx_->ictx, ang_error, time_s) * ctx_->Ki) -
          (angular_vel * ctx_->Kd);
}
