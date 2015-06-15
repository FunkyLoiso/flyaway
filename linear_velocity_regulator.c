#include "linear_velocity_regulator.h"

#include <math.h>   /* fmin, fmax */
#include <stdlib.h> /* malloc, free */

#include "math_funcs.h"
#include "integration.h"

typedef struct {
  double Kp;
  double Kd;
  double angle_limit_rad;

  integration_context ictx;
}context;

lin_vel_regulator_context create_lin_vel_regulator(double Kp, double Kd, double angle_limit_rad)
{
  context* ctx = (context*)malloc(sizeof(context));
  ctx->Kp = Kp;
  ctx->Kd = Kd;
  ctx->angle_limit_rad = abs(angle_limit_rad);

  ctx->ictx = create_integrator(0.0);

  return ctx;
}

void destroy_lin_vel_regulator(lin_vel_regulator_context ctx)
{
  destroy_integrator( ((context*)ctx)->ictx );
  free(ctx);
}

/*
 * des_vel ---->(-)---->(*Kp)---->(-)---->(limit)----> angle_out
 *               ^                 ^
 *               |                 |             
 *          (integrator)           |
 *               |                 |
 * lin_acc ------+----->(*Kd)------+                                            
 */
double regulate_lin_vel(lin_vel_regulator_context ctx, double desired_vel, double lin_acc, double time_s)
{
  context* ctx_ = (context*)ctx;
  double lin_vel = integrate(&ctx_->ictx, lin_acc, time_s);

  double unlimited_angle =  ((desired_vel - lin_vel) * ctx_->Kp) -
                            (lin_acc * ctx_->Kd);

  return limit(unlimited_angle, -ctx_->angle_limit_rad, ctx_->angle_limit_rad);
}

