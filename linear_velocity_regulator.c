#include "linear_velocity_regulator.h"

#include <math.h>   /* fmin, fmax */
#include <stdlib.h> /* malloc, free */

#include "integration.h"

typedef struct {
  double Kp;
  double Kd;
  double angle_limit;

  integration_context ictx;
}context;

double limit(double val, double min_limit, double max_limit){
  return fmin( max_limit, fmax(val, min_limit) );
}

lin_vel_regulator_context create_lin_vel_regulator(double Kp, double Kd, double angle_limit)
{
  context* ctx = (context*)malloc(sizeof(context));
  ctx->Kp = Kp;
  ctx->Kd = Kd;
  ctx->angle_limit = abs(angle_limit);

  integration_context ictx = {0.0};
  ctx->ictx = ictx;

  return ctx;
}

void destroy_lin_vel_regulator(lin_vel_regulator_context ctx)
{
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
  integrate(&ctx_->ictx, lin_acc, time_s);

  double unlimited_angle =  ((desired_vel - ctx_->ictx.accumulated_value) * ctx_->Kp) -
                            (lin_acc * ctx_->Kd);

  return limit(unlimited_angle, -ctx_->angle_limit, ctx_->angle_limit);
}

