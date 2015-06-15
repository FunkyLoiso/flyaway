#include "integration.h"

#include "math_funcs.h"


typedef struct {
  double accumulated_value;
  double saturation_limit;
  double last_time_s;
  double total_time_s;
}context;

integration_context create_integrator(double saturation_limit)
{
  context* ret = (context*) malloc(sizeof(context));
  ret->accumulated_value = 0.0;
  ret->saturation_limit = abs(saturation_limit);
  ret->last_time_s = 0.0;
  ret->total_time_s = 0.0;

  return ret;
}

void destroy_integrator(integration_context ctx)
{
  free(ctx);
}

double integrate(integration_context ctx, double value, double cur_time_s)
{
  context* ctx_ = (context*)ctx;
  if(0.0 != ctx_->last_time_s) { /* don't update values the first time after reset */
    double dt_s = cur_time_s - ctx_->last_time_s;
    ctx_->accumulated_value += value * dt_s;
    if(0.0 != ctx_->saturation_limit) {
      ctx_->accumulated_value = limit(ctx_->accumulated_value, -ctx_->saturation_limit, ctx_->saturation_limit);
    }
    ctx_->total_time_s += dt_s;
  }
  ctx_->last_time_s = cur_time_s;
  return ctx_->accumulated_value;
}

