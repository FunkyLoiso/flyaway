#include "integration.h"

void integrate(integration_context* ctx, double value, double cur_time_s)
{
  if(ctx->last_time_s != 0.0) { /* don't update values the first time after reset */
    double dt_s = cur_time_s - ctx->last_time_s;
    ctx->accumulated_value += value * dt_s;
    ctx->total_time_s += dt_s;
  }
  ctx->last_time_s = cur_time_s;
}
