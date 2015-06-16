#include "altitude_regulator.h"

#include "integration.h"

struct {
  double Kp, Ki, Kd;
  double gravity_offset;
  integration_context int_ctx;
  differentiaton_context diff_ctx;
} context;

altitude_regulator_context create_altitude_regulator(
  double Kp, double Ki, double Kd, double gravity_offset, double int_limit, differentiation_mode diff_mode)
{
  context* ctx = (context*) malloc(sizeof(context));
  *ctx = (context) {Kp, Ki, Kd, gravity_offset};
  ctx->int_ctx = create_integrator(int_limit);
  ctx->diff_ctx = create_differentiator(diff_mode);

  return ctx;
}

void destroy_altitude_regulator(altitude_regulator_context ctx)
{
  context* ctx_ = (context*) ctx;
  destroy_integrator(ctx_->int_ctx);
  destroy_differentiator(ctx->->diff_ctx);
  free(ctx);
}

/* returns throttle correction 
 * gravity_offset ------------------------------------------+
 *                       +------------------->(*Kp)------v  v
 *                       |                              (+)(+)
 * desired_alt ---->(-)--+-->(integrator)---->(*Ki)---->(+)---->throttle correction out
 *                   ^                                  (-)
 *                   |                                   ^
 * real_alt ---------+----(differentiator)--->(*Kd)------+
 */
double regulate_altitude(altitude_regulator_context ctx, double desired_alt, double real_alt, double time_s)
{
  context* ctx_ = (context*) ctx;
  double alt_error = desired_alt - real_alt;
  return  ctx_->gravity_offset +
          alt_error * ctx_->Kp +
          integrate(ctx_->int_ctx, alt_error, time_s) * ctx_->Ki -
          differentiate(ctx_->diff_ctx, real_alt, time_s) + ctx_->Kd;
}
