#include "differentiaton.h"

#include <stdlib.h> /* malloc, free */
#include <string.h> /* memmove */

#include "logging.h"

typedef struct {
  differentiation_mode mode;
  double values[5];
  double times[5];
  int samples_collected;
} context;

differentiaton_context create_differentiator(differentiation_mode mode)
{
  context* ctx = (context*) malloc(sizeof(context));
  *ctx = (context) {mode, {0.0}, 0.0, 0};
  return ctx;
}

void destroy_differentiator(differentiaton_context ctx)
{
  free(ctx);
}

double differentiate(differentiaton_context ctx, double value, double cur_time_s)
{
  context* ctx_ = (context*)ctx;
  if(ctx_->samples_collected < (ctx_->mode-1)) { /* first collect required number of samples - 1 */
    ctx_->values[ctx_->samples_collected] = value;
    ctx_->times[ctx_->samples_collected] = cur_time_s;
    ++ctx_->samples_collected;
    return 0.0;
  }

  /* add new value and time */
  ctx_->values[ctx_->mode-1] = value;
  ctx_->times[ctx_->mode-1] = cur_time_s;

  /* calculate new ret value based on mode */
  double* point_value = ctx_->values + ctx_->mode-1; /* value at the diff point */
  double* point_time = ctx_->times + ctx_->mode-1; /* time at the diff point */
  double ret;
  switch(ctx_->mode) {
    case DIFF_FAST:
      ret = (point_value[0] - point_value[-1]) / (point_time[0] - point_time[-1]);
      break;

    case DIFF_DELAY1:
      ret = (point_value[1] - point_value[-1]) / (point_time[1] - point_time[-1]);
      break;

    case DIFF_DELAY2: {
        double r1 = 4.0/3.0 * (point_value[1] - point_value[-1]) / (point_time[1] - point_time[-1]);
        double r2 = 1.0/3.0 * (point_value[2] - point_value[-2]) / (point_time[2] - point_time[-2]);
        ret = r1 - r2;
      }
      break;

    default:
      LOG_ERROR("Invalid diff mode: %d", ctx_->mode);
      ret = 0.0;
  }

  /* shift values and times left one position */
  memmove( ctx_->values, ctx_->values+1, sizeof(double) * (ctx_->mode-1) );
  memmove( ctx_->times, ctx_->times+1, sizeof(double) * (ctx_->mode-1) );

  return ret;
}
