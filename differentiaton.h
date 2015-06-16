#ifndef differentiaton_h__
#define differentiaton_h__

typedef void* differentiaton_context;

typedef enum {
  DIFF_FAST = 2,    /* df = ( f(x[0]) - f(x[-1]) ) / dt;    no delay   */
  DIFF_DELAY1 = 3,  /* df = ( f(x[1]) - f(x[-1]) ) / 2*dt;  1 sample delay */
  DIFF_DELAY2 = 5   /* df = 4/3 * ( f(x[1]) - f(x[-1]) ) / 2*dt  +  1/3 * ( f(x[2]) - f(x[-2]) ) / 4*dt;  2 samples delay */
} differentiation_mode;

differentiaton_context create_differentiator(differentiation_mode mode);
void destroy_differentiator(differentiaton_context ctx);
double differentiate(differentiaton_context ctx, double value, double cur_time_s);

#endif // differentiaton_h__
