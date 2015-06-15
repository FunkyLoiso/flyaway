#ifndef math_funcs_h__
#define math_funcs_h__

#include <math.h>

double limit(double val, double min_limit, double max_limit){
  return fmin( max_limit, fmax(val, min_limit) );
}

#endif // math_funcs_h__
