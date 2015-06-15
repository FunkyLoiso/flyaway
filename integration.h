#ifndef integration_h__
#define integration_h__

typedef struct {
  double accumulated_value;
  double last_time_s;
  double total_time_s;
}integration_context;

void integrate(integration_context* ctx, double value, double cur_time_s);

#endif // integration_h__
