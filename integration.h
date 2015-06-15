#ifndef integration_h__
#define integration_h__

typedef void* integration_context;

/* 0.0 for no limit */
integration_context create_integrator(double saturation_limit);
void destroy_integrator(integration_context ctx);
double integrate(integration_context ctx, double value, double cur_time_s);

#endif // integration_h__
