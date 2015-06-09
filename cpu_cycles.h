#ifndef CPU_CYCLES_H_INCLUDED
#define CPU_CYCLES_H_INCLUDED

static const long long RASPBERRY_PI_FREQ_HZ = 700000000; /* 700 MHz */

long long cpu_cycles();
inline long long cycles_to_ns(long long cycles) {
  return cycles * 1000000000 / RASPBERRY_PI_FREQ_HZ;
}

inline double cycles_to_s(long long cycles) {
  return ((double)cycles) / RASPBERRY_PI_FREQ_HZ;
}


#endif // CPU_CYCLES_H_INCLUDED
