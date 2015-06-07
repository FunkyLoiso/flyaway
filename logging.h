#ifndef LOGGING_H_INCLUDED
#define LOGGING_H_INCLUDED

#include <stdio.h>

#define LOG_ERROR(...) {fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");}
#ifdef _DEBUG
  #define LOG_DEBUG(...) {frpintf(stdout, __VA_ARGS__); fprintf(stdout, "\n");}
#else
  #define LOG_DEBUG(...)
#endif

#endif
