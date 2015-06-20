#ifndef LOGGING_H_INCLUDED
#define LOGGING_H_INCLUDED

#include <stdio.h>

#define LOG_ERROR(...) { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_ERROR_ERRNO(...) { fprintf(stderr, __VA_ARGS__); fprintf(stderr, " errno: %d\nstrerror: \"%s\"\n", errno, strerror(errno)); }
#ifdef _DEBUG
  #define LOG_DEBUG(...) {printf(__VA_ARGS__); printf("\n");}
#else
  #define LOG_DEBUG(...)
#endif

#endif
