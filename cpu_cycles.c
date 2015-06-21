#include "cpu_cycles.h"

#include <unistd.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>

/* taken from http://neocontra.blogspot.co.uk/2013/05/user-mode-performance-counters-for.html */

static int fddev = -1;

__attribute__((constructor)) static void /* will run before main */
init(void)
{
 static struct perf_event_attr attr;
 attr.type = PERF_TYPE_HARDWARE;
 attr.config = PERF_COUNT_HW_CPU_CYCLES;
 fddev = syscall(__NR_perf_event_open, &attr, 0, -1, -1, 0);
}

__attribute__((destructor)) static void /* will run after main */
fini(void)
{
 close(fddev);
}

long long cpu_cycles()
{
 long long result = 0;
 if (read(fddev, &result, sizeof(result)) != sizeof(result)) return 0;
 return result;
}
