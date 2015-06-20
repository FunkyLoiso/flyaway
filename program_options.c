#include "program_options.h"
#include <unistd.h>
#include <libconfig.h>

static config_t config;

int parse_command_line(int argc, const char* argv[])
{
  int opt;

  while ((opt = getopt(argc, argv, "c:")) != -1) {
    switch (opt) {
        case 'c':
          const char* cfg_path = optarg;
          config_destroy(&config);
          config_init(&config);

          break;
        default: /* '?' */
          fprintf(stderr, "Usage: %s [-c cfg-path]\n", argv[0]);
          return -1;
    }
  }

}

