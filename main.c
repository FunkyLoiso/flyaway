#include "input.h"

/* 
 *  Rules:
 *  1. No memory allocation
 *  2. No file io in release build
 */

void setup(void) {
}

input_commands_t input_cmd;

void loop(void) {
  /* 1. Read operator commands. */
  read_test_inputs(&input_cmd);
  /* 2. Read sensors. */
  /* 3. Calculate sensor fusion data */
  /* 4. Calculate sensor-based data */
  /* 5. Perform linear velocities regulation */
  /* 6. Perform roll regulation */
  /* 7. Perform pitch regulation */
  /* 8. Perform yaw regulation */
  /* 9. Perform height regulation */
  /*10. Do control signal mixing */
  /*11. Set motor controller PWMs */
  /*12. Send telemetry */
  /*13. Perform loop frequency limiting */
}

int main(void)
{
  setup();
  for (;;) loop();
}

