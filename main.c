#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>

#include "input.h"
#include "sensors.h"

/* 
 *  Rules:
 *  1. No memory allocation
 *  2. No file io in release build
 */

void setup(void) {
  int rc = init_sensors();
  if(rc) {
    exit(rc);
  }
}

input_commands_t input_cmd;
SENSOR_DATA sensor_data;

void loop(void) {
  /* 1. Read operator commands. */
  read_test_inputs(&input_cmd);
  /* 2. Read sensors. */
  int rc = read_sensors(&sensor_data);
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
  delay(5); /*no more than 200 Hz*/

  static unsigned int last_output = 0;
  if(millis() - last_output > 1000) {
    printf("acc: %f %f %f alt: %f avel: %f %f %f bar_temp: %f guro_temp: %f mag: %f %f %f\n",
           sensor_data.acc_data.x, sensor_data.acc_data.y, sensor_data.acc_data.z,
           sensor_data.altitude,
           sensor_data.avel_data.x, sensor_data.avel_data.y, sensor_data.avel_data.z,
           sensor_data.bmp085_temp,
           sensor_data.itg3200_temp,
           sensor_data.mag_data.x, sensor_data.mag_data.y, sensor_data.mag_data.z);
    last_output = millis();
    fflush(stdout);
  }
}

int main(void)
{
  setup();
  for (;;) loop();
}

