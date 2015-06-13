#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>

#include "input.h"
#include "sensors.h"
#include "sensor_fusion.h"

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
sensor_data raw_sensor_data;
fused_sensor_data fused_data;

void loop(void) {
  /* 1. Read operator commands. */
  read_test_inputs(&input_cmd);
  /* 2. Read sensors. */
  int rc = read_sensors(&raw_sensor_data);
  if(rc){

  }
  /* 3. Calculate sensor fusion data */
  fuse_sensor_data(&raw_sensor_data, &fused_data);
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
           raw_sensor_data.acc_data.data.x, raw_sensor_data.acc_data.data.y, raw_sensor_data.acc_data.data.z,
           raw_sensor_data.altitude.val,
           raw_sensor_data.avel_data.data.x, raw_sensor_data.avel_data.data.y, raw_sensor_data.avel_data.data.z,
           raw_sensor_data.bmp085_temp.val,
           raw_sensor_data.itg3200_temp.val,
           raw_sensor_data.mag_data.data.x, raw_sensor_data.mag_data.data.y, raw_sensor_data.mag_data.data.z);
    last_output = millis();
    fflush(stdout);
  }
}

int main(void)
{
  setup();
  for (;;) loop();
}

