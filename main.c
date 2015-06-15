#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>

#include "logging.h"
#include "cpu_cycles.h"
#include "input.h"
#include "sensors.h"
#include "sensor_fusion.h"
#include "linear_velocity_regulator.h"

/* 
 *  Rules:
 *  1. No memory allocation
 *  2. No file io in release build
 */

input_commands_t input_cmd;
sensor_data raw_sensor_data;
fused_sensor_data fused_data;
lin_vel_regulator_context lvr_ctx_x, lvr_ctx_y;

int loop(void) {
  long long start = cpu_cycles();

  /* 1. Read operator commands. */
  read_test_inputs(&input_cmd);

  /* 2. Read sensors. */
  int rc = read_sensors(&raw_sensor_data);
  if(rc) return rc;

  /* 3. Calculate sensor fusion data */
  fuse_sensor_data(&raw_sensor_data, &fused_data);

  /* 4. Perform linear velocities regulation */
  double pitch_cmd_rad = regulate_lin_vel(lvr_ctx_x, input_cmd.cmd_vel_x, fused_data.lin_acc.x, fused_data.time_s);
  double roll_cmd_rad = regulate_lin_vel(lvr_ctx_y, input_cmd.cmd_vel_y, fused_data.lin_acc.y, fused_data.time_s);

  /* 5. Perform roll regulation */
  /* 6. Perform pitch regulation */
  /* 7. Perform yaw regulation */
  /* 8. Perform height regulation */
  /* 9. Do control signal mixing */
  /*10. Set motor controller PWMs */
  /*11. Send telemetry */
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

  /*12. Perform loop frequency limiting */
  static const long long min_cycle_length_mcs = 5000; /* 200 Hz */
  const long long time_left_mcs = min_cycle_length_mcs - cycles_to_mcs( cpu_cycles() - start );
  if(time_left_mcs > 0) {
    delayMicroseconds(time_left_mcs);
  }

  return 0;
}

int main(void)
{
  int rc = init_sensors();
  if(rc) {
    exit(rc);
  }

  lvr_ctx_x = create_lin_vel_regulator(0.32, 0.1, 12.0 * M_PI / 180.0);
  lvr_ctx_y = create_lin_vel_regulator(0.32, 0.1, 12.0 * M_PI / 180.0);

  if(0 == lvr_ctx_x || 0 == lvr_ctx_y) {
    LOG_ERROR("error creating linear velocity regulators");
    exit(10);
  }

  while( !loop() );

  destroy_lin_vel_regulator(lvr_ctx_x);
  destroy_lin_vel_regulator(lvr_ctx_y);

  return 0;
}
