#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "logging.h"
#include "cpu_cycles.h"
#include "input.h"
#include "sensors.h"
#include "sensor_fusion.h"
#include "linear_velocity_regulator.h"
#include "angle_regulator.h"
#include "altitude_regulator.h"
#include "throttle_mixing.h"
#include "motors_controller.h"

/* 
 *  Rules:
 *  1. No memory allocation
 *  2. No file io in release build
 */

input_commands_t input_cmd = {};
sensor_data raw_sensor_data = {};
fused_sensor_data fused_data = {};
lin_vel_regulator_context lvr_ctx_x = 0, lvr_ctx_y = 0;
angle_regulator_context ar_ctx_yaw = 0, ar_ctx_pitch = 0, ar_ctx_roll = 0;
altitude_regulator_context alt_reg_ctx = 0;
throttle_correction thr_correction = {};
motors_throttles motors_thr = {};

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
  thr_correction.d_throttle_roll = regulate_angle(ar_ctx_roll, roll_cmd_rad, fused_data.attitude.roll, fused_data.avel.x, fused_data.time_s);

  /* 6. Perform pitch regulation */
  thr_correction.d_throttle_pitch = regulate_angle(ar_ctx_pitch, pitch_cmd_rad, fused_data.attitude.pitch, fused_data.avel.y, fused_data.time_s);

  /* 7. Perform yaw regulation */
  thr_correction.d_throttle_yaw = regulate_angle(ar_ctx_yaw, input_cmd.cmd_yaw, fused_data.attitude.yaw, fused_data.avel.z, fused_data.time_s);

  /* 8. Perform altitude regulation */
  thr_correction.d_throttle_alt = regulate_altitude(alt_reg_ctx, input_cmd.cmd_h, fused_data.altitude, fused_data.time_s);

  /* 9. Do control signal mixing */
  mix_throttles(&thr_correction, &motors_thr);

  /*10. Set motor controller PWMs */
  /*rc = set_motors_throttles(motors_thr);
  if(rc) return rc;*/

  /*11. Send telemetry */
  static unsigned int last_output = 0;
  if(millis() - last_output > 500) {
    printf("%f acc: %f %f %f alt: %f avel: %f %f %f bar_temp: %f guro_temp: %f mag: %f %f %f",
      cycles_to_s( raw_sensor_data.acc_data.ts ),
      raw_sensor_data.acc_data.data.x, raw_sensor_data.acc_data.data.y, raw_sensor_data.acc_data.data.z,
      raw_sensor_data.altitude.val,
      raw_sensor_data.avel_data.data.x, raw_sensor_data.avel_data.data.y, raw_sensor_data.avel_data.data.z,
      raw_sensor_data.bmp085_temp.val,
      raw_sensor_data.itg3200_temp.val,
      raw_sensor_data.mag_data.data.x, raw_sensor_data.mag_data.data.y, raw_sensor_data.mag_data.data.z);

    printf(" \n");

    last_output = millis();
    fflush(stdout);
  }

  /*12. Perform loop frequency limiting */
//  static const long long min_cycle_length_mcs = 5000; /* 200 Hz */
//  const long long time_left_mcs = min_cycle_length_mcs - cycles_to_mcs( cpu_cycles() - start );
//  if(time_left_mcs > 0) {
//    delayMicroseconds(time_left_mcs);
//  }

  return 0;
}

int main(/*int argc, const char* argv[]*/)
{
//  int r = ITG3200_init(0x68, ITG3200_FILTER_BANDWIDTH_5, 3);
//  if(r) printf("%d: %s", r, strerror(errno));
//  FILE* itg3200_curve_file = fopen("/tmp/curve.csv", "w");
//  if(!itg3200_curve_file) {
//    printf("%s", strerror(errno));
//  }
//  else {
//    r = itg3200_callibration_curve(1000, 900000, itg3200_curve_file);
//    if(r) printf("%d: %s", r, strerror(errno));
//  }
//  exit(r);

  int rc = init_sensors();
  if(rc) {
    exit(rc);
  }

  rc = zero_altitude();
  if(rc) {
    exit(rc);
  }

  /* default address is 0100 0000 */
  /*rc = init_motors_controller(0x40, 0, 4, 8, 12, 0.3, 0.8, 200);
  if(rc) {
    LOG_ERROR_ERRNO("Error during PCA9685 init. Internal code: %d,", rc);
    exit(rc);
  }*/

  /* linear velocity regulators */
  lvr_ctx_x = create_lin_vel_regulator(0.32, 0.1, 12.0 * M_PI / 180.0);
  lvr_ctx_y = create_lin_vel_regulator(0.32, 0.1, 12.0 * M_PI / 180.0);

  if(0 == lvr_ctx_x || 0 == lvr_ctx_y) {
    LOG_ERROR("Error creating linear velocity regulators");
    exit(10);
  }

  /* angle regulators */
  ar_ctx_pitch = create_angle_regulator(2.0, 1.1, 1.2, 1.0);
  ar_ctx_roll = create_angle_regulator(2.0, 1.1, 1.2, 1.0);
  ar_ctx_yaw = create_angle_regulator(4.0, 0.5, 3.5, 1.0);
  if(0 == ar_ctx_roll || 0 == ar_ctx_pitch || 0 == ar_ctx_yaw) {
    LOG_ERROR("Error creating angle regulators");
    exit(20);
  }

  /* altitude regulator */
  alt_reg_ctx = create_altitude_regulator(2.0, 1.1, 3.3, 38.8672, 1.0, DIFF_FAST);
  if(0 == alt_reg_ctx) {
    LOG_ERROR("Error creating altitude regulator");
    exit(30);
  }

  while( !loop() );

  destroy_lin_vel_regulator(lvr_ctx_x);
  destroy_lin_vel_regulator(lvr_ctx_y);

  destroy_angle_regulator(ar_ctx_pitch);
  destroy_angle_regulator(ar_ctx_roll);
  destroy_angle_regulator(ar_ctx_yaw);

  destroy_altitude_regulator(alt_reg_ctx);

  return 0;
}
