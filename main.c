#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <unistd.h>

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
#include "sixaxis.h"

#include "MadgwickAHRS.h"

#define WRITE_STDOUT
//#define WRITE_FILE

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

sixaxis_button_state buttons = {};
sixaxis_axis_state axes = {};

#ifdef WRITE_FILE
FILE* out_csv = 0;
#endif

int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void on_button(uint8_t button, bool is_pressed) {
  if(!is_pressed) return;
  switch(button) {
    case SIXAXIS_UP:
      thr_correction.d_throttle_alt += 5.0;
      break;

  case SIXAXIS_DOWN:
    thr_correction.d_throttle_alt -= 5.0;
    break;

  case SIXAXIS_CIRCLE:
    thr_correction.d_throttle_alt = 0.0;
    break;

  default: return;
  }

  printf("Altitide throttle is %lf\n", thr_correction.d_throttle_alt);
}

int loop(void) {
  long long start = cpu_cycles();

  /* 1. Read operator commands. */
//  read_test_inputs(&input_cmd);
  int rc = sixaxis_update(&buttons, &axes);
  if(rc) return rc;

  /* 2. Read sensors. */
  rc = read_sensors(&raw_sensor_data);
  if(rc) return rc;

  /* 3. Calculate sensor fusion data */
  fuse_sensor_data(&raw_sensor_data, &fused_data);

  /* 4. Perform linear velocities regulation */
  double pitch_cmd_rad = 0.0;
  double roll_cmd_rad = 0.0;
  pitch_cmd_rad = regulate_lin_vel(lvr_ctx_x, input_cmd.cmd_vel_x, fused_data.lin_acc.x, fused_data.time_s);
  roll_cmd_rad = regulate_lin_vel(lvr_ctx_y, input_cmd.cmd_vel_y, fused_data.lin_acc.y, fused_data.time_s);

  /* 5. Perform roll regulation */
  thr_correction.d_throttle_roll = regulate_angle(ar_ctx_roll, roll_cmd_rad, fused_data.attitude.roll, fused_data.avel.x, fused_data.time_s);

  /* 6. Perform pitch regulation */
  thr_correction.d_throttle_pitch = regulate_angle(ar_ctx_pitch, pitch_cmd_rad, fused_data.attitude.pitch, fused_data.avel.y, fused_data.time_s);

  /* 7. Perform yaw regulation */
  thr_correction.d_throttle_yaw = regulate_angle(ar_ctx_yaw, input_cmd.cmd_yaw, fused_data.attitude.yaw, fused_data.avel.z, fused_data.time_s);

  /* 8. Perform altitude regulation */
//  thr_correction.d_throttle_alt = regulate_altitude(alt_reg_ctx, input_cmd.cmd_h, fused_data.altitude, fused_data.time_s);

  if(kbhit()) {
    double input = 0.0;
    rc = scanf("%lf", &input);
    printf("got input %lf\n", input);
    thr_correction.d_throttle_alt = input;

    input_cmd.cmd_yaw = fused_data.attitude.yaw;
  }

  if(thr_correction.d_throttle_alt < 1.0) { // turn rotors off
    thr_correction = (throttle_correction) {};
  }

  /* 9. Do control signal mixing */
  mix_throttles(&thr_correction, &motors_thr);

  /*10. Set motor controller PWMs */
  motors_thr.m_head /= 100.0;
  motors_thr.m_left /= 100.0;
  motors_thr.m_right /= 100.0;
  motors_thr.m_tail /= 100.0;
  rc = set_motors_throttles(motors_thr);
  if(rc) return rc;

  /*11. Send telemetry */
#ifdef WRITE_STDOUT
  static unsigned int last_output = 0;
  if(millis() - last_output > 250) {
//    printf("%f acc %f %f %f alt %f avel %f %f %f bar_temp %f guro_temp %f mag %f %f %f",
//      cycles_to_s( raw_sensor_data.acc_data.ts ),
//      raw_sensor_data.acc_data.data.x, raw_sensor_data.acc_data.data.y, raw_sensor_data.acc_data.data.z,
//      raw_sensor_data.altitude.val,
//      raw_sensor_data.avel_data.data.x, raw_sensor_data.avel_data.data.y, raw_sensor_data.avel_data.data.z,
//      raw_sensor_data.bmp085_temp.val,
//      raw_sensor_data.itg3200_temp.val,
//      raw_sensor_data.mag_data.data.x, raw_sensor_data.mag_data.data.y, raw_sensor_data.mag_data.data.z);

//    printf("\n%f %f %f %f\n", q0, q1, q2, q3);

    /* fused data */
    printf(" roll %f pitch %f yaw %f lin_acc %f %f %f",
           fused_data.attitude.roll, fused_data.attitude.pitch, fused_data.attitude.yaw,
           fused_data.lin_acc.x, fused_data.lin_acc.y, fused_data.lin_acc.z);

//    printf("\ncmd_vel %f %f pitch_cmd %f roll_cmd %f\n",
//           input_cmd.cmd_vel_x, input_cmd.cmd_vel_y,
//           pitch_cmd_rad, roll_cmd_rad);

    printf("\nth_roll %f th_pitch %f th_yaw %f th_alt %f m_head %f m_tail %f m_left %f m_right %f\n\n",
           thr_correction.d_throttle_roll, thr_correction.d_throttle_pitch, thr_correction.d_throttle_yaw, thr_correction.d_throttle_alt,
           motors_thr.m_head, motors_thr.m_tail, motors_thr.m_left, motors_thr.m_right);

    printf("\n");
    last_output = millis();
    fflush(stdout);
  }
#endif
#ifdef WRITE_FILE
//    fprintf(out_csv, "time,accx,accy,accz,alt,avelx,avely,avelz,bar_temp,guro_temp,magx,magy,magz,roll,pitch,yaw,
  //  lin_accx,lin_accy,lin_accz,pitch_cmd,roll_cmd,th_roll,th_pitch,th_yaw,th_alt,m_head,m_tail,m_left,m_right");
  fprintf(out_csv, "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",
          cycles_to_s( raw_sensor_data.acc_data.ts ),
          raw_sensor_data.acc_data.data.x, raw_sensor_data.acc_data.data.y, raw_sensor_data.acc_data.data.z,
          raw_sensor_data.altitude.val,
          raw_sensor_data.avel_data.data.x, raw_sensor_data.avel_data.data.y, raw_sensor_data.avel_data.data.z,
          raw_sensor_data.bmp085_temp.val,
          raw_sensor_data.itg3200_temp.val,
          raw_sensor_data.mag_data.data.x, raw_sensor_data.mag_data.data.y, raw_sensor_data.mag_data.data.z,
          fused_data.attitude.roll, fused_data.attitude.pitch, fused_data.attitude.yaw,
          fused_data.lin_acc.x, fused_data.lin_acc.y, fused_data.lin_acc.z,
          pitch_cmd_rad, roll_cmd_rad,
          thr_correction.d_throttle_roll, thr_correction.d_throttle_pitch, thr_correction.d_throttle_yaw, thr_correction.d_throttle_alt,
          motors_thr.m_head, motors_thr.m_tail, motors_thr.m_left, motors_thr.m_right);
#endif

  /*12. Perform loop frequency limiting */
  static const long long min_cycle_length_mcs = 5000; /* 200 Hz */
  const long long time_left_mcs = min_cycle_length_mcs - cycles_to_mcs( cpu_cycles() - start );
  if(time_left_mcs > 0) {
    delayMicroseconds(time_left_mcs);
  }

#ifdef WRITE_FILE
  static int counter = 0;
  if(++counter > 12000) return -1; /*stop after ~2 minutes*/
#endif

  return 0;
}

int main(/*int argc, const char* argv[]*/)
{
#ifdef WRITE_FILE
  out_csv = fopen("/tmp/out.csv", "w");
  if(!out_csv) {
      printf("%s", strerror(errno));
  }
  fprintf(out_csv, "time;accx;accy;accz;alt;avelx;avely;avelz;bar_temp;guro_temp;magx;magy;magz;roll;pitch;yaw;lin_accx;lin_accy;lin_accz;pitch_cmd;roll_cmd;th_roll;th_pitch;th_yaw;th_alt;m_head;m_tail;m_left;m_right\n");
#endif
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

  /* default address is 0100 0000 */
  rc = init_motors_controller(0x40, 0, 4, 8, 12, 0.08, 0.098, 60);
  if(rc) {
    LOG_ERROR_ERRNO("Error during PCA9685 init. Internal code: %d,", rc);
    exit(rc);
  }

  /* linear velocity regulators */
  lvr_ctx_x = create_lin_vel_regulator(0.32 * M_PI / 180.0, 0.1 * M_PI / 180.0, 12.0 * M_PI / 180.0);
  lvr_ctx_y = create_lin_vel_regulator(0.32 * M_PI / 180.0, 0.1 * M_PI / 180.0, 12.0 * M_PI / 180.0);

  if(0 == lvr_ctx_x || 0 == lvr_ctx_y) {
    LOG_ERROR("Error creating linear velocity regulators");
    exit(10);
  }

  /* angle regulators */
  double mult = 180.0/ M_PI / 5.0;
  double pr_Kp = 2.0 * mult;
  double pr_Ki = 1.1 * mult;
  double pr_Kd = 1.2 * mult;
  double pr_int_limit = 1.0;
  ar_ctx_pitch = create_angle_regulator(pr_Kp, pr_Ki, pr_Kd, pr_int_limit);
  ar_ctx_roll = create_angle_regulator(pr_Kp, pr_Ki, pr_Kd, pr_int_limit);
  ar_ctx_yaw = create_angle_regulator(4.0 * 180.0 / M_PI / 5.0, 0.5 * 180.0 / M_PI / 5.0, 3.5 * 180.0 / M_PI / 5.0, 1.0);
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

  /* init sixaxis */
  rc = sixaxis_init("/dev/input/js0", &on_button);
  if(rc) {
    exit(rc);
  }

  fflush(stdout);

  double input = 0.0;
  motors_thr.m_head = input;
  motors_thr.m_tail = input;
  motors_thr.m_left = input;
  motors_thr.m_right = input;

  set_motors_throttles(motors_thr);
  printf("Set throttle to 0, waiting 5 seconds\n");
  delay(5000);

  printf("Go!\n");

  while( !loop() );
//  for(;;) {
//    double input = 0.084;
//    rc = scanf("%lf", &input);
//    printf("got input %lf\n", input);
//    motors_thr.m_head = input;
//    motors_thr.m_tail = input;
//    motors_thr.m_left = input;
//    motors_thr.m_right = input;
//    set_motors_throttles(motors_thr);
//  }

  destroy_lin_vel_regulator(lvr_ctx_x);
  destroy_lin_vel_regulator(lvr_ctx_y);

  destroy_angle_regulator(ar_ctx_pitch);
  destroy_angle_regulator(ar_ctx_roll);
  destroy_angle_regulator(ar_ctx_yaw);

  destroy_altitude_regulator(alt_reg_ctx);

#ifdef WRITE_FILE
  fclose(out_csv);
#endif

  return 0;
}
