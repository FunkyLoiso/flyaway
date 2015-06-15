#include "sensor_fusion.h"

#include "cpu_cycles.h"
#include "MadgwickAHRS.h"

static const vector_double_3d gravity_acc_I = {0.0, 0.0, 1.0}; /* gravity acceleration vector in g relative to inertial frame of reference */

static long long last_ts = 0;
static vector_double_4d attitude = {1.0, 0.0, 0.0, 0.0};

static quat_to_matrix(vector_double_4d quat, matrix_double_3x3* out_matrix) {
  double q0 = quat.q0;
  double q1 = quat.q1;
  double q2 = quat.q2;
  double q3 = quat.q3;
  double sq_q1 = 2 * q1 * q1;
  double sq_q2 = 2 * q2 * q2;
  double sq_q3 = 2 * q3 * q3;
  double q1_q2 = 2 * q1 * q2;
  double q3_q0 = 2 * q3 * q0;
  double q1_q3 = 2 * q1 * q3;
  double q2_q0 = 2 * q2 * q0;
  double q2_q3 = 2 * q2 * q3;
  double q1_q0 = 2 * q1 * q0;
  out_matrix->cols[0].x = 1 - sq_q2 - sq_q3;
  out_matrix->cols[0].y = q1_q2 - q3_q0;
  out_matrix->cols[0].z = q1_q3 + q2_q0;
  out_matrix->cols[1].x = q1_q2 + q3_q0;
  out_matrix->cols[1].y = 1 - sq_q1 - sq_q3;
  out_matrix->cols[1].z = q2_q3 - q1_q0;
  out_matrix->cols[2].x = q1_q3 - q2_q0;
  out_matrix->cols[2].y = q2_q3 + q1_q0;
  out_matrix->cols[2].z = 1 - sq_q1 - sq_q2;
}

void fuse_sensor_data(sensor_data data, fused_sensor_data* out_fused_data)
{
  const long long cur_ts = data->acc_data.ts; /* use accelerometer ts as the base one */

  /* update attitude */
  if(0 != last_ts) {
    /* update attitude starting with the second call */
    MadgwickAHRSupdate( data->avel_data.data.x, data->avel_data.data.y, data->avel_data.data.z,
                        data->acc_data.data.x, data->acc_data.data.y, data->acc_data.data.z,
                        data->mag_data.data.x, data->mag_data.data.y, data->mag_data.data.z,
                        cycles_to_s(cur_ts - last_ts));
  }
  last_ts = cur_ts;
  attitude.q0 = q0;
  attitude.q1 = q1;
  attitude.q2 = q2;
  attitude.q3 = q3;

  out_fused_data->attitude = attitude;

  /* calculate gravity acceleration as 1g vector pointing -z */
  matrix_double_3x3 rot_matrix;
  quat_to_matrix(attitude, &rot_matrix);
  vector_double_3d gravity_acc = cross_product_double_3d(gravity_acc_I, rot_matrix.cols[2]);
  /* calculate linear acceleration as total acceleration - gravity acceleration */
  out_fused_data->lin_acc.x = data.acc_data.data.x - gravity_acc.x;
  out_fused_data->lin_acc.y = data.acc_data.data.y - gravity_acc.y;
  out_fused_data->lin_acc.z = data.acc_data.data.z - gravity_acc.z;

  /* copy angular velocities and altitude */
  out_fused_data->altitude = data->altitude.val;
  out_fused_data->avel = data->avel_data.data;
  out_fused_data->ts = cur_ts;
  out_fused_data->time_s = cycles_to_s(cur_ts);
}
