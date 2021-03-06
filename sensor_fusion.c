#include "sensor_fusion.h"

#include "cpu_cycles.h"
#include "MadgwickAHRS.h"

static const vector_double_3d gravity_acc_I = {0.0, 0.0, -1.0}; /* gravity acceleration vector in g relative to inertial frame of reference */
static long long last_ts = 0;

static matrix_double_3x3 quat_to_matrix(double q0, double q1, double q2, double q3) {
  matrix_double_3x3 res;
  double sq_q1 = 2 * q1 * q1;
  double sq_q2 = 2 * q2 * q2;
  double sq_q3 = 2 * q3 * q3;
  double q1_q2 = 2 * q1 * q2;
  double q3_q0 = 2 * q3 * q0;
  double q1_q3 = 2 * q1 * q3;
  double q2_q0 = 2 * q2 * q0;
  double q2_q3 = 2 * q2 * q3;
  double q1_q0 = 2 * q1 * q0;

  /* maybe wrong column/row order?
  http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/ suggests
  1 - 2*qy2 - 2*qz2  2*qx*qy - 2*qz*qw  2*qx*qz + 2*qy*qw
  2*qx*qy + 2*qz*qw  1 - 2*qx2 - 2*qz2  2*qy*qz - 2*qx*qw
  2*qx*qz - 2*qy*qw	 2*qy*qz + 2*qx*qw	1 - 2*qx2 - 2*qy2
  */
  res.cols[0].x = 1 - sq_q2 - sq_q3;
  res.cols[0].y = q1_q2 - q3_q0;
  res.cols[0].z = q1_q3 + q2_q0;
  res.cols[1].x = q1_q2 + q3_q0;
  res.cols[1].y = 1 - sq_q1 - sq_q3;
  res.cols[1].z = q2_q3 - q1_q0;
  res.cols[2].x = q1_q3 - q2_q0;
  res.cols[2].y = q2_q3 + q1_q0;
  res.cols[2].z = 1 - sq_q1 - sq_q2;

  return res;
}

void fuse_sensor_data(sensor_data* data, fused_sensor_data* out_fused_data)
{
  const long long cur_ts = data->acc_data.ts; /* use accelerometer ts as the base one */

  /* update attitude */
  if(0 != last_ts) {
    /* update attitude starting with the second call */
    MadgwickAHRSupdate(-data->avel_data.data.x, -data->avel_data.data.y, data->avel_data.data.z,
                        data->acc_data.data.x, data->acc_data.data.y, data->acc_data.data.z,
                        data->mag_data.data.x, data->mag_data.data.y, data->mag_data.data.z,
                        cycles_to_s(cur_ts - last_ts));
  }
  last_ts = cur_ts;

  matrix_double_3x3 rot_matrix = quat_to_matrix(q0, q1, q2, q3);
  /*  taken from android frameworks OrientationSensor.cpp
      vec3_t g;
      const float rad2deg = 180 / M_PI;
      const mat33_t R(mSensorFusion.getRotationMatrix());
      g[0] = atan2f(-R[1][0], R[0][0])    * rad2deg;
      g[1] = atan2f(-R[2][1], R[2][2])    * rad2deg;
      g[2] = asinf ( R[2][0])             * rad2deg;
      if (g[0] < 0)
      g[0] += 360;

      *outEvent = event;
      outEvent->orientation.azimuth = g.x;
      outEvent->orientation.pitch   = g.y;
      outEvent->orientation.roll    = g.z;


      other info:
      heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)  yaw
      attitude = asin(2*qx*qy + 2*qz*qw)                    pitch
      bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)     roll

      except when qx*qy + qz*qw = 0.5 (north pole)
      which gives:
      heading = 2 * atan2(x,w)
      bank = 0
      and when qx*qy + qz*qw = -0.5 (south pole)
      which gives:
      heading = -2 * atan2(x,w)
      bank = 0
  */

  /* minus to correspond to our rotary axis directions, see axis.jpg */
  out_fused_data->attitude.yaw    = atan2( rot_matrix.cols[1].x, rot_matrix.cols[0].x );
  out_fused_data->attitude.roll  = atan2( -rot_matrix.cols[2].y, rot_matrix.cols[2].z );
  out_fused_data->attitude.pitch   = asin( rot_matrix.cols[2].x );

  /* calculate gravity acceleration as 1g vector pointing -z */
  vector_double_3d gravity_acc = rot_matrix.cols[2]; /* ...? */

  /* calculate linear acceleration as total acceleration - gravity acceleration */
  out_fused_data->lin_acc.x = data->acc_data.data.x - gravity_acc.x;
  out_fused_data->lin_acc.y = data->acc_data.data.y - gravity_acc.y;
  out_fused_data->lin_acc.z = data->acc_data.data.z - gravity_acc.z;

  /* copy angular velocities and altitude */
  out_fused_data->altitude = data->altitude.val;
  out_fused_data->avel = data->avel_data.data;
  out_fused_data->ts = cur_ts;
  out_fused_data->time_s = cycles_to_s(cur_ts);
}
