#ifndef QUAT_CONTROLLER_H
#define QUAT_CONTROLLER_H

#include "control/PID.h"
#include "control/filter.h"


class QuatController{
    
	private:
  // PID gains
    Vector3 rate_P;  // P gain on attitude
    Vector3 k_p;  // P gain on angular velocity
    Vector3 k_d;  // D gain on angular velocity
    Vector3 k_i;  // I gain on angular velocity

    // additional parameters
    Vector3 torque_limit;
    float sum_max;  // max error sum magnitude
    float gyro_lpf;
    float d_gyro_lpf;
    float u_diff_limit;

    // memory variables for controller
    Vector3 last_gyro;  // last gyro reading for derivative
    Vector3 last_d_gyro;  // last derivative error for low-pass filtering

    // dynamic model of quadcopter
    Matrix4 torque_matrix;  // Mapping from motor settings [0,1] to body torques and thrust
    Matrix4 torque_matrix_inv;  // Mapping from torques and body thrust to motor settings
	public:
    Vector3 err_sum;  // sum of omega errors
    Vector3 smooth_gyro;  // smoothed gyro readings
    Vector3 last_axis;  // last attitude error axis
    Vector3 omega_des;  // desired angular velocity
    Vector3 tau;  // desired torques
		Vector4 m_p;  // last control

		QuatController();
 
		Vector4 process(Vector4 q_des, Vector4 q_est, Vector3 gyro, float throttle, float dt);
		Vector4 mix_motors(Vector3 tau, float throttle, float dt);
    Vector4 clamp_motor_jerk(Vector4 control, float dt);
    void update_gains(uint8_t axis, Parameters cfg);
};

#endif // QUAT_CONTROLLER_H
