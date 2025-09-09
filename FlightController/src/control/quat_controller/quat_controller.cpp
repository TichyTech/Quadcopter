#include "quat_controller.h"

// MOTOR NUMBERING DIAGRAM
// TOP VIEW
// FRONT
//---------
//        X
//        ^
//        |
//
//     1     2 
//      \ | /
// <-Y   |||    
//      / | \ 
//     4     3

QuatController::QuatController(){
  rate_P = {2.0f, 2.0f, 0.2f};
  k_p = {0.2f, 0.2f, 0.05f};
  k_i = {0.08f, 0.08f, 0.03f};
  k_d = {0.002f, 0.002f, 0.001f};

  torque_limit = {1.0f, 1.0f, 0.1f};  // max torques around each axis
  gyro_lpf = 0.6f;  // gyro lpf
  d_gyro_lpf = 0.2f;  // derivative lpf
  u_diff_limit = 5.0f;  // max control change per second
  sum_max = 0.1f;  // max integral error

  // initialize memory variables
  last_gyro = {0,0,0};
  last_d_gyro = {0,0,0};

  err_sum = {0,0,0};
  smooth_gyro = {0,0,0};
  last_axis = {0,0,0};
  omega_des = {0,0,0};
  tau = {0,0,0};
  m_p = {0,0,0,0};
  
  // The Torque matrix can be decomposed as a product of a diagonal scaling matrix S and an orthonormal matrix U
  Vector4 t_v = {0.8, 0.8, 0.1, 6.0};  // estimated torques around roll, pitch, yaw and thrust
  Vector4 inv_t_v = {1/t_v(0), 1/t_v(1), 1/t_v(2), 1/t_v(3)};
  Matrix4 S = diag_matrix(t_v * 2.0f);
  Matrix4 S_inv = diag_matrix(inv_t_v * 0.5f);
  Matrix4 U = {0.5, -0.5, -0.5, 0.5,
              -0.5, -0.5, 0.5, 0.5,
              0.5, -0.5, 0.5, -0.5,
              0.5, 0.5, 0.5, 0.5}; 
  torque_matrix = S * U;
  torque_matrix_inv = (~U) * S_inv;
}

Vector4 QuatController::process(Vector4 q_des, Vector4 q_est, Vector3 gyro, float throttle, float dt){
  // compute attitude error and desired angular velocity
  Vector4 q_des_body = quat_mult(q_est, q_des);  // quaternion error in body frame
  Vector3 axis = q_des_body.Submatrix<3,1>(1,0);  // this is sin(theta/2)*u where u is the rotation axis
  last_axis = axis;
  float sine = norm(axis);
  float cosine = q_des_body(0);
  axis = axis / sine;  // normalized desired rotation axis
  float angle = 2 * atan2(sine, cosine);
  omega_des = product(axis, rate_P) * angle;  // omega is proportional to angle error

  smooth_gyro = (1 - gyro_lpf)*smooth_gyro + gyro_lpf*gyro*TO_RAD;  // lpf gyro
  Vector3 omega_err = omega_des - smooth_gyro;  // angular velocity error
  err_sum = err_sum + omega_err * dt;  // error integral
  err_sum = clamp(err_sum, -sum_max, sum_max);  // prevent integral windup
  Vector3 d_gyro = (smooth_gyro - last_gyro) / dt;  // gyro derivative
  d_gyro = (1 - d_gyro_lpf)*last_d_gyro + d_gyro_lpf*d_gyro;  // lpf derivative on gyro derivative
  // now save for next iteration
  last_gyro = smooth_gyro;
  last_d_gyro = d_gyro;
  
  tau = product(k_p, omega_err) + product(k_i, err_sum) - product(k_d, smooth_gyro);
  tau = clamp(tau, -torque_limit, torque_limit);  // clamp torques to limits

  return mix_motors(tau, 24.0f * throttle, dt);
}

Vector4 QuatController::mix_motors(Vector3 tau, float throttle, float dt){
  Vector4 f;  // construct force vector
  f.Submatrix<3,1>(0,0) = tau;
  f(3) = throttle;
  Vector4 u = torque_matrix_inv * f;  // solve for motor controls
  u = clamp(u, 0, 1);  // clamp controls to valid range

  Vector4 u_diff = (u - m_p) / dt;  // compute diff from previous control
  u_diff = clamp(u_diff, -u_diff_limit, u_diff_limit);  // clamp the diff and add back
  m_p = m_p + u_diff * dt;
  return m_p;
}

/**
 * update the internal controller variables via a config struct
 */
void QuatController::update_gains(uint8_t axis, Parameters cfg){
  switch (axis){
    case 0:
      rate_P = {cfg.a, cfg.b, cfg.c};
      break;
    case 1:
      k_p = {cfg.a, cfg.b, cfg.c};
      break;
    case 2: 
      k_i = {cfg.a, cfg.b, cfg.c};
      break;
    case 3:
      k_d = {cfg.a, cfg.b, cfg.c};
      break;
    case 4: 
      torque_limit = {cfg.a, cfg.b, cfg.c};
      break;
    case 5:
      gyro_lpf = cfg.a;
      d_gyro_lpf = cfg.b;
      sum_max = cfg.c;
      u_diff_limit = cfg.x;
      break;
    default: 
      break;
  }
}