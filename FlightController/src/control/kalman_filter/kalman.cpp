#include "kalman.h"

// constants for the EKF
#define QUAT_VAR_INIT 0.01f
#define BIAS_VAR_INIT 0.00001f
#define STEADY_BIAS_VAR_MULT 10
#define BIAS_VAR 0.00001f  // [rad/s]
#define GYRO_VAR 0.000015f 
const Vector3 ACC_VAR_BASE = {0.05, 0.05, 0.05};
const Vector3 MAG_VAR_BASE = {1, 1, 1};

const Vector3 mag_bias_quad = {-0.737,0.444,0.583};  // quadratic coefficient for magnetic bias w.r.t. throttle

#define MAG_INC 1.13f  // measured local magnetic inclination [rad]

// definitions for steady state detection
#define THRESHOLD (3.0f)
#define GYRO_TIMEOUT (300)  // ms 
#define ACC_TIMEOUT (40)  // ms

KalmanFilter::KalmanFilter(){
  q = {1,0,0,0};
  b = {0,0,0};

  // init variance matrices
  P.Fill(0);
  Q.Fill(0);
  // Set initial state variance
  P.Submatrix<4,4>(0,0) = I_4 * QUAT_VAR_INIT;
  P.Submatrix<3,3>(4,4) = I_3 * BIAS_VAR_INIT;

  // gyro and acc estimated using measurement, magnetometer guessed
  R_acc = diag_matrix(ACC_VAR_BASE);
  R_mag = diag_matrix(MAG_VAR_BASE);

  a_w = {0,0,1};
  // m_w = {cos(MAG_INC), 0, -sin(MAG_INC)};
  m_w = {1,0,0};  // here we need to remove magnetic component parallel to acc

  gyro_timer = 0;
  acc_timer = 0;

  last_inn_mag = 0.0f;
}

/** 
 * Init state quaternion using a DCM 
 */
void KalmanFilter::init_quat(Matrix3 DCM){
  q = {0.5f*sqrt(trace(DCM) + 1),
       0.5f*sgn(DCM(2,1) - DCM(1,2))*sqrt(DCM(0,0) - DCM(1,1) - DCM(2,2) + 1),
       0.5f*sgn(DCM(0,2) - DCM(2,0))*sqrt(DCM(1,1) - DCM(2,2) - DCM(0,0) + 1),
       0.5f*sgn(DCM(1,0) - DCM(0,1))*sqrt(DCM(2,2) - DCM(0,0) - DCM(1,1) + 1)};
}

/** 
 * Track gyro magnitude to detect fast movement (gyro_vec [deg], dt [sec])
 */
void KalmanFilter::track_gyro(Vector3 w, float dt){
  if ((abs(w(0)) > THRESHOLD) || (abs(w(1)) > THRESHOLD) || (abs(w(2)) > THRESHOLD)){
    gyro_timer = 0;  // reset timer
  }
  else{
    gyro_timer += dt;
  }
}

/** 
 * Track acc magnitude to detect fast movement (a [g], dt [sec])
 */
void KalmanFilter::track_acc(Vector3 a, float dt){
  float a_norm = norm(a);
  if ((0.98f < a_norm) && (a_norm < 1.02f)){  // acceleration in valid range
    acc_timer += dt;
  }
  else acc_timer = 0;  // reset timer
}

/**
 * Whether or not the drone is steady enough
 */
bool KalmanFilter::gyro_steady(){
  return (1000.0f*gyro_timer > GYRO_TIMEOUT);  // if gyro is steady for at least GYRO_TIMEOUT
}

/**
 * Whether or not the drone is steady enough
 */
bool KalmanFilter::acc_steady(){
  return (1000.0f*acc_timer > ACC_TIMEOUT);  
}

/**
 * remove magnetic bias from mag measurement
 */
Vector3 KalmanFilter::remove_mag_bias(Vector3 mag, float throttle){
  static float lp_th = throttle;
  lp_th = 0.9f*lp_th + 0.1f*throttle;  // low pass filtered throttle

  Vector3 mg_bias = lp_th*lp_th*mag_bias_quad;
  return mag - mg_bias;
}


/** 
 * Use rotation model to predict quaternion attitude using gyro and time 
 */
Vector4 KalmanFilter::predict(Vector3 gyro_vec, float dt){
  // Vector3 w = TO_RAD*gyro_vec - b;
  Vector3 w = TO_RAD*gyro_vec;
  track_gyro(gyro_vec, dt);  // update gyro tracking

  // dq(t+1)/dq
  Matrix<4,4> quat_mult = 
  {0,   -w(0), -w(1), -w(2),
  w(0),  0,    -w(2),  w(1),
  w(1),  w(2),  0   , -w(0),
  w(2), -w(1),  w(0),  0};
  Matrix<4,4> F11 = I_4 - (0.5f*dt)*quat_mult;  // using small angle approximation

  // The following three lines implement the F11 matrix without the small angle approximation
  // float w_norm = norm(w);
  // float half_angle = 0.5f * w_norm * dt;
  // Matrix<4,4> F11 = cos(half_angle) * I_4 - sin(half_angle) * quat_mult * (1/w_norm);

  // dq(t+1)/db (*2/dt)
  Matrix<4,3> F12 = 
  {-q(1), -q(2), -q(3), 
    q(0),  q(3), -q(2),
   -q(3),  q(0),  q(1),
    q(2), -q(1),  q(0)};
  Zeros<3,4> Zeros3_4;  // db(t+1)/dq
  Matrix<7,7> F = (F11 || (0.5f*dt)*F12) && (Zeros3_4 || I_3);  // construct the F matrix

  Q.Submatrix<4,4>(0,0) = (0.25f*dt*dt*GYRO_VAR)*F12*~F12;  // update quat variance using dq/dw
  float bias_var = BIAS_VAR*dt;
  if (gyro_steady()) bias_var = (STEADY_BIAS_VAR_MULT*BIAS_VAR)*dt;  // increase the variance a bit 
  Q.Submatrix<3,3>(4,4) = I_3 * bias_var;  // init small process variance for bias
  Q = Q + I_7*(1e-5f*dt);  // baseline process noise

  q = F11*q;  // predict quaternion
  P = F*P*(~F) + Q;  // predict covariance

  q = normalize(q);  // renormalize quaternion
  return q;
}


/**
 * Fuse magnetometer measurement via EKF
 */
Vector4 KalmanFilter::fuse_mag(Vector3 m){
  // expected measurement h(x) = [R(q)a_w]
  Matrix3 Rq = quat2R(q);
  Matrix<3,1> hx = Rq*m_w;

  // actual measurement z = [m] 
  Matrix<3,1> z = m; 

  // prepare zero matrix for dh(x)p/db=0, since we do not measure the bias
  Zeros<3,3> Zeros3_3;
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  // H = dh(x)/dx using derivative of d(R(q)x)/dq
  Matrix<3,7> H = 2.0f*((q0*m_w + v_x*m_w) || (I_3*dot(v,m_w) + v*~m_w -q0*skew(m_w) -m_w*~v)) || Zeros3_3;

  // Compute the covariance matrix of the measurements and prepare Cholesky decomp
  Matrix<3,3> S = H*P*~H + R_mag;
  Matrix<3,3> S_cp = S;
  CholeskyDecomposition LLT = CholeskyDecompose(S_cp);  // Decompose the matrix LL^T

  if (!LLT.positive_definite){  // rather return right away and crash the drone :D
    return inf_4vector;
  }

  // Kalman gain multiplied by S
  Matrix<7, 3> KS = P*~H;  // K = PH^TS^-1, KS = PH^T
  // S\H computation
  Matrix<3,7> SinvH;
  for (int i=0; i<7; i++){  // solve S^-1 * H(:,i) for each column of H
    SinvH.Submatrix<3,1>(0,i) = CholeskySolve(LLT,H.Submatrix<3,1>(0,i));  
  }

  // Kalman Update of the state and state covariance
  Matrix<3,1> error = z - hx;
  Matrix<7,1> innovation = KS * CholeskySolve(LLT, error);
  P = (I_7 - KS*SinvH)*P;

  // Save result back to our state vectors
  Vector4 inn_q = innovation.Submatrix<4,1>(0,0);  // quaternion innovation
  Vector3 inn_b = innovation.Submatrix<3,1>(4,0);  // bias innovation

  inn_q = clamp_innovation(inn_q, q);  // restrict quat innovations to small, orthogonal
  q = normalize(q + inn_q);  // add innovation and renormalize
  b = b + 0.05f*inn_b;  // always use smaller step for magnetometer bias estimation

  return q;
}

/**
 * Fuse accelerometer measurement via EKF
 */
Vector4 KalmanFilter::fuse_acc(Vector3 a){
  // expected measurement h(x) = [R(q)a_w]
  Matrix3 Rq = quat2R(q);
  Matrix<3,1> hx = Rq*a_w;

  // actual measurement z = [a] 
  Matrix<3,1> z = a; 

  // prepare zero matrix for dh(x)/db=0, since we do not measure the bias
  Zeros<3,3> Zeros3_3;
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  // H = dh(x)/dx using derivative of d(R(q)x)/dq
  Matrix<3,7> H = 2.0f*((q0*a_w + v_x*a_w) || (I_3*dot(v,a_w) + v*~a_w -q0*skew(a_w) -a_w*~v)) || Zeros3_3;
  // Compute the covariance matrix of the measurements and prepare Cholesky decomp
  Matrix<3,3> S = H*P*~H;  
  // if (acc_steady()) S = S + 0.01f*R_acc;  // reduce variance of accelerometer if steady
  // else S = S + R_acc;
  S = S + R_acc;
  Matrix<3,3> S_cp = S;
  CholeskyDecomposition LLT = CholeskyDecompose(S_cp);  // Decompose the matrix LL^T

  if (!LLT.positive_definite){  // rather return right away and crash the drone :D
    return inf_4vector;
  }

  // Kalman gain multiplied by S
  Matrix<7, 3> KS = P*~H;  // K = PH^TS^-1, KS = PH^T
  // S\H computation
  Matrix<3,7> SinvH;
  for (int i=0; i<7; i++){  // solve S^-1 * H(:,i) for each column of H
    SinvH.Submatrix<3,1>(0,i) = CholeskySolve(LLT,H.Submatrix<3,1>(0,i));  
  }

  // Kalman Update of the state and state covariance
  Matrix<3,1> error = z - hx;
  Matrix<3,1> Sinverr = CholeskySolve(LLT, error);
  Matrix<7,1> innovation = KS * Sinverr;  // inn = P H^T S^-1 (z - h(x))
  P = (I_7 - KS*SinvH)*P;

  // Save result back to our state vectors
  Vector4 inn_q = innovation.Submatrix<4,1>(0,0);  // quaternion innovation
  Vector3 inn_b = innovation.Submatrix<3,1>(4,0);  // bias innovation

  inn_q = clamp_innovation(inn_q, q);  // restrict quat innovations to small, orthogonal
  q = normalize(q + inn_q);  // add innovation and renormalize
  if(acc_steady()) b = b + inn_b;  // if acc not steady, use smaller step
  else b = b + 0.05f*inn_b;

  last_inn_mag = norm(inn_q);

  return q;
}

float KalmanFilter::get_P_max()
{
    return max_norm(P);
}

float KalmanFilter::get_P_min()
{
    return min_norm(P);
}

/**
 * Clamp the state variance matrix to reasonable range
 */
float KalmanFilter::clamp_variance()
{
  float max_entry = max_norm(P);
  if (max_entry > 2.0f) {
    float mult = 2.0f/max_entry;  // variance 2.0 is already too much probably
    P = mult*P;
  }
  return max_entry;
}

/**
 * Clamp the quaternion innovation vector to be tangent to unit sphere and less than 0.5 in magnitude
 */
Vector4 KalmanFilter::clamp_innovation(Vector4 inn_q, Vector4 q)
{
    Vector4 t_inn_q = inn_q - dot(inn_q, q)*q;  // remove component parallel to q
    float t_inn_q_norm = norm(t_inn_q);
    if (t_inn_q_norm > 0.5) {
      t_inn_q = t_inn_q * (0.5f / t_inn_q_norm);  // normalize to length 0.5
    }
    return t_inn_q;
}