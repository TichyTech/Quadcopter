#include "algebra.h"
#include "debug/debugging.h"
#include "config.h"

class KalmanFilter{
  private:
    Matrix<7,7> P;  // state variance
    Matrix<7,7> Q;  // process noise variance

    Matrix<3,3> R_mag;  // mag measurement variance
    Matrix<3,3> R_acc;  // acc measurement variance  

    Vector3 a_w;  // up vector in world frame
    Vector3 m_w;  // magnetic vector in world frame

    float var_w;  // gyro variance scale
    float mag_inc;  // magnetic inclination

    float gyro_timer;  // timer for gyro magnitude tracking
    float acc_timer;  // timer for accelerometer magnitude tracking

  public:
    Vector4 q;  // attitude quaternion of the world frame NWU
    Vector3 b;  // gyro bias estimate [rad]
    float last_inn_mag;

    KalmanFilter();
    void init_quat(Matrix3 DCM);  // initialize quaternion from a DCM matrix
    void track_gyro(Vector3 w, float dt);
    void track_acc(Vector3 a, float dt);
    float clamp_variance();
    bool gyro_steady();
    bool acc_steady();
    float get_P_max();
    float get_P_min();
    Vector3 remove_mag_bias(Vector3 mag, float th);
    Vector4 predict(Vector3 gyro_vec, float dt);
    Vector4 fuse_acc(Vector3 a);
    Vector4 fuse_mag(Vector3 m);
    Vector4 clamp_innovation(Vector4 inn_q, Vector4 q);
};
