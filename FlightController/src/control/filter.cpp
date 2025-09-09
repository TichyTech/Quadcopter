#include "filter.h"

#define DCM_REJ_K 0.002f

Matrix3 acc_mag2DCM(Measurements m){
  // Use current measurements to construct a direction cosine matrix [North, West, Up]
  Matrix3 DCM;
  DCM.Column(1) = normalize(skew(m.acc_vec)*m.mag_vec);  // West vector
  DCM.Column(0) = normalize(skew(DCM.Column(1)) * m.acc_vec);  // North vector
  DCM.Column(2) = normalize(m.acc_vec);  // Up vector
  return DCM;  
}

/**
 * Complementary filter implementation
 */
Matrix3 update_DCM_CF(Matrix3 current_DCM, Measurements m){
  Vector3 gyro_omega = -m.gyro_vec*TO_RAD*m.integration_period;  // Gyro measured rotation
  // compute correcting rotations in axis angle form

  float acc_norm = norm(m.acc_vec);
  Vector3 joint_omega = {0,0,0};

  if (acc_norm > 0.7 & acc_norm < 1.3){
    Matrix3 measured_DCM = acc_mag2DCM(m);  // first, build a DCM from the measured acc and mag vectors
    Vector3 acc_omega = get_omega(current_DCM.Column(2), measured_DCM.Column(2)); // Pitch and Roll correction 
    Vector3 mag_omega = get_omega(current_DCM.Column(0), measured_DCM.Column(0)); // Yaw correction
    mag_omega = current_DCM.Column(2) * dot(mag_omega, current_DCM.Column(2));  // project the mag compensation into Z axis
    // For small enough rotations, we can add the axis angle representation together, since infinitesimal rotations commute
    joint_omega = (acc_omega + mag_omega)*(1 - CF_RATIO) + gyro_omega*CF_RATIO;  // complementary filter in axis-angle domain
    printVec3(acc_omega, 2);
    Serial.println();
  }
  else{
    joint_omega = gyro_omega;  // if acceleration is out of range, use only gyro
  }

  Matrix3 updated_DCM = current_DCM;
  float joint_mag = norm(joint_omega);
  if (joint_mag > 1e-6){
    updated_DCM = rodriguez(joint_omega/joint_mag, joint_mag)*current_DCM;  // update using rotation matrix obtained from the combined axis angle
    if ( !(0.99 < det(updated_DCM) < 1.01) ) updated_DCM = normalize_matrix(updated_DCM);  // normalize matrix if needed
  }
  
  return updated_DCM; 
}

/** 
 * My implementation of a predict-correct DCM filter
 */
Matrix3 update_DCM_rejection(Matrix3 current_DCM, Measurements m){
  // First, predict, how DCM changes due to gyro measurement
  Vector3 gyro_omega = -m.gyro_vec*TO_RAD*m.integration_period;  // Gyro measured rotation

  Vector3 accmag_omega = {0,0,0};
  float acc_norm = norm(m.acc_vec);
  static float acc_timer = 0;  // timer for acceleration rejection
  if (acc_norm > 0.9 & acc_norm < 1.1){
    acc_timer = acc_timer + m.integration_period;
    if (acc_timer > 0.1){  // at least 100 ms in range
      Matrix3 measured_DCM = acc_mag2DCM(m);  // first, build a DCM from the measured acc and mag vectors
      accmag_omega = get_omega(current_DCM.Column(2), measured_DCM.Column(2)); // Pitch and Roll correction 

      float mag_norm = norm(m.mag_vec);
      if (mag_norm > 0.22 & mag_norm < 0.67){
        Vector3 mag_omega = get_omega(current_DCM.Column(1), measured_DCM.Column(1)); // Yaw correction
        accmag_omega = accmag_omega + mag_omega;
      }
    }
  }
  else acc_timer = 0;

  Vector3 joint_omega = accmag_omega*DCM_REJ_K + gyro_omega;
  float total_angle = norm(joint_omega);
  Matrix3 updated_DCM = current_DCM;
  if (total_angle > 1e-6){
    updated_DCM = rodriguez(joint_omega/total_angle, total_angle)*current_DCM;
    if ( !(0.99 < det(updated_DCM) < 1.01) ) updated_DCM = normalize_matrix(updated_DCM);  // normalize matrix for numerical stability
  }
  
  return updated_DCM; 
}

Vector3 get_omega(Vector3 a, Vector3 b){  // c = angle(a,b)*(a x b)/|a x b|
  Vector3 c = skew(a) * b;  // rotation axis
  float sine = norm(c);  
  if (sine < 0.01) return zero_3vector;  // for small values of sine, sin(x) ~ x, 0.01 rad ~ 0.5 degree (negligible)
  float cosine = dot(a,b);
  float angle = atan2(sine, cosine);  // rotation angle
  return c * (1/sine) * angle;  // Theta * e
}

Vector3 DCM2RPY(Matrix3 D){  // direction cosine matrix to roll, pitch and yaw angles
  Vector3 RPY;
  RPY(0) =  TO_DEG*atan2(D(1,2), D(2,2)); 
  RPY(1) = -TO_DEG*atan2(D(0,2), sqrt(D(0,1)*D(0,1) + D(0,0)*D(0,0)) );
  RPY(2) =  TO_DEG*atan2(D(0,1), D(0,0));
  return RPY;
}
