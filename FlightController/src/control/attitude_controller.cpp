#include "attitude_controller.h"

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

// useful definitions
const Vector4 roll_action = {1,-1,-1,1};
const Vector4 pitch_action = {-1,-1,1,1};
const Vector4 yaw_action = {1,-1,1,-1};
const Vector4 thrust_action = {1,1,1,1};

AttController::AttController(){
  roll_rate_PID = PID(0.8, 3, 0.03, 3, 0.3, 40, "roll");
  pitch_rate_PID = PID(0.8, 3, 0.03, 3, 0.3, 40, "pitch");
  yaw_rate_PID = PID(0.8, 3, 0.02, 3, 0.33, 40, "yaw"); 

  // For a linear system, this would yield a time constant tau = 1/P
  roll_P = 3;
  pitch_P = 3;
  yaw_P = 0.5;

  motor_percentages = {0,0,0,0};
  last_PID_outputs = {0,0,0};
}

/**
 * update the PID parameters using a new config struct
 */
void AttController::update_PID_params(int axis, Parameters cfg){
  switch (axis){
    case 0:
      roll_rate_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 1:
      pitch_rate_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 2:
      yaw_rate_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    default: 
      break;
  }
}

Vector4 AttController::process(AttReference att_ref, State current_state, float dt, float battery){
  // using the current state and setpoint state, compute the PID action
  Vector3 forces = {0,0,0};  // rpy forces to apply 
  Vector3 des_rates = {0,0,0};

  last_reference = att_ref;  // store latest reference
  Vector3 ang_err = {att_ref.roll - current_state.rpy(0), att_ref.pitch - current_state.rpy(1), constrain_angle(att_ref.yaw - current_state.rpy(2))};
  last_ang_err = ang_err;

  // Proportional controller on RPY
  des_rates(0) += roll_P*ang_err(0);  
  des_rates(1) += pitch_P*ang_err(1);
  des_rates(2) += yaw_P*ang_err(2);

  des_rates(0) = constrain(des_rates(0), -150, 150);
  des_rates(1) = constrain(des_rates(1), -150, 150);
  des_rates(2) = constrain(des_rates(2), -90, 90);

  last_pid_err = des_rates - current_state.omega;

  forces(0) = roll_rate_PID.process( des_rates(0), current_state.omega(0), dt);
  forces(1) = pitch_rate_PID.process(des_rates(1), current_state.omega(1), dt);
  forces(2) = yaw_rate_PID.process(  des_rates(2), current_state.omega(2), dt);

  last_PID_outputs = forces;  // for telemetry

  forces(0) = constrain(forces(0), -60, 60);
  forces(1) = constrain(forces(1), -60, 60);
  forces(2) = constrain(forces(2), -30, 30);

  Vector4 new_percentages = mix_motors(forces, att_ref.throttle);
  // if (battery > 10) motor_percentages *= (IDEAL_VOLTAGE*IDEAL_VOLTAGE)/(battery*battery);  // battery charge compensation
  motor_percentages = clamp_motor_jerk(new_percentages, dt);
  
  return motor_percentages;
}

/**
 * Transform from local forces and throttle to motor pecentages
 */
Vector4 AttController::mix_motors(Vector3 forces, float throttle){

  Vector4 mp = {0, 0, 0, 0};
  mp += thrust_action * throttle;  // compensate gravity
  mp += roll_action * forces(0) * (1.0f/MOTOR_FORCE);  
  mp += pitch_action * forces(1) * (1.0f/MOTOR_FORCE); 
  mp += yaw_action * forces(2) * (1.0f/MOTOR_FORCE);

  mp(0) = constrain(mp(0), 0, 1);
  mp(1) = constrain(mp(1), 0, 1);
  mp(2) = constrain(mp(2), 0, 1);
  mp(3) = constrain(mp(3), 0, 1);

  return mp;
}


Vector4 AttController::clamp_motor_jerk(Vector4 new_percentages, float dt){
  Vector4 d_p = (new_percentages - motor_percentages)/dt;  //normalize by time constant
  // clamp to about +-10 full-scale per second (or 1% per millisecond)
  const float change_limit = 2;
  d_p(0) = constrain(d_p(0), -change_limit, change_limit);
  d_p(1) = constrain(d_p(1), -change_limit, change_limit);
  d_p(2) = constrain(d_p(2), -change_limit, change_limit);
  d_p(3) = constrain(d_p(3), -change_limit, change_limit);
  motor_percentages = motor_percentages + d_p*dt;
  return motor_percentages;
}