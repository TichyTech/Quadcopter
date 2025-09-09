#include "position_controller.h"
#include "debug/debugging.h"

PosController::PosController(){
    vel_x_PID = PID(1, 0, 0.02, 0, 0.9, 0, "vel_x");
    vel_y_PID = PID(1, 0, 0.02, 0, 0.9, 0, "vel_y");
    vel_z_PID = PID(1, 0, 0.02, 0, 0.9, 0, "vel_z");

    pos_x_PID = PID(1, 0, 0, 0, 0.9, 0, "pos_x");
    pos_y_PID = PID(1, 0, 0, 0, 0.9, 0, "pos_y");
    pos_z_PID = PID(1, 0, 0, 0, 0.9, 0, "pos_z");

    last_pos_dif = {0,0,0};
    last_vel_diff = {0,0,0};
}


Vector3 PosController::process(Vector3 ref_pos_NWU, State state, float dt){
    float rad_yaw = TO_RAD*state.rpy(2);
    float yaw_cos = cos(rad_yaw);
    float yaw_sin = sin(rad_yaw);

    Vector3 pos_diff_NWU = ref_pos_NWU - state.pos;  // global position diff

    Vector3 pos_diff = {0, 0, pos_diff_NWU(2)};  // local position diff
    Matrix<2,2> rot_mat = {yaw_cos, yaw_sin,
                           -yaw_sin, yaw_cos};
    pos_diff.Submatrix<2,2>(0,0) = rot_mat * pos_diff_NWU.Submatrix<2,2>(0,0);
    last_pos_dif = pos_diff;

    Vector3 local_ref = state.pos + pos_diff; 
    Vector3 vel_ref = {0,0,0};
    Vector3 acc_ref = {0,0,0};
    acc_ref(0) = pos_x_PID.process(local_ref(0), state.pos(0), dt);
    acc_ref(1) = pos_y_PID.process(local_ref(1), state.pos(1), dt);
    acc_ref(2) = pos_z_PID.process(local_ref(2), state.pos(2), dt);
    // last_vel_diff = vel_ref - state.vel;

    // Vector3 acc_ref = {0,0,0};
    // acc_ref(0) = vel_x_PID.process(vel_ref(0), state.vel(0), dt);
    // acc_ref(1) = vel_y_PID.process(vel_ref(1), state.vel(1), dt);
    // acc_ref(2) = vel_z_PID.process(vel_ref(2), state.vel(2), dt);

    return acc_ref;
}

void PosController::update_PID_params(int axis, Parameters cfg){
  switch (axis){
    case 0:
      pos_x_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 1:
      pos_y_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 2:
      pos_z_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 3:
      vel_x_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 4:
      vel_y_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    case 5:
      vel_z_PID.set_PID_params(cfg.a, cfg.b, cfg.c, cfg.x, cfg.y);
      break;
    default: 
      break;
  }
}
