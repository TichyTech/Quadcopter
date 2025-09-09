#include "mediator.h"

Mediator::Mediator(AttController &att_ctrl, QuatController &quat_ctrl, PosController &pos_ctrl, Communication &comm) :
    att_ctrl(att_ctrl), 
    quat_ctrl(quat_ctrl),
    pos_ctrl(pos_ctrl),
    comm(comm)
    {}

void Mediator::update_configs()
{

    for (int i = 0; i < static_cast<uint8_t>(PID_AXIS::COUNT); i++){  // iterate over all PID axes
        if (!comm.updated_config[i]) continue;
        PID_AXIS axis = static_cast<PID_AXIS>(i);
        switch (axis) {
            case PID_AXIS::ROLL:
                att_ctrl.update_PID_params(0, comm.pop_config(i));
                break;
            case PID_AXIS::PITCH:
                att_ctrl.update_PID_params(1, comm.pop_config(i));
                break;
            case PID_AXIS::YAW:
                att_ctrl.update_PID_params(2, comm.pop_config(i));
                break;
            case PID_AXIS::XY:
                pos_ctrl.update_PID_params(0, comm.pop_config(i));
                pos_ctrl.update_PID_params(1, comm.pop_config(i));
                break;
            case PID_AXIS::Z:
                pos_ctrl.update_PID_params(2, comm.pop_config(i));
                break;
            case PID_AXIS::XYVEL:
                pos_ctrl.update_PID_params(3, comm.pop_config(i));
                pos_ctrl.update_PID_params(4, comm.pop_config(i));
                break;
            case PID_AXIS::ZVEL:
                pos_ctrl.update_PID_params(5, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_RATE:
                quat_ctrl.update_gains(0, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_P:
                quat_ctrl.update_gains(1, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_I:
                quat_ctrl.update_gains(2, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_D:
                quat_ctrl.update_gains(3, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_TL:
                quat_ctrl.update_gains(4, comm.pop_config(i));
                break;
            case PID_AXIS::QUAT_LPC:
                quat_ctrl.update_gains(5, comm.pop_config(i));
                break;
            default:
                break;
        }
    }
}
