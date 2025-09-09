#ifndef PERIPHERALS_MEDIATOR_H
#define PERIPHERALS_MEDIATOR_H

#include <Arduino.h>
#include "peripherals/comms/NRF24.h"
#include "control/attitude_controller.h"
#include "control/position_controller.h"
#include "control/quat_controller/quat_controller.h"

class Mediator{
    private:
        AttController &att_ctrl;
        PosController &pos_ctrl;
        QuatController &quat_ctrl;
        Communication &comm;
    public:
        Mediator(AttController &att_ctrl, QuatController &quat_ctrl, PosController &pos_ctrl, Communication &comm);
        void update_configs();
};

#endif // PERIPHERALS_MEDIATOR_H