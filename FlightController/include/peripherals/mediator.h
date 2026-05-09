#ifndef PERIPHERALS_MEDIATOR_H
#define PERIPHERALS_MEDIATOR_H

#include <Arduino.h>
#include "peripherals/drivers/NRF24.h"
#include "control/position_controller.h"
#include "control/quat_controller.h"

class Mediator{
    private:
        QuatController &quat_ctrl;
        PosController &pos_ctrl;
        Communication &comm;
    public:
        Mediator(QuatController &quat_ctrl, PosController &pos_ctrl, Communication &comm);
        void update_configs();
};

#endif // PERIPHERALS_MEDIATOR_H