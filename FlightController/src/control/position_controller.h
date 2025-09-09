#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "algebra.h"
#include "PID.h"

class PosController{
    
	private:
		PID vel_x_PID;
		PID vel_y_PID;
		PID vel_z_PID;

		PID pos_x_PID;
		PID pos_y_PID;
		PID pos_z_PID;

	public:
		Vector3 last_pos_dif;
		Vector3 last_vel_diff;

        PosController();
        Vector3 process(Vector3 ref_pos, State state, float dt);
		void update_PID_params(int axis, Parameters cfg);
};

#endif // POSITION_CONTROLLER_H