#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "PID.h"
#include "filter.h"

class AttController{
    
	private:
		PID roll_rate_PID;
		PID pitch_rate_PID;
		PID yaw_rate_PID; 

		float roll_P;
		float pitch_P;
		float yaw_P;

		Vector4 motor_percentages;
	public:
		AttController();
    	Vector3 last_PID_outputs;
		AttReference last_reference;
		Vector3 last_ang_err;
		Vector3 last_pid_err;

		Vector4 process(AttReference att_ref, State current_state, float dt, float battery);
		Vector4 mix_motors(Vector3 forces, float throttle);
        Vector4 clamp_motor_jerk(Vector4 new_percentages, float dt);
        void update_PID_params(int axis, Parameters cfg);

};

#endif // ATTITUDE_CONTROLLER_H