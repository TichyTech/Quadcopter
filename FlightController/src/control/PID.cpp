#include "PID.h"

PID::PID(float setP, float setI, float setD, float sat, float setLPc, float set_differr_deadband, String setName){
	float prev_error = 0;
	float prev_diff_error = 0;
	float sum_error = 0;

	P = setP;
	I = setI;
	D = setD;
	saturation = sat;
	LPc = setLPc; // derivative low pass filter coefficient
	differr_deadband = set_differr_deadband;
	nm = setName;
  last_output = 0;
};

void PID::set_PID_params(float setP, float setI, float setD, float sat, float setLPc){
	P = setP;
	I = setI;
	D = setD;
	saturation = sat;
	LPc = setLPc; // derivative low pass filter coefficient
	if (DEBUG) Serial.println(nm + " PID parameters set " + String(P, 2) + " " + String(LPc, 2));
}
	
float PID::process(float reference, float measurement, float dt){
	float error = reference - measurement;

	float diff_error = (error - prev_error) / dt; // derivation (around 10 with very slow movements)
	if (abs(diff_error) <= differr_deadband)
		diff_error = 0;																						 // ignore noise in derivative
	diff_error = LPc * diff_error + (1 - LPc) * prev_diff_error; // Low pass filter on derivative to reduce noise spikes

	prev_diff_error = diff_error;
	prev_error = error;

	sum_error += error * dt;									 // integration
	sum_error = constrain(sum_error, -saturation, saturation); // anti windup

  last_output = P * error + I * sum_error + D * diff_error;

	return last_output;
}