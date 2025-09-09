#ifndef PID_H
#define PID_H

#include "config.h"
#include "definitions.h"

class PID{
  private:
    float prev_error;
    float prev_diff_error;
    float sum_error;
    float differr_deadband;
    float P, I, D, saturation, LPc;
  public:
    String nm = "";
    float last_output;

    PID(){};
    PID(float setP, float setI, float setD, float sat, float setLPc, float set_differr_deadband, String setName);
    void set_PID_params(float setP, float setI, float setD, float sat, float setLPc);
    float process(float reference, float measurement, float integration_period);
};

#endif