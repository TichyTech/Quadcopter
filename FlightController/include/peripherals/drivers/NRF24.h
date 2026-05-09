#ifndef NRF24_H
#define NRF24_H

#include <RF24.h>
#include <nRF24L01.h> 
#include "algebra.h"
#include "config.h"
#include <SPI.h>
#include "comm_structs.h"

const byte CMDADD[4] = "DRR";  // receive address
const byte STATADD[4] = "DRT";  // transmit address

class Communication{
  private:
    RF24 radio;
    SPIClass &_spi;
  public:
    uint16_t avg_diff;
    long last_ctrl_ms;  // last time we received control
    bool comm_timed_out;  // ctrl delay exceeded timeout threshold
    int16_t batt_telem_countdown;

    bool updated_config[static_cast<uint8_t>(PID_AXIS::COUNT)] = {};
    Parameters latest_configs[static_cast<uint8_t>(PID_AXIS::COUNT)];
    AttReference latest_control;

    Communication(SPIClass &spi);

    void setup_nrf();
    void push_config(const ConfigMessage& new_config);
    Parameters pop_config(uint8_t axis);
    AttReference update_commands(float initial_yaw);
    Message create_state_msg(const State &state, float battery, const uint8_t numSV, int avg_diff);
    Message create_ekf_msg(const State &state, const Measurements &m, float inn_mag);
    Message create_pos_msg(const Vector3 &pos_diff, const Vector3 &vel_diff, const Vector3 &acc_ref);
    Message create_attitude_msg(const AttReference &att_ref, const Vector3 &ang_err, const Vector3 &pid_err, const Vector3 &PID_outputs);
    Message create_quattitude_msg(const AttReference &att_ref, const Vector3 &axis, const Vector3 &omega_des, const Vector3 &tau, const Vector4 mps);
    void send_msg(Message msg);

};

#endif // NRF24_H

