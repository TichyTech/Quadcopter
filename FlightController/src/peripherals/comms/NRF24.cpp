#include "NRF24.h"
#include "debug/debugging.h"

Communication::Communication(SPIClass &spi) : 
    _spi(spi)
  {
  radio = RF24(NRF_CE_PIN, SPI0_CS_NRF_PIN);
  comm_timed_out = 1;

  avg_diff = 0; // exponential average of control sequence diff

  latest_control = {0,0,0,0,0};  // roll, pitch, yaw, throttle, motors_on
}

void Communication::setup_nrf(){
  Serial.println("Setting up communication module");

  if (!radio.begin(&_spi, NRF_CE_PIN, SPI0_CS_NRF_PIN)) { // blocking 
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} 
  }
  
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.setChannel(3);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicAck();
  radio.setAutoAck(1);
  radio.setRetries(0, 4);

  radio.openWritingPipe(STATADD);
  radio.openReadingPipe(1, CMDADD); 
  radio.startListening();  
  last_ctrl_ms = 0;  
}

AttReference Communication::update_commands(float initial_yaw){  // receive latest command message with lost signal handling
  static uint32_t ctrl_sequence_num = 0;
  ControlMessage new_ctrl_msg;

  bool received_ctrl = false;  // received new control flag
  while (radio.available()) { // read latest 32 bytes
    Message received_msg;
    radio.read(&received_msg.data, sizeof(received_msg.data)); 
    if (received_msg.message_type() == MessageType::CONTROL){ // control message received
      if (DEBUG && DEBUG_COMM) Serial.println("Control received");
      last_ctrl_ms = millis(); 
      received_ctrl = true;
      new_ctrl_msg.deserialize(received_msg.data);
    }
    else if (received_msg.message_type() == MessageType::CONFIG){ // PID config message received
      ConfigMessage cfg_message;
      cfg_message.deserialize(received_msg.data);
      push_config(cfg_message);
      if (DEBUG){
        Serial.print("Config received: ");
        cfg_message.printTo(Serial);
        Serial.println();
      }
    }
  }
  
  unsigned long ctrl_delay = millis() - last_ctrl_ms;
  if (received_ctrl){  // received new control
    comm_timed_out = 0;
    latest_control.roll = new_ctrl_msg.roll;
    latest_control.pitch = new_ctrl_msg.pitch;
    latest_control.yaw = constrain_angle(initial_yaw + float(new_ctrl_msg.yaw_diff));
    latest_control.throttle = float(new_ctrl_msg.throttle);
    latest_control.motors_on = new_ctrl_msg.motors_on;
    // check sequence number on the packet and compare to last seen
    ctrl_sequence_num++;
    int32_t diff = new_ctrl_msg.sequence - ctrl_sequence_num;
    avg_diff = (9*avg_diff + diff)/10;  // exponential average
    if (ctrl_sequence_num != new_ctrl_msg.sequence) {
      if (DEBUG && DEBUG_COMM){
        Serial.print("Sequence mismatch: ");
        Serial.print(new_ctrl_msg.sequence - ctrl_sequence_num);
      }
      ctrl_sequence_num = new_ctrl_msg.sequence;
    }
  }
  else if (ctrl_delay > 1000){  // if no ctrl message received in last 1 second, try to hover
    comm_timed_out = 1;
    latest_control.roll = 0;
    latest_control.pitch = 0;
  }

  return latest_control;
};

void Communication::push_config(ConfigMessage new_config){
  Parameters latest_config = {new_config.a, new_config.b, new_config.c, new_config.x, new_config.y, new_config.z};
  latest_configs[static_cast<uint8_t>(new_config.axis)] = latest_config;  // store latest config
  updated_config[static_cast<uint8_t>(new_config.axis)] = true;  // set new config flag
}

Parameters Communication::pop_config(uint8_t axis){
  if (axis >= static_cast<uint8_t>(PID_AXIS::COUNT)) return Parameters();
  updated_config[axis] = false;
  return latest_configs[axis];
}

Message Communication::create_state_msg(State& state, float battery, uint8_t numSV, int avg_diff){
  StateMessage state_msg(millis(), state.rpy.storage, state.pos.storage, state.vel.storage, battery, numSV, avg_diff);
  return Message(state_msg.data);
}

Message Communication::create_ekf_msg(State& state, Measurements& m, float inn_mag){
  EKFMessage ekf_msg(millis(), state.rpy.storage, m.acc_vec.storage, m.mag_vec.storage, m.gyro_vec.storage, inn_mag);
  return Message(ekf_msg.data);
}

Message Communication::create_pos_msg(Vector3& pos_diff, Vector3& vel_diff, Vector3& acc_ref){
  PositionMessage pos_msg(millis(), pos_diff.storage, vel_diff.storage, acc_ref.storage);
  return Message(pos_msg.data);
}

Message Communication::create_attitude_msg(AttReference& att_ref, Vector3& ang_err, Vector3& pid_err, Vector3& PID_outputs){
  float ang_ref[3] = {att_ref.roll, att_ref.pitch, att_ref.yaw};
  AttitudeMessage att_msg(millis(), ang_ref, ang_err.storage, pid_err.storage, PID_outputs.storage, att_ref.throttle);
  return Message(att_msg.data);
}

Message Communication::create_quattitude_msg(AttReference& att_ref, Vector3& omega_des, Vector3& i_err, Vector3& tau, Vector4 mps){
  float ang_ref[3] = {att_ref.roll, att_ref.pitch, att_ref.yaw};
  QuattitudeMessage quat_msg(millis(), ang_ref, omega_des.storage, i_err.storage, tau.storage, mps.storage);
  return Message(quat_msg.data);
}

/**
 * Send message to remote. Takes just below 1ms.
 */
void Communication::send_msg(Message msg){
  if(DEBUG && DEBUG_COMM){
    Serial.print("Sending msg:");
    Serial.println(static_cast<uint8_t>(msg.message_type()));
  }
  radio.stopListening(); 
  bool report = radio.write(&msg.data, sizeof(msg.data), 1);
  radio.startListening(); 
}
