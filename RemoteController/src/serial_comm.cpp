#include "serial_comm.h"

/**
 * take received state_data struct, remap values to original ranges and format into string 
 */
void format_state(StateMessage s_msg, char* buffr, size_t bsize){
  float all_floats[12] = {
    s_msg.roll, s_msg.pitch, s_msg.yaw,   
    s_msg.x, s_msg.y, s_msg.z, 
    s_msg.vx, s_msg.vy, s_msg.vz, 
    s_msg.battery, float(s_msg.numSV), s_msg.avg_diff
  };

  // format floats into string
  snprintf(buffr, bsize, "State: %lu ",  s_msg.ms);
  format_floats_to_buffer(buffr, all_floats, 12);
}

void format_control(ControlMessage ctrl_msg, char* buffr, size_t bsize){

  float all_floats[5] = {
    ctrl_msg.roll, ctrl_msg.pitch, ctrl_msg.yaw_diff, ctrl_msg.alt_diff, ctrl_msg.throttle
  };

  // format floats into string
  printf(buffr, bsize, "Control: ");
  format_floats_to_buffer(buffr, all_floats, 5);
}

void format_attitude(AttitudeMessage att_msg, char* buffr, size_t bsize){
  float all_floats[12] = {
    att_msg.roll_ref, att_msg.pitch_ref, att_msg.yaw_ref, 
    att_msg.roll_err, att_msg.pitch_err, att_msg.yaw_err, 
    att_msg.roll_rate_err, att_msg.pitch_rate_err, att_msg.yaw_rate_err, 
    att_msg.roll_pid_output, att_msg.pitch_pid_output, att_msg.yaw_pid_output};

  // format floats into string
  snprintf(buffr, bsize, "State: %lu ",  att_msg.ms);
  format_floats_to_buffer(buffr, all_floats, 12);
}

/**
 * take received ekf_struct struct, remap values to original ranges and format into string 
 */
void format_ekf(EKFMessage ekf_msg, char* buffr, size_t bsize){
  float all_floats[12] = {
    ekf_msg.roll, ekf_msg.pitch, ekf_msg.yaw,
    ekf_msg.acc_x, ekf_msg.acc_y, ekf_msg.acc_z,
    ekf_msg.mag_x, ekf_msg.mag_y, ekf_msg.mag_z,
    ekf_msg.gyro_x, ekf_msg.gyro_y, ekf_msg.gyro_z
  };

  // format floats into string
  snprintf(buffr, bsize, "EKF: %lu ",  ekf_msg.ms);
  format_floats_to_buffer(buffr, all_floats, 12);
}

/**
 * add float array to string with spaces between
 */
void format_floats_to_buffer(char* buffr, float* all_floats, uint8_t num_floats){
  char temp_b[16]; 
  int str_len = strlen(buffr);
  for (int i = 0; i < num_floats; i++){
    dtostrf(all_floats[i], 3, 2, temp_b);
    int float_len = strlen(temp_b);
    for (int j = 0; j < float_len; j++){
      buffr[str_len + j] = temp_b[j];  // copy float to string
    }
    buffr[str_len + float_len] = ' ';  // set a space
    buffr[str_len + float_len + 1] = '\0';  // set a string end char
    str_len += float_len + 1;
  }
}

/**
 * Interpret telemetry message and print it to serial in either human readable or binary format
 */
void print_message_to_serial(Message& msg){
  static uint32_t last_shown = 0;
  if (msg.message_type() == MessageType::STATE){  // state data
    #if PRINT_MIN_DELAY > 0  // wait at least MIN_DELAY before printing telemetry
    uint32_t current_t = millis();
    if (current_t - last_shown < PRINT_MIN_DELAY) return;
    last_shown = current_t;
    #endif
    
    if (HUMAN_READABLE){
        char buffr[100];
        StateMessage s_msg(msg.data);
        format_state(s_msg, buffr, sizeof(buffr));  //1168 micros
        Serial.println(buffr);  // 828 micros
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
  else if (msg.message_type() == MessageType::CONTROL){  // control data
    if (HUMAN_READABLE){
      ControlMessage ctrl_msg(msg.data);
      char buffr[100];
      format_control(ctrl_msg, buffr, sizeof(buffr));
      Serial.println(buffr);
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
  else if (msg.message_type() == MessageType::ATTITUDE){  // attitude data
    if (HUMAN_READABLE){
      AttitudeMessage att_msg(msg.data);
      char buffr[100];
      format_attitude(att_msg, buffr, sizeof(buffr));
      Serial.println(buffr);
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
  else if (msg.message_type() == MessageType::POS){  // attitude data
    if (HUMAN_READABLE){
      PositionMessage pos_msg(msg.data);
      pos_msg.printTo(Serial);
      Serial.println();
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
  else if (msg.message_type() == MessageType::EKF){  // ekf data
    if (HUMAN_READABLE){
      EKFMessage ekf_msg(msg.data);
      char buffr[100];
      format_ekf(ekf_msg, buffr, sizeof(buffr));
      Serial.println(buffr);
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
  else if (msg.message_type() == MessageType::QUATTITUDE){  // quattitude data
    if (HUMAN_READABLE){
      QuattitudeMessage quat_msg(msg.data);
      quat_msg.printTo(Serial);
    }
    else Serial.write((uint8_t*)&msg.data, sizeof(msg.data));
  }
}


String valid_pid_axis[] = {"roll", "pitch", "yaw", "xy", "z", "xyv", "zv", "qr", "qrp", "qri", "qrd", "qtl", "qlp"};
bool string_valid_pid_axis(String str){
  for (int i = 0; i < 13; i++) if (str == valid_pid_axis[i]) return true;
  return false;
}

PID_AXIS string_to_pid_axis(String str){
  if (str == "roll") return PID_AXIS::ROLL;
  if (str == "pitch") return PID_AXIS::PITCH;
  if (str == "yaw") return PID_AXIS::YAW;
  if (str == "xy") return PID_AXIS::XY;
  if (str == "z") return PID_AXIS::Z;
  if (str == "xyv") return PID_AXIS::XYVEL;
  if (str == "zv") return PID_AXIS::ZVEL;
  if (str == "qr") return PID_AXIS::QUAT_RATE;
  if (str == "qrp") return PID_AXIS::QUAT_P;
  if (str == "qri") return PID_AXIS::QUAT_I;
  if (str == "qrd") return PID_AXIS::QUAT_D;
  if (str == "qtl") return PID_AXIS::QUAT_TL;
  if (str == "qlp") return PID_AXIS::QUAT_LPC;
  return PID_AXIS::ROLL;  // default to ROLL if not found
}

bool parse_serial(Message& msg){
  if (!(Serial.available() > 0)) return 0;  // no message
  String serial_line = Serial.readStringUntil('\n');  // serial line
  uint8_t space_loc = serial_line.indexOf(' ');  // first space
  String msg_type = serial_line.substring(0, space_loc);
  if (!string_valid_pid_axis(msg_type)) return 0;  // no message
  ConfigMessage new_config = config_from_line(serial_line);

  new_config.axis = string_to_pid_axis(msg_type);
  if (DEBUG) {
    Serial.print("Config for axis: ");
    Serial.println(msg_type);
    if (static_cast<uint8_t> (new_config.axis) < 7){
      String text = "P " + String(new_config.a, 2) + ", I " + String(new_config.b, 2) + 
      ", D " + String(new_config.c, 2) + ", sat " + String(new_config.x, 2) + ", LPc " + String(new_config.y, 2);
      Serial.println(text);
    }
    else {
      String text = "a " + String(new_config.a, 2) + ", b " + String(new_config.b, 2) + 
      ", c " + String(new_config.c, 2) + ", x " + String(new_config.x, 2) + ", y " + 
      String(new_config.y, 2) + ", z " + String(new_config.z, 2);
      Serial.println(text);
    }
  }

  msg.load_data(new_config.serialize());
  return 1;
}

ConfigMessage config_from_line(String serial_line){
  uint8_t space_loc = serial_line.indexOf(' ');  // first space
  float serial_reading[6] = {0,0,0,0,0,0};

  bool end_of_line = false;
  for (int i = 0; i < 6; i++){
    int next_space = serial_line.indexOf(' ', space_loc + 1);
    if (next_space == -1) {
      next_space = serial_line.length();
      end_of_line = true;
    }
    String numero = serial_line.substring(space_loc, next_space);
    serial_reading[i] = numero.toFloat();
    space_loc = next_space;
    if (end_of_line) break;
  } 

  float a = serial_reading[0];
  float b = serial_reading[1];
  float c = serial_reading[2];
  float x = serial_reading[3];
  float y = serial_reading[4];
  float z = serial_reading[5];
  // put into config message and print possibly
  ConfigMessage cfg_msg(0, a, b, c, x, y, z);
  return cfg_msg;
}