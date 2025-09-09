#ifndef COMMSTR
#define COMMSTR

#include <Arduino.h>

/// Data transformations for transmission purposes
#define ANGLE_FS 180.0f
#define FORCE_FS 200.0f
#define POS_FS 1000.0f
#define VEL_FS 10.0f
#define GYRO_TELEM_FS 720.0f
#define ACC_TELEM_FS 2.0f
#define MAG_TELEM_FS 1.3f
#define INN_MAG_FS 0.01f
#define MAX_RAD 6.28318530718f

#define MAX_15BIT 32767.0f
#define MAX_16BIT 65535.0f
#define MAX_8BIT 255.0f

using data_array_t = uint8_t[32];

// State struct conversions

/* Transform position from [-1000,1000] float to int16 */
inline int16_t pos_to_int(float pos){
  return int16_t(pos * (MAX_15BIT / POS_FS));
}

/* Transform pos from int16 to float [-1000, 1000] */
inline float pos_to_float(uint16_t pos){
  return float(pos) * (POS_FS / MAX_15BIT);
}

/* Transform vel from [-10,10] float to int16 */
inline int16_t vel_to_int(float vel){
  return int16_t(vel * (MAX_15BIT / VEL_FS));
}

/* Transform vel from int16 to float [-10, 10] */
inline float vel_to_float(uint16_t vel){
  return float(vel) * (VEL_FS / MAX_15BIT);
}

/* Transform a float angle [-180, 180] to 16 bit int representation for transmitting */
inline int16_t angle_to_int(float angle){
  return int16_t(angle * (MAX_15BIT / ANGLE_FS));  // * (2^15 - 1) / 180 ... rescaling to int16_t full scale
}

/* Transform an int angle [-(2^15 - 1), 2^15 - 1] to float angle [-180, 180] */
inline float angle_to_float(int16_t angle){
  return float(angle) * (ANGLE_FS / MAX_15BIT);  // * 180 / (2^15 - 1) ... rescaling to float again
}

/* Transform a float angle [-2pi, 2pi] to 16 bit int representation for transmitting */
inline int16_t rad_to_int(float angle){
  return int16_t(angle * (MAX_15BIT / MAX_RAD));  // * (2^15 - 1) / 2pi ... rescaling to int16_t full scale
}

/* Transform an int angle [-(2^15 - 1), 2^15 - 1] to float angle [-2pi, 2pi] */
inline float rad_to_float(int16_t angle){
  return float(angle) * (MAX_RAD / MAX_15BIT);  // * 2pi / (2^15 - 1) ... rescaling to float again
}

/* Transform motor setting from [0,1] float to int [0, 2^8 - 1] */
inline uint8_t action_to_int(float action){
  return uint8_t(action * MAX_8BIT);  // multiply by 2^8 - 1
}

/* Transform motor setting from int [0, 2^8 - 1] to float [0, 1] */
inline float action_to_float(uint8_t action){
  return float(action) * (1 / MAX_8BIT);  // divide by 2^8 - 1
}

/* Transform PID output from float [-FS, FS] to int [0, 2^15 - 1] */
inline int16_t force_to_int(float force){
  return int16_t (force * (MAX_15BIT / FORCE_FS));  // * (2^15 - 1) / FS
}

/* Transform PID output from int [0, 2^15 - 1] to float [-FS, FS]*/
inline float force_to_float(int16_t force){
  return float(force) * (FORCE_FS / MAX_15BIT);  // * FS / (2^15 - 1)
}
// sensor struct conversions

inline int16_t gyro_to_int(float gyro){
  return int16_t(gyro * (MAX_15BIT / GYRO_TELEM_FS));
}

inline float gyro_to_float(int16_t gyro){
  return float(gyro) * (GYRO_TELEM_FS / MAX_15BIT);
}

inline int16_t acc_to_int(float acc){
  return int16_t(acc * (MAX_15BIT / ACC_TELEM_FS));
}

inline float acc_to_float(int16_t acc){
  return float(acc) * (ACC_TELEM_FS / MAX_15BIT);
}

inline int16_t mag_to_int(float mag){
  return int16_t(mag * (MAX_15BIT / MAG_TELEM_FS));
}

inline float mag_to_float(int16_t mag){
  return float(mag) * (MAG_TELEM_FS / MAX_15BIT);
}

inline int16_t inn_mag_to_int(float inn_mag){
  return int16_t(inn_mag * (MAX_16BIT / INN_MAG_FS));
}

inline float inn_mag_to_float(int16_t inn_mag){
  return float(inn_mag) * (INN_MAG_FS / MAX_16BIT);
}

inline int16_t throttle_to_int(float throttle){
  return int16_t(throttle * MAX_16BIT);
}

inline float throttle_to_float(int16_t throttle){
  return float(throttle) * (1 / MAX_16BIT);
}

// Data structs for communication
enum class PID_AXIS : uint8_t {
  ROLL = 0,
  PITCH = 1,
  YAW = 2, 
  XY = 3,
  Z = 4,
  XYVEL = 5,
  ZVEL = 6,
  QUAT_RATE = 7,
  QUAT_P = 8, 
  QUAT_I = 9, 
  QUAT_D = 10,
  QUAT_TL = 11,
  QUAT_LPC = 12,
  COUNT = 13
};

enum class MessageType : uint8_t {
  NONE = 0,
  CONTROL = 1,
  CONFIG = 2,
  STATE = 3,
  ATTITUDE = 4,
  POS = 5,
  EKF = 6,
  QUATTITUDE = 7
};

struct Message{
  data_array_t data = {};
  Message(){};
  Message(data_array_t& buff);
  MessageType message_type();
  void load_data(data_array_t& buff);
  void print_raw_data(Print& out);
};

struct ControlMessage : public Message{
  static constexpr MessageType message_id = MessageType::CONTROL;

  float roll;
  float pitch;
  float yaw_diff;
  float throttle;
  float alt_diff;
  uint8_t motors_on;
  uint32_t sequence;
  ControlMessage(float rpy[3], float throttle, float alt_diff, uint8_t motors_on, uint32_t sequence);
  using Message::Message;
  data_array_t& serialize();
  void deserialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct ConfigMessage : public Message{
  static constexpr MessageType message_id = MessageType::CONFIG;

  PID_AXIS axis;
  float a;
  float b;
  float c;
  float x;
  float y;
  float z;
  ConfigMessage(uint8_t axis, float a, float b, float c, float x, float y, float z);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct StateMessage : public Message{
  static constexpr MessageType message_id = MessageType::STATE;

  uint32_t ms;
  float roll;
  float pitch;
  float yaw;
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float battery;
  uint8_t numSV;
  int avg_diff;
  StateMessage(uint32_t ms, float rpy[3], float pos[3], float vel[3], float battery, uint8_t numSV, int avg_diff);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct AttitudeMessage : public Message{
  static constexpr MessageType message_id = MessageType::ATTITUDE;

  uint32_t ms;
  float roll_ref;
  float pitch_ref;
  float yaw_ref;
  float roll_err;
  float pitch_err;
  float yaw_err;
  float roll_rate_err;
  float pitch_rate_err;
  float yaw_rate_err;
  float roll_pid_output;
  float pitch_pid_output;
  float yaw_pid_output;
  float throttle;
  AttitudeMessage(uint32_t ms, float rpy_ref[3], float rpy_err[3], float rpy_rate_err[3], float rpy_pid_output[3], float throttle);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct PositionMessage : public Message{
  static constexpr MessageType message_id = MessageType::POS;

  uint32_t ms;
  float x_err;
  float y_err;
  float z_err;
  float x_vel_err;
  float y_vel_err;
  float z_vel_err;
  float x_acc_ref;
  float y_acc_ref;
  float z_acc_ref;
  PositionMessage(uint32_t ms, float pos_err[3], float vel_err[3], float acc_ref[3]);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct EKFMessage : public Message{
  static constexpr MessageType message_id = MessageType::EKF;
  uint32_t ms;
  float roll;
  float pitch;
  float yaw;
  float acc_x;
  float acc_y;
  float acc_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float inn_mag;
  EKFMessage(uint32_t ms, float rpy[3], float acc[3], float mag[3], float gyro[3], float inn_mag);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

struct QuattitudeMessage : public Message{
  static constexpr MessageType message_id = MessageType::QUATTITUDE;

  uint16_t ms;
  float roll_ref, pitch_ref, yaw_ref;
  float omega_x, omega_y, omega_z;
  float i_err_x, i_err_y, i_err_z;
  float tau_x, tau_y, tau_z;
  float mp1, mp2, mp3, mp4;
  QuattitudeMessage(uint16_t ms, float rpy_ref[3], float omega_des[3], float i_err[3], float tau[3], float mps[4]);
  using Message::Message;
  data_array_t& serialize();
  bool deserialize(data_array_t& buff);
  void printTo(Print &out);
};

#endif
