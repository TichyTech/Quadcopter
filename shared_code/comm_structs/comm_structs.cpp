#include "comm_structs.h"

Message::Message(data_array_t& buff) {
  load_data(buff);
}

void Message::load_data(data_array_t& buff) {
  for (int i = 0; i < 32; ++i) {
    data[i] = buff[i];
  }
}

void Message::print_raw_data(Print &out)
{
  for (int i = 0; i < 32; ++i) {
    out.print(data[i], HEX);
    out.print(" ");
  }
}

MessageType Message::message_type(){
    return static_cast<MessageType> (data[0]);
}

ControlMessage::ControlMessage(float rpy[3], float throttle, float alt_diff, uint8_t motors_on, uint32_t sequence)
  : roll(rpy[0]), pitch(rpy[1]), yaw_diff(rpy[2]), throttle(throttle), alt_diff(alt_diff), motors_on(motors_on), sequence(sequence) 
{
  serialize();
}

data_array_t& ControlMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<int16_t*>(&data[1]) = angle_to_int(roll);
  *reinterpret_cast<int16_t*>(&data[3]) = angle_to_int(pitch);
  *reinterpret_cast<int16_t*>(&data[5]) = angle_to_int(yaw_diff);
  *reinterpret_cast<uint8_t*>(&data[7]) = action_to_int(throttle);
  *reinterpret_cast<int16_t*>(&data[8]) = pos_to_int(alt_diff);
  *reinterpret_cast<uint8_t*>(&data[10]) = motors_on;
  *reinterpret_cast<uint32_t*>(&data[11]) = sequence;
  return data;
}

bool ControlMessage::deserialize(data_array_t &buff)
{
  load_data(buff);
  if (message_type() != message_id) return false;
  deserialize();
  return true;
}

void ControlMessage::deserialize() {
  roll = angle_to_float(*reinterpret_cast<int16_t*>(&data[1]));
  pitch = angle_to_float(*reinterpret_cast<int16_t*>(&data[3]));
  yaw_diff = angle_to_float(*reinterpret_cast<int16_t*>(&data[5]));
  throttle = action_to_float(*reinterpret_cast<uint8_t*>(&data[7]));
  alt_diff = pos_to_float(*reinterpret_cast<int16_t*>(&data[8]));
  motors_on = *reinterpret_cast<uint8_t*>(&data[10]);
  sequence = *reinterpret_cast<uint32_t*>(&data[11]);
}

void ControlMessage::printTo(Print &out) {
  out.print("ControlMessage: roll=");
  out.print(roll, 2);
  out.print(", pitch=");
  out.print(pitch, 2);
  out.print(", yaw_diff=");
  out.print(yaw_diff, 2);
  out.print(", throttle=");
  out.print(throttle, 2);
  out.print(", alt_diff=");
  out.print(alt_diff, 2);
  out.print(", motors_on=");
  out.print(motors_on);
  out.print(", sequence=");
  out.println(sequence);
}

ConfigMessage::ConfigMessage(uint8_t axis, float a, float b, float c, float x, float y, float z)
  : axis(static_cast<PID_AXIS>(axis)), a(a), b(b), c(c), x(x), y(y), z(z)
{
  serialize();
}

data_array_t& ConfigMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  data[1] = static_cast<uint8_t>(axis);
  *reinterpret_cast<float*>(&data[2]) = a;
  *reinterpret_cast<float*>(&data[6]) = b;
  *reinterpret_cast<float*>(&data[10]) = c;
  *reinterpret_cast<float*>(&data[14]) = x;
  *reinterpret_cast<float*>(&data[18]) = y;
  *reinterpret_cast<float*>(&data[22]) = z;
  return data;
}

bool ConfigMessage::deserialize(data_array_t& buff) {
  load_data(buff);
  if (message_type() != message_id) return false;
  axis = static_cast<PID_AXIS>(buff[1]);
  a = *reinterpret_cast<float*>(&buff[2]);
  b = *reinterpret_cast<float*>(&buff[6]);
  c = *reinterpret_cast<float*>(&buff[10]);
  x = *reinterpret_cast<float*>(&buff[14]);
  y = *reinterpret_cast<float*>(&buff[18]);
  z = *reinterpret_cast<float*>(&buff[22]);
  return true;
}

void ConfigMessage::printTo(Print &out) {
  out.print("ConfigMessage: axis=");
  out.println(static_cast<uint8_t>(axis));
  out.print(" a=");
  out.print(a, 2);
  out.print(", b=");
  out.print(b, 2);
  out.print(", c=");
  out.print(c, 2);
  out.print(", x=");
  out.print(x, 2);
  out.print(", y=");
  out.print(y, 2);
  out.print(", z=");
  out.print(z, 2);
}

StateMessage::StateMessage(uint32_t ms, float rpy[3], float pos[3], float vel[3], float battery, uint8_t numSV, int avg_diff)
  : ms(ms), roll(rpy[0]), pitch(rpy[1]), yaw(rpy[2]),
    x(pos[0]), y(pos[1]), z(pos[2]),
    vx(vel[0]), vy(vel[1]), vz(vel[2]),
    battery(battery), numSV(numSV), avg_diff(avg_diff)
{
  serialize();
}

data_array_t& StateMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<uint32_t*>(&data[1]) = ms;
  *reinterpret_cast<int16_t*>(&data[5]) = angle_to_int(roll);
  *reinterpret_cast<int16_t*>(&data[7]) = angle_to_int(pitch);
  *reinterpret_cast<int16_t*>(&data[9]) = angle_to_int(yaw);
  *reinterpret_cast<int16_t*>(&data[11]) = pos_to_int(x);
  *reinterpret_cast<int16_t*>(&data[13]) = pos_to_int(y);
  *reinterpret_cast<int16_t*>(&data[15]) = pos_to_int(z);
  *reinterpret_cast<int16_t*>(&data[17]) = vel_to_int(vx);
  *reinterpret_cast<int16_t*>(&data[19]) = vel_to_int(vy);
  *reinterpret_cast<int16_t*>(&data[21]) = vel_to_int(vz);
  *reinterpret_cast<float*>(&data[23]) = battery;
  data[27] = numSV;
  *reinterpret_cast<int*>(&data[28]) = avg_diff;
  return data;
}

bool StateMessage::deserialize(data_array_t& buff) {
  load_data(buff);
  if (message_type() != message_id) return false;
  ms = *reinterpret_cast<uint32_t*>(&buff[1]);
  roll = angle_to_float(*reinterpret_cast<int16_t*>(&buff[5]));
  pitch = angle_to_float(*reinterpret_cast<int16_t*>(&buff[7]));
  yaw = angle_to_float(*reinterpret_cast<int16_t*>(&buff[9]));
  x = pos_to_float(*reinterpret_cast<int16_t*>(&buff[11]));
  y = pos_to_float(*reinterpret_cast<int16_t*>(&buff[13]));
  z = pos_to_float(*reinterpret_cast<int16_t*>(&buff[15]));
  vx = vel_to_float(*reinterpret_cast<int16_t*>(&buff[17]));
  vy = vel_to_float(*reinterpret_cast<int16_t*>(&buff[19]));
  vz = vel_to_float(*reinterpret_cast<int16_t*>(&buff[21]));
  battery = *reinterpret_cast<float*>(&buff[23]);
  numSV = buff[27];
  avg_diff = *reinterpret_cast<int*>(&buff[28]);
  return true;
}

void StateMessage::printTo(Print &out) {
  out.print("StateMessage: ms=");
  out.print(ms);
  out.print(", roll=");
  out.print(roll, 2);
  out.print(", pitch=");
  out.print(pitch, 2);
  out.print(", yaw=");
  out.print(yaw, 2);
  out.print(", x=");
  out.print(x, 2);
  out.print(", y=");
  out.print(y, 2);
  out.print(", z=");
  out.print(z, 2);
  out.print(", vx=");
  out.print(vx, 2);
  out.print(", vy=");
  out.print(vy, 2);
  out.print(", vz=");
  out.print(vz, 2);
  out.print(", battery=");
  out.print(battery, 2);
  out.print(", numSV=");
  out.print(numSV);
  out.print(", avg_diff=");
  out.print(avg_diff, 2);
  out.println();
}

AttitudeMessage::AttitudeMessage(uint32_t ms, float rpy_ref[3], float rpy_err[3], float rpy_rate_err[3], float rpy_pid_output[3], float throttle) :
  ms(ms), roll_ref(rpy_ref[0]), pitch_ref(rpy_ref[1]), yaw_ref(rpy_ref[2]),
  roll_err(rpy_err[0]), pitch_err(rpy_err[1]), yaw_err(rpy_err[2]),
  roll_rate_err(rpy_rate_err[0]), pitch_rate_err(rpy_rate_err[1]), yaw_rate_err(rpy_rate_err[2]),
  roll_pid_output(rpy_pid_output[0]), pitch_pid_output(rpy_pid_output[1]), yaw_pid_output(rpy_pid_output[2]), throttle(throttle)
{
  serialize();
}

data_array_t& AttitudeMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<uint32_t*>(&data[1]) = ms;
  *reinterpret_cast<int16_t*>(&data[5]) = angle_to_int(roll_ref);
  *reinterpret_cast<int16_t*>(&data[7]) = angle_to_int(pitch_ref);
  *reinterpret_cast<int16_t*>(&data[9]) = angle_to_int(yaw_ref);
  *reinterpret_cast<int16_t*>(&data[11]) = angle_to_int(roll_err);
  *reinterpret_cast<int16_t*>(&data[13]) = angle_to_int(pitch_err);
  *reinterpret_cast<int16_t*>(&data[15]) = angle_to_int(yaw_err);
  *reinterpret_cast<int16_t*>(&data[17]) = gyro_to_int(roll_rate_err);
  *reinterpret_cast<int16_t*>(&data[19]) = gyro_to_int(pitch_rate_err);
  *reinterpret_cast<int16_t*>(&data[21]) = gyro_to_int(yaw_rate_err);
  *reinterpret_cast<int16_t*>(&data[23]) = force_to_int(roll_pid_output);
  *reinterpret_cast<int16_t*>(&data[25]) = force_to_int(pitch_pid_output);
  *reinterpret_cast<int16_t*>(&data[27]) = force_to_int(yaw_pid_output);
  *reinterpret_cast<int16_t*>(&data[29]) = throttle_to_int(throttle);
  return data;
}

bool AttitudeMessage::deserialize(data_array_t& buff) {
  load_data(buff);
  if (message_type() != message_id) return false;
  ms = *reinterpret_cast<uint32_t*>(&buff[1]);
  roll_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[5]));
  pitch_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[7]));
  yaw_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[9]));
  roll_err = angle_to_float(*reinterpret_cast<int16_t*>(&buff[11]));
  pitch_err = angle_to_float(*reinterpret_cast<int16_t*>(&buff[13]));
  yaw_err = angle_to_float(*reinterpret_cast<int16_t*>(&buff[15]));
  roll_rate_err = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[17]));
  pitch_rate_err = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[19]));
  yaw_rate_err = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[21]));
  roll_pid_output = force_to_float(*reinterpret_cast<int16_t*>(&buff[23]));
  pitch_pid_output = force_to_float(*reinterpret_cast<int16_t*>(&buff[25]));
  yaw_pid_output = force_to_float(*reinterpret_cast<int16_t*>(&buff[27]));
  throttle = throttle_to_float(*reinterpret_cast<int16_t*>(&buff[29]));
  return true;
}

void AttitudeMessage::printTo(Print &out) {
  out.print("AttitudeMessage: ms=");
  out.print(ms);
  out.print(", roll_ref=");
  out.print(roll_ref, 2);
  out.print(", pitch_ref=");
  out.print(pitch_ref, 2);
  out.print(", yaw_ref=");
  out.print(yaw_ref, 2);
  out.print(", roll_err=");
  out.print(roll_err, 2);
  out.print(", pitch_err=");
  out.print(pitch_err, 2);
  out.print(", yaw_err=");
  out.print(yaw_err, 2);
  out.print(", roll_rate_err=");
  out.print(roll_rate_err, 2);
  out.print(", pitch_rate_err=");
  out.print(pitch_rate_err, 2);
  out.print(", yaw_rate_err=");
  out.print(yaw_rate_err, 2);
  out.print(", roll_pid_output=");
  out.print(roll_pid_output, 2);
  out.print(", pitch_pid_output=");
  out.print(pitch_pid_output, 2);
  out.print(", yaw_pid_output=");
  out.print(yaw_pid_output, 2);
  out.print(", throttle=");
  out.print(throttle, 2);
  out.println();
}

PositionMessage::PositionMessage(uint32_t ms, float pos_err[3], float vel_err[3], float acc_ref[3]) :
  ms(ms), x_err(pos_err[0]), y_err(pos_err[1]), z_err(pos_err[2]),
  x_vel_err(vel_err[0]), y_vel_err(vel_err[1]), z_vel_err(vel_err[2]),
  x_acc_ref(acc_ref[0]), y_acc_ref(acc_ref[1]), z_acc_ref(acc_ref[2])
{
  serialize();
}

data_array_t& PositionMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<uint32_t*>(&data[1]) = ms;
  *reinterpret_cast<int16_t*>(&data[5]) = pos_to_int(x_err);
  *reinterpret_cast<int16_t*>(&data[7]) = pos_to_int(y_err);
  *reinterpret_cast<int16_t*>(&data[9]) = pos_to_int(z_err);
  *reinterpret_cast<int16_t*>(&data[11]) = vel_to_int(x_vel_err);
  *reinterpret_cast<int16_t*>(&data[13]) = vel_to_int(y_vel_err);
  *reinterpret_cast<int16_t*>(&data[15]) = vel_to_int(z_vel_err);
  *reinterpret_cast<uint16_t*>(&data[17]) = acc_to_int(x_acc_ref);
  *reinterpret_cast<uint16_t*>(&data[19]) = acc_to_int(y_acc_ref);
  *reinterpret_cast<uint16_t*>(&data[21]) = acc_to_int(z_acc_ref);
  return data;
}

bool PositionMessage::deserialize(data_array_t& buff) {
  load_data(buff);
  if (message_type() != message_id) return false;
  ms = *reinterpret_cast<uint32_t*>(&buff[1]);
  x_err = pos_to_float(*reinterpret_cast<int16_t*>(&buff[5]));
  y_err = pos_to_float(*reinterpret_cast<int16_t*>(&buff[7]));
  z_err = pos_to_float(*reinterpret_cast<int16_t*>(&buff[9]));
  x_vel_err = vel_to_float(*reinterpret_cast<int16_t*>(&buff[11]));
  y_vel_err = vel_to_float(*reinterpret_cast<int16_t*>(&buff[13]));
  z_vel_err = vel_to_float(*reinterpret_cast<int16_t*>(&buff[15]));
  x_acc_ref = acc_to_float(*reinterpret_cast<uint16_t*>(&buff[17]));
  y_acc_ref = acc_to_float(*reinterpret_cast<uint16_t*>(&buff[19]));
  z_acc_ref = acc_to_float(*reinterpret_cast<uint16_t*>(&buff[21]));
  return true;
}

void PositionMessage::printTo(Print &out) {
  out.print("PositionMessage: ms=");
  out.print(ms);
  out.print(", x_err=");
  out.print(x_err, 2);
  out.print(", y_err=");
  out.print(y_err, 2);
  out.print(", z_err=");
  out.print(z_err, 2);
  out.print(", x_vel_err=");
  out.print(x_vel_err, 2);
  out.print(", y_vel_err=");
  out.print(y_vel_err, 2);
  out.print(", z_vel_err=");
  out.print(z_vel_err, 2);
  out.print(", x_acc_ref=");
  out.print(x_acc_ref, 2);
  out.print(", y_acc_ref=");
  out.print(y_acc_ref, 2);
  out.print(", z_acc_ref=");
  out.print(z_acc_ref, 2);
  out.println();
}

EKFMessage::EKFMessage(uint32_t ms, float rpy[3], float acc[3], float mag[3], float gyro[3], float inn_mag)
  : ms(ms), roll(rpy[0]), pitch(rpy[1]), yaw(rpy[2]),
    acc_x(acc[0]), acc_y(acc[1]), acc_z(acc[2]),
    mag_x(mag[0]), mag_y(mag[1]), mag_z(mag[2]),
    gyro_x(gyro[0]), gyro_y(gyro[1]), gyro_z(gyro[2]),
    inn_mag(inn_mag)
{
  serialize();
}

data_array_t& EKFMessage::serialize() {
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<uint32_t*>(&data[1]) = ms;
  *reinterpret_cast<int16_t*>(&data[5]) = angle_to_int(roll);
  *reinterpret_cast<int16_t*>(&data[7]) = angle_to_int(pitch);
  *reinterpret_cast<int16_t*>(&data[9]) = angle_to_int(yaw);
  *reinterpret_cast<int16_t*>(&data[11]) = acc_to_int(acc_x);
  *reinterpret_cast<int16_t*>(&data[13]) = acc_to_int(acc_y);
  *reinterpret_cast<int16_t*>(&data[15]) = acc_to_int(acc_z);
  *reinterpret_cast<int16_t*>(&data[17]) = mag_to_int(mag_x);
  *reinterpret_cast<int16_t*>(&data[19]) = mag_to_int(mag_y);
  *reinterpret_cast<int16_t*>(&data[21]) = mag_to_int(mag_z);
  *reinterpret_cast<int16_t*>(&data[23]) = gyro_to_int(gyro_x);
  *reinterpret_cast<int16_t*>(&data[25]) = gyro_to_int(gyro_y);
  *reinterpret_cast<int16_t*>(&data[27]) = gyro_to_int(gyro_z);
  *reinterpret_cast<int16_t*>(&data[29]) = inn_mag_to_int(inn_mag);
  return data;
}

bool EKFMessage::deserialize(data_array_t& buff) {
  load_data(buff);
  if (message_type() != message_id) return false;  
  ms = *reinterpret_cast<uint32_t*>(&buff[1]);
  roll = angle_to_float(*reinterpret_cast<int16_t*>(&buff[5]));
  pitch = angle_to_float(*reinterpret_cast<int16_t*>(&buff[7]));
  yaw = angle_to_float(*reinterpret_cast<int16_t*>(&buff[9]));
  acc_x = acc_to_float(*reinterpret_cast<int16_t*>(&buff[11]));
  acc_y = acc_to_float(*reinterpret_cast<int16_t*>(&buff[13]));
  acc_z = acc_to_float(*reinterpret_cast<int16_t*>(&buff[15]));
  mag_x = mag_to_float(*reinterpret_cast<int16_t*>(&buff[17]));
  mag_y = mag_to_float(*reinterpret_cast<int16_t*>(&buff[19]));
  mag_z = mag_to_float(*reinterpret_cast<int16_t*>(&buff[21]));
  gyro_x = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[23]));
  gyro_y = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[25]));
  gyro_z = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[27]));
  inn_mag = inn_mag_to_float(*reinterpret_cast<int16_t*>(&buff[29]));
  return true;
}

void EKFMessage::printTo(Print &out) {
  out.print("EKFMessage: ms=");
  out.print(ms);
  out.print(", roll=");
  out.print(roll, 2);
  out.print(", pitch=");
  out.print(pitch, 2);
  out.print(", yaw=");
  out.print(yaw, 2);
  out.print(", acc_x=");
  out.print(acc_x, 2);
  out.print(", acc_y=");
  out.print(acc_y, 2);
  out.print(", acc_z=");
  out.print(acc_z, 2);
  out.print(", mag_x=");
  out.print(mag_x, 2);
  out.print(", mag_y=");
  out.print(mag_y, 2);
  out.print(", mag_z=");
  out.print(mag_z, 2);
  out.print(", gyro_x=");
  out.print(gyro_x, 2);
  out.print(", gyro_y=");
  out.print(gyro_y, 2);
  out.print(", gyro_z=");
  out.print(gyro_z, 2);
  out.print(", inn_mag=");
  out.print(inn_mag, 2);
  out.println();
}

QuattitudeMessage::QuattitudeMessage(uint16_t ms, float rpy_ref[3], float omega_des[3], float i_err[3], float tau[3], float mps[4]) : 
ms(ms), roll_ref(rpy_ref[0]), pitch_ref(rpy_ref[1]), yaw_ref(rpy_ref[2]),
i_err_x(i_err[0]), i_err_y(i_err[1]), i_err_z(i_err[2]),
omega_x(omega_des[0]), omega_y(omega_des[1]), omega_z(omega_des[2]),
tau_x(tau[0]), tau_y(tau[1]), tau_z(tau[2]),
mp1(mps[0]), mp2(mps[1]), mp3(mps[2]), mp4(mps[3])
{
  serialize();
}

data_array_t& QuattitudeMessage::serialize(){
  data[0] = static_cast<uint8_t> (message_id);
  *reinterpret_cast<uint16_t*>(&data[1]) = ms;
  *reinterpret_cast<int16_t*>(&data[3]) = angle_to_int(roll_ref);
  *reinterpret_cast<int16_t*>(&data[5]) = angle_to_int(pitch_ref);
  *reinterpret_cast<int16_t*>(&data[7]) = angle_to_int(yaw_ref);
  *reinterpret_cast<uint16_t*>(&data[9]) = rad_to_int(i_err_x);
  *reinterpret_cast<uint16_t*>(&data[11]) = rad_to_int(i_err_y);
  *reinterpret_cast<uint16_t*>(&data[13]) = rad_to_int(i_err_z);
  *reinterpret_cast<int16_t*>(&data[15]) = gyro_to_int(omega_x);
  *reinterpret_cast<int16_t*>(&data[17]) = gyro_to_int(omega_y);
  *reinterpret_cast<int16_t*>(&data[19]) = gyro_to_int(omega_z);
  *reinterpret_cast<int16_t*>(&data[21]) = rad_to_int(tau_x);
  *reinterpret_cast<int16_t*>(&data[23]) = rad_to_int(tau_y);
  *reinterpret_cast<int16_t*>(&data[25]) = rad_to_int(tau_z);
  *reinterpret_cast<uint8_t*>(&data[27]) = action_to_int(mp1);
  *reinterpret_cast<uint8_t*>(&data[28]) = action_to_int(mp2);
  *reinterpret_cast<uint8_t*>(&data[29]) = action_to_int(mp3);
  *reinterpret_cast<uint8_t*>(&data[30]) = action_to_int(mp4);
  return data;
}

bool QuattitudeMessage::deserialize(data_array_t& buff){
  load_data(buff);
  if (message_type() != message_id) return false;  
  ms = *reinterpret_cast<uint16_t*>(&buff[1]);
  roll_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[3]));
  pitch_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[5]));
  yaw_ref = angle_to_float(*reinterpret_cast<int16_t*>(&buff[7]));
  i_err_x = rad_to_float(*reinterpret_cast<uint16_t*>(&buff[9]));
  i_err_y = rad_to_float(*reinterpret_cast<uint16_t*>(&buff[11]));
  i_err_z = rad_to_float(*reinterpret_cast<uint16_t*>(&buff[13]));
  omega_x = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[15]));
  omega_y = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[17]));
  omega_z = gyro_to_float(*reinterpret_cast<int16_t*>(&buff[19]));
  tau_x = rad_to_float(*reinterpret_cast<int16_t*>(&buff[21]));
  tau_y = rad_to_float(*reinterpret_cast<int16_t*>(&buff[23]));
  tau_z = rad_to_float(*reinterpret_cast<int16_t*>(&buff[25]));
  mp1 = action_to_float(*reinterpret_cast<uint8_t*>(&buff[27]));
  mp2 = action_to_float(*reinterpret_cast<uint8_t*>(&buff[28]));
  mp3 = action_to_float(*reinterpret_cast<uint8_t*>(&buff[29]));
  mp4 = action_to_float(*reinterpret_cast<uint8_t*>(&buff[30]));
  return true;
}

void QuattitudeMessage::printTo(Print &out){
  out.print("QuattitudeMessage: ms=");
  out.print(ms);
  out.print(", roll_ref=");
  out.print(roll_ref, 2);
  out.print(", pitch_ref=");
  out.print(pitch_ref, 2);
  out.print(", yaw_ref=");
  out.print(yaw_ref, 2);
  out.print(", i_err_x=");
  out.print(i_err_x, 2);
  out.print(", i_err_y=");
  out.print(i_err_y, 2);
  out.print(", i_err_z=");
  out.print(i_err_z, 2);
  out.print(", omega_x=");
  out.print(omega_x, 2);
  out.print(", omega_y=");
  out.print(omega_y, 2);
  out.print(", omega_z=");
  out.print(omega_z, 2);
  out.print(", tau_x=");
  out.print(tau_x, 2);
  out.print(", tau_y=");
  out.print(tau_y, 2);
  out.print(", tau_z=");
  out.print(tau_z, 2);
  out.print(", mp1=");
  out.print(mp1, 2);
  out.print(", mp2=");
  out.print(mp2, 2);
  out.print(", mp3=");
  out.print(mp3, 2);
  out.print(", mp4=");
  out.print(mp4, 2);
  out.println();
}
