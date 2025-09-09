final float ACC_FS = 2.0;
final float MAG_FS = 1.3;
final float GYRO_FS = 720.0;

final float BATT_FS = 16.0;
final float ANGLE_FS = 180.0;
final float FORCE_FS = 200.0;
final float POS_FS = 1000.0;
final float VEL_FS = 10.0;
final float INN_MAG_FS = 0.01;
final float RAD_FS = 2 * PI;

final float MAX_15BIT = 32767.0;
final float MAX_16BIT = 65535.0;
final float MAX_8BIT = 255.0;

void loadValsStateMessage(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [9];  
  ms = buffer_wrapped.getInt() & 0xFFFFFFFFL;  // first entry is a uint32_t ms
  
  for (int i = 0; i < 9; i++){
    signed_vals[i] = buffer_wrapped.getShort();
  }
  
  RPY[0] = float(signed_vals[0]) * (ANGLE_FS / MAX_15BIT);
  RPY[1] = float(signed_vals[1]) * (ANGLE_FS / MAX_15BIT);
  RPY[2] = float(signed_vals[2]) * (ANGLE_FS / MAX_15BIT);
  pos[0] = float(signed_vals[3]) * (POS_FS / MAX_15BIT);
  pos[1] = float(signed_vals[4]) * (POS_FS / MAX_15BIT);
  pos[2] = float(signed_vals[5]) * (POS_FS / MAX_15BIT);
  vel[0] = float(signed_vals[6]) * (VEL_FS / MAX_15BIT);
  vel[1] = float(signed_vals[7]) * (VEL_FS / MAX_15BIT);
  vel[2] = float(signed_vals[8]) * (VEL_FS / MAX_15BIT);  
  battery = buffer_wrapped.getFloat();
  numSV = buffer_wrapped.get() & 0xFF;
  avg_seq_diff = buffer_wrapped.getInt();
}

void loadValsAttitudeMessage(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [12];  
  ms = buffer_wrapped.getInt() & 0xFFFFFFFFL;  // first entry is a uint32_t ms
  
  for (int i = 0; i < 12; i++){
    signed_vals[i] = buffer_wrapped.getShort();
  }
  
  ref_angles[0] = float(signed_vals[0]) * (ANGLE_FS / MAX_15BIT);
  ref_angles[1] = float(signed_vals[1]) * (ANGLE_FS / MAX_15BIT);
  ref_angles[2] = float(signed_vals[2]) * (ANGLE_FS / MAX_15BIT);
  ang_err[0] = float(signed_vals[3]) * (ANGLE_FS / MAX_15BIT);
  ang_err[1] = float(signed_vals[4]) * (ANGLE_FS / MAX_15BIT);
  ang_err[2] = float(signed_vals[5]) * (ANGLE_FS / MAX_15BIT);
  rate_err[0] = float(signed_vals[6]) * (GYRO_FS / MAX_15BIT);
  rate_err[1] = float(signed_vals[7]) * (GYRO_FS / MAX_15BIT);
  rate_err[2] = float(signed_vals[8]) * (GYRO_FS / MAX_15BIT);  
  pid_out[0] = float(signed_vals[9]) * (FORCE_FS / MAX_15BIT);
  pid_out[1] = float(signed_vals[10]) * (FORCE_FS / MAX_15BIT);
  pid_out[2] = float(signed_vals[11]) * (FORCE_FS / MAX_15BIT);
  throttle = float(buffer_wrapped.getShort() & 0xFFFF) * (1.0 / MAX_16BIT);
}

void mix_motors(){
  float[] forces = new float[3];
  forces[0] = constrain(pid_out[0], -60.0, 60.0);
  forces[1] = constrain(pid_out[1], -60.0, 60.0);
  forces[2] = constrain(pid_out[2], -30.0, 30.0);

  motor_percentages[0] = throttle + (forces[0] - forces[1] + forces[2]) / 680.0;
  motor_percentages[1] = throttle + (-forces[0] - forces[1] - forces[2]) / 680.0;
  motor_percentages[2] = throttle + (-forces[0] + forces[1] + forces[2]) / 680.0;
  motor_percentages[3] = throttle + (forces[0] + forces[1] - forces[2]) / 680.0;

  for (int i = 0; i < 4; i++){
    motor_percentages[i] = constrain(motor_percentages[i], 0, 1);
  }
}

void loadValsPositionMessage(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [9];
  
  ms = buffer_wrapped.getInt() & 0xFFFFFFFFL;  // first entry is a uint32_t ms
  
  for (int i = 0; i < 9; i++){
    signed_vals[i] = buffer_wrapped.getShort();
  }
  
  pos_diff[0] = float(signed_vals[0]) * (POS_FS / MAX_15BIT);
  pos_diff[1] = float(signed_vals[1]) * (POS_FS / MAX_15BIT);
  pos_diff[2] = float(signed_vals[2]) * (POS_FS / MAX_15BIT);
  
  vel_diff[0] = float(signed_vals[3]) * (VEL_FS / MAX_15BIT);
  vel_diff[1] = float(signed_vals[4]) * (VEL_FS / MAX_15BIT);
  vel_diff[2] = float(signed_vals[5]) * (VEL_FS / MAX_15BIT);
  
  acc_ref[0] = float(signed_vals[6]) * (ACC_FS / MAX_15BIT);
  acc_ref[1] = float(signed_vals[7]) * (ACC_FS / MAX_15BIT);
  acc_ref[2] = float(signed_vals[8]) * (ACC_FS / MAX_15BIT);
}

void loadValsEKFMessage(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [12];  
  ms = buffer_wrapped.getInt() & 0xFFFFFFFFL;  // first entry is a uint32_t ms
  
  for (int i = 0; i < 12; i++) signed_vals[i] = buffer_wrapped.getShort();
  
  RPY[0] = float(signed_vals[0]) * (ANGLE_FS / MAX_15BIT);
  RPY[1] = float(signed_vals[1]) * (ANGLE_FS / MAX_15BIT);
  RPY[2] = float(signed_vals[2]) * (ANGLE_FS / MAX_15BIT);
  acc[0] = float(signed_vals[3]) * (ACC_FS / MAX_15BIT);
  acc[1] = float(signed_vals[4]) * (ACC_FS / MAX_15BIT);
  acc[2] = float(signed_vals[5]) * (ACC_FS / MAX_15BIT);
  mag[0] = float(signed_vals[6]) * (MAG_FS / MAX_15BIT);
  mag[1] = float(signed_vals[7]) * (MAG_FS / MAX_15BIT);
  mag[2] = float(signed_vals[8]) * (MAG_FS / MAX_15BIT);  
  gyro[0] = float(signed_vals[9]) *  (GYRO_FS / MAX_15BIT);
  gyro[1] = float(signed_vals[10]) * (GYRO_FS / MAX_15BIT);
  gyro[2] = float(signed_vals[11]) * (GYRO_FS / MAX_15BIT);  
  inn_mag = float(buffer_wrapped.getShort() & 0xFFFF) * (INN_MAG_FS / MAX_16BIT);
}

void loadValsQuattitudeMessage(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [12];  
  ms = buffer_wrapped.getShort() & 0xFFFF;  // first entry is a uint16_t ms
  
  for (int i = 0; i < 12; i++) signed_vals[i] = buffer_wrapped.getShort();
  
  ref_angles[0] = float(signed_vals[0]) * (ANGLE_FS / MAX_15BIT);
  ref_angles[1] = float(signed_vals[1]) * (ANGLE_FS / MAX_15BIT);
  ref_angles[2] = float(signed_vals[2]) * (ANGLE_FS / MAX_15BIT);
  i_err[0] = float(signed_vals[3]) * (RAD_FS / MAX_15BIT);
  i_err[1] = float(signed_vals[4]) * (RAD_FS / MAX_15BIT);
  i_err[2] = float(signed_vals[5]) * (RAD_FS / MAX_15BIT);
  omega_des[0] = float(signed_vals[6]) * (GYRO_FS / MAX_15BIT);
  omega_des[1] = float(signed_vals[7]) * (GYRO_FS / MAX_15BIT);
  omega_des[2] = float(signed_vals[8]) * (GYRO_FS / MAX_15BIT);  
  tau[0] = float(signed_vals[9]) *  (RAD_FS / MAX_15BIT);
  tau[1] = float(signed_vals[10]) * (RAD_FS / MAX_15BIT);
  tau[2] = float(signed_vals[11]) * (RAD_FS / MAX_15BIT);  
  motor_percentages[0] = float(buffer_wrapped.get() & 0xFF) * (1 / MAX_8BIT);
  motor_percentages[1] = float(buffer_wrapped.get() & 0xFF) * (1 / MAX_8BIT);
  motor_percentages[2] = float(buffer_wrapped.get() & 0xFF) * (1 / MAX_8BIT);
  motor_percentages[3] = float(buffer_wrapped.get() & 0xFF) * (1 / MAX_8BIT);
}
