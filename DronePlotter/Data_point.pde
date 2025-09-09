class StateDatapoints{
  long [] mss;
  float [][] RPYs;
  float [][] poss;
  float [][] vels;
  float [][] refs;
  float [] batts;
  
  int start = 0;
  int items = 0;
  int len;
  
  StateDatapoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.poss = new float [length][3];
    this.vels = new float [length][3];
    this.refs = new float[length][3];
    this.batts = new float [length];
    this.mss = new long [length];
  }
  
  void add_datapoint(float [] RPY, float [] pos, float[] vel, float[] ref, float batt, long ms){
    arrayCopy(RPY, this.RPYs[start]);
    arrayCopy(pos, this.poss[start]);
    arrayCopy(vel, this.vels[start]);
    arrayCopy(ref, this.refs[start]);
    this.batts[start] = batt;
    this.mss[start] = ms;
    start = (start + 1) % this.len;  // move start index instead
    
    if (this.items < len) this.items++;
  }
  
  void save_to_file(){
    if (this.items == 0) return;
    float curr_minute = minute();
    float curr_sec = second();
    PrintWriter writer = createWriter("Logs/"+ str(curr_minute) + "_" + str(curr_sec) + "/StateLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_RPY = this.RPYs[idx];
      float [] c_pos = this.poss[idx];
      float [] c_vel = this.vels[idx];
      float [] c_ref = this.refs[idx];
      float c_batt = this.batts[idx];
      long c_ms = this.mss[idx];
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.3f %.3f %.3f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String pos_string = String.format("%.3f %.3f %.3f ", c_pos[0], c_pos[1], c_pos[2]);
      String vel_string = String.format("%.3f %.3f %.3f ", c_vel[0], c_vel[1], c_vel[2]);
      String ref_string = String.format("%.3f %.3f %.3f ", c_ref[0], c_ref[1], c_ref[2]);
      String batt_string = String.format("%.2f", c_batt);
      writer.println(ms_string + RPY_string + pos_string + vel_string + ref_string + batt_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to StateLog.txt");
  }
}

class AttitudeDatapoints{
  long [] mss;
  float [][] refs;
  float [][] rpy_errs;
  float [][] rate_errs;
  float [][] pid_outs;
  float [] throttles;
  
  int start = 0;
  int items = 0;
  int len;
  
  AttitudeDatapoints(int length){
    this.len = length;
    this.refs = new float [length][3];
    this.rpy_errs = new float [length][3];
    this.rate_errs = new float [length][3];
    this.pid_outs = new float[length][3];
    this.throttles = new float [length];
    this.mss = new long [length];
  }
  
  void add_datapoint(float [] ref, float [] rpy_err, float[] rate_err, float[] pid_out, float throttle, long ms){
    arrayCopy(ref, this.refs[start]);
    arrayCopy(rpy_err, this.rpy_errs[start]);
    arrayCopy(rate_err, this.rate_errs[start]);
    arrayCopy(pid_out, this.pid_outs[start]);
    this.throttles[start] = throttle;
    this.mss[start] = ms;
    start = (start + 1) % this.len;  // move start index instead
    
    if (this.items < len) this.items++;
  }
  
  void save_to_file(){
    if (this.items == 0) return;
    float curr_minute = minute();
    float curr_sec = second();
    PrintWriter writer = createWriter("Logs/"+ str(curr_minute) + "_" + str(curr_sec) + "/AttitudeLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_ref = this.refs[idx];
      float [] c_rpy_err = this.rpy_errs[idx];
      float [] c_rate_err = this.rate_errs[idx];
      float [] c_pid_out = this.pid_outs[idx];
      float c_th = this.throttles[idx];
      long c_ms = this.mss[idx];
      String ms_string = String.format("%d ", c_ms);
      String string0 = String.format("%.3f %.3f %.3f ", c_ref[0], c_ref[1], c_ref[2]);
      String string1 = String.format("%.3f %.3f %.3f ", c_rpy_err[0], c_rpy_err[1], c_rpy_err[2]);
      String string2 = String.format("%.3f %.3f %.3f ", c_rate_err[0], c_rate_err[1], c_rate_err[2]);
      String string3 = String.format("%.3f %.3f %.3f ", c_pid_out[0], c_pid_out[1], c_pid_out[2]);
      String string4 = String.format("%.2f", c_th);
      writer.println(ms_string + string0 + string1 + string2 + string3 + string4);
    }
    writer.close();  // Always close the writer
    println("Saved log to AttitudeLog.txt");
  }
}

class EKFDatapoints{
  float [][] RPYs;
  float [][] accs;
  float [][] mags;
  float [][] gyros;
  float [] inn_mags;
  long [] mss;
  
  int start = 0;
  int items = 0;
  int len;
  
  EKFDatapoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.accs = new float [length][3];
    this.mags = new float [length][3];
    this.gyros = new float [length][3];
    this.inn_mags = new float[length];
    this.mss = new long [length];
  }
  
  void add_datapoint(float[] RPY, float[] acc, float[] mag, float[] gyro, float inn_mag, long ms){
      arrayCopy(RPY, this.RPYs[start]);
      arrayCopy(acc, this.accs[start]);
      arrayCopy(mag, this.mags[start]);
      arrayCopy(gyro, this.gyros[start]);
      this.inn_mags[start] = inn_mag;
      this.mss[start] = ms;
      start = (start + 1) % this.len;  // move start index instead
      
      if (items < len) items++;
  }
  
  void save_to_file(){
    if (this.items == 0) return;
    float curr_minute = minute();
    float curr_sec = second();
    PrintWriter writer = createWriter("Logs/" + str(curr_minute) + "_" + str(curr_sec) + "/EKFLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_acc = this.accs[idx];
      float [] c_mag = this.mags[idx];
      float [] c_gyro = this.gyros[idx];
      float [] c_RPY = this.RPYs[idx];
      float c_inn = this.inn_mags[idx];
      long c_ms = this.mss[idx];
      
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.3f %.3f %.3f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String acc_string = String.format("%.3f %.3f %.3f ", c_acc[0], c_acc[1], c_acc[2]);
      String mag_string = String.format("%.3f %.3f %.3f ", c_mag[0], c_mag[1], c_mag[2]);
      String gyro_string = String.format("%.3f %.3f %.3f ", c_gyro[0], c_gyro[1], c_gyro[2]);
      String inn_string = String.format("%.5f", c_inn);
      writer.println(ms_string + RPY_string + acc_string + mag_string + gyro_string + inn_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to EKFLog.txt");
  }
  
}

class QuattitudeDatapoints{
  float [][] RPYs;
  float [][] i_errs;
  float [][] des_omegas;
  float [][] taus;
  float [][] mps;
  long [] mss;
  
  int start = 0;
  int items = 0;
  int len;
  
  QuattitudeDatapoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.i_errs = new float [length][3];
    this.des_omegas = new float [length][3];
    this.taus = new float [length][3];
    this.mps = new float[length][4];
    this.mss = new long [length];
  }
  
  void add_datapoint(float[] RPY, float[] omega_des, float[] i_err, float[] tau, float[] mp, long ms){
      arrayCopy(RPY, this.RPYs[start]);
      arrayCopy(i_err, this.i_errs[start]);
      arrayCopy(omega_des, this.des_omegas[start]);
      arrayCopy(tau, this.taus[start]);
      arrayCopy(mp, this.mps[start]);
      this.mss[start] = ms;
      start = (start + 1) % this.len;  // move start index instead
      
      if (items < len) items++;
  }
  
  void save_to_file(){
    if (this.items == 0) return;
    float curr_minute = minute();
    float curr_sec = second();
    PrintWriter writer = createWriter("Logs/" + str(curr_minute) + "_" + str(curr_sec) + "/QuattitudeLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_RPY = this.RPYs[idx];
      float [] c_i_err = this.i_errs[idx];
      float [] c_omega = this.des_omegas[idx];
      float [] c_tau = this.taus[idx];
      float [] c_mp = this.mps[idx];
      long c_ms = this.mss[idx];
      
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.3f %.3f %.3f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String omega_string = String.format("%.3f %.3f %.3f ", c_omega[0], c_omega[1], c_omega[2]);
      String i_err_string = String.format("%.3f %.3f %.3f ", c_i_err[0], c_i_err[1], c_i_err[2]);
      String tau_string = String.format("%.3f %.3f %.3f ", c_tau[0], c_tau[1], c_tau[2]);
      String mp_string = String.format("%.3f %.3f %.3f %.3f", c_mp[0], c_mp[1], c_mp[2], c_mp[3]);
      writer.println(ms_string + RPY_string + omega_string + i_err_string + tau_string + mp_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to QuattitudeLog.txt");
  }
}
