void wait_for_setup(){
  while (sp.available() == 0){
    println("Waiting for Setup");
    delay(300);
  }  // wait for serial
  String mystr = new String();
  while (true){  // wait for finishing setup of remote controller
    if (sp.available() > 0){
      int numBytes = sp.readBytesUntil('\n', buff); // try reading bytes from serial
      mystr = (new String(buff, 0, numBytes)).trim();
      println(mystr);
    }
    if (mystr.equals("Setup done")) break;
    delay(20);
  }
  println("Setup done!");
}

void handle_serial_gibberish(){
  byte [] buffer = new byte [32];
  if (sp.available() >= 32){
    numBytes = sp.readBytes(buffer); // try reading bytes from serial
  }
  while (numBytes > 0){
    ByteBuffer buffer_wrapped = ByteBuffer.wrap(buffer);
    buffer_wrapped.order(ByteOrder.LITTLE_ENDIAN); // Match endianness of C++ side
    int msg_type = buffer_wrapped.get() & 0xFF;
    if(msg_type == 3){    
      loadValsStateMessage(buffer_wrapped); 
      state_database.add_datapoint(RPY, pos, vel, ref_angles, battery, ms);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 4){
      loadValsAttitudeMessage(buffer_wrapped);
      mix_motors();
      att_database.add_datapoint(ref_angles, ang_err, rate_err, pid_out, throttle, ms);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 5){
      loadValsPositionMessage(buffer_wrapped);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 6){
      loadValsEKFMessage(buffer_wrapped);
      ekf_database.add_datapoint(RPY, acc, mag, gyro, inn_mag, ms);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 7){
      loadValsQuattitudeMessage(buffer_wrapped);
      quat_database.add_datapoint(ref_angles, omega_des, i_err, tau, motor_percentages, ms);
      num_messages++;
      new_messages++;
    }
    if (sp.available() >= 32){
      numBytes = sp.readBytes(buffer); // try reading bytes from serial
    }
    else numBytes = 0;
  }
}
