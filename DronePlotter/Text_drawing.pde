void drawText(String text, int x, int y){
  fill(0);
  textSize(32);
  textAlign(LEFT, BASELINE);
  text(text, x, y);  // Centered in the bar
}

void drawVector3(float[] vector, int x, int y, String txt, float color_scale){
  int ww = 160; 
  int hh = 120;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(32);
  textAlign(LEFT, CENTER);
  text(txt, x + 5, y + hh/2);
  
  colorMode(HSB, 1);
  for (int i = 0; i < 3; i ++){
    float c = (vector[i] > 0) ? 0.3 : 1;
    float val = constrain(abs(vector[i])/color_scale, 0 , 0.99);
    //stroke(c, 1, val);
    fill(c, 1, val);
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", vector[i]), x + ww - 5, y + hh/2 + i*32 - 32);
  }
  colorMode(RGB, 1);
}

void drawMotorPercentage(float[] mp, float throttle, int x, int y){
  int ww = 380; 
  int hh = 100;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(32);
  textAlign(CENTER, CENTER);
  String text = String.format("Throttle: %.2f\nMotors: %.2f %.2f %.2f %.2f", throttle, mp[0], mp[1], mp[2], mp[3]);
  text(text, x + ww/2, y + hh/2);  // Centered in the bar
}


void drawSensors(float[] acc, float[] mag, float[] gyro, int x, int y){
  int ww = 280;
  int hh = 180;
  
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(32);
  textAlign(CENTER, CENTER);

  colorMode(HSB, 1);
  fill(1, 1, 0);
  textAlign(LEFT, TOP);
  text("Acc:", x + 10, y);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", acc[i]), x + ww/3, y + 55 + i*40);
  }

  textAlign(LEFT, TOP);
  text("Mag:", x + 10 + ww/3, y);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", mag[i]), x + 2*ww/3 - 5, y + 55 + i*40);
  }
  
  textAlign(LEFT, TOP);
  text("Gyro:", x + 2*ww/3 + 10, y);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", gyro[i]), x + ww - 10, y + 55 + i*40);
  }
  
  colorMode(RGB, 1);
}

void drawStatus(float battery, int numSV, int avg_seq_diff, int x, int y){
  int ww = 160; 
  int hh = 150;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(32);
  textAlign(LEFT, CENTER);
  String text = String.format("Batt: %.2f \nnumSV: %d\nSeqDiff: %d", battery, numSV, avg_seq_diff);
  text(text, x + 5, y + hh/2);  // Centered in the bar
}

void drawMessageCount(long dt, int x, int y){
  String msg_counter = String.format("Total msg: %d", num_messages);
  float update = new_messages * 1000/dt;
  mean_msg = 0.05 * update + 0.95*mean_msg;
  String msg_freq = String.format("msg/s: %.2f ", mean_msg);
  new_messages = 0;
  
  int ww = 280;
  int hh = 100;
  
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle
  
  textAlign(LEFT, CENTER);
  fill(0);
  textSize(32);
  text(msg_counter + "\n" + msg_freq, x + 5, y + hh/2);  // Centered in the bar
}
