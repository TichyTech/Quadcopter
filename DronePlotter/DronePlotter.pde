import processing.opengl.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

long last_draw = 0;

Serial sp;
byte[] buff = new byte[512];
long ms;

// state message variables
float[] RPY = new float[3];
float[] pos = new float[3];
float[] vel = new float[3];
float battery = 0;
int numSV = 0;
int avg_seq_diff = 0;

// attitude message variables
float[] ref_angles = new float[3];
float[] ang_err = new float[3];
float[] rate_err = new float[3];
float[] pid_out = new float[3];
float throttle = 0;

float[] motor_percentages = new float[4];

// position message variables
float[] pos_diff = new float[3];
float[] vel_diff = new float[3];
float[] acc_ref = new float[3];

// EKF message variables
float[] acc = new float[3];
float[] mag = new float[3];
float[] gyro = new float[3];
float inn_mag = 0;

// Quattitude message variables
float [] i_err = new float[3];
float [] omega_des = new float[3];
float [] tau = new float[3];

// message statistics
long num_messages = 0;
long new_messages = 0;
float mean_msg = 0;

// Command line variables
String inputText = "";        // Store the current input text
String lastInputText = "";    // Store the last input text (saved text)
int cursorPos = 0;          // Position of the cursor in the input text

// Logging structure 
StateDatapoints state_database;
EKFDatapoints ekf_database;
AttitudeDatapoints att_database;
QuattitudeDatapoints quat_database;

int serialPort = 2;
 
void setup() {
  size(1440, 1080, P3D);
  surface.setTitle("Drone Plotter");
  surface.setLocation(100, 100);
  colorMode(RGB, 1); 
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  printArray(Serial.list());
  if (serialPort >= Serial.list().length){
    println("serial index out of range, exiting");
    exit();
    return;
  }
  String portName = Serial.list()[serialPort];
  sp = new Serial(this, portName, 500000);
  println("Connecting to " + portName);
  
  state_database = new StateDatapoints(2048);
  ekf_database = new EKFDatapoints(2048);
  att_database = new AttitudeDatapoints(2048);
  quat_database = new QuattitudeDatapoints(2048);
  wait_for_setup();
}
   
int numBytes = 0;

void draw() {
  
  handle_serial_gibberish();
  
  long current_millis = millis();
  long dt = current_millis - last_draw;
  if (dt > 15){
    background(0.5,0.5,0.5); 
    directionalLight(1, 1, 1, 0, 0, -1);
    pushMatrix();
    translate(width/2, height/2);
    scale(1, -1, 1); 
    rotateX(-PI/2);
    rotateX(0.1);
    rotateZ(0.1);
    drawAxes(300);
    drawRPYm(RPY, motor_percentages, 0.9);
    drawRef(ref_angles, 0.2);
    popMatrix();
    drawVector3(ref_angles, 50, 20, "Ref:", 10.0);
    drawVector3(RPY, 50, 140, "RPY:", 20.0);
    
    drawVector3(ang_err, 50, 260, "ang_e: ", 15.0);
    drawVector3(rate_err, 50, 380, "w_e: ", 50.0);
    drawVector3(pid_out, 50, 500, "f:", 50.0);
    
    drawVector3(i_err, 210, 260, "i_err: ", 4);
    drawVector3(omega_des, 210, 380, "w_d: ", 2.0);
    drawVector3(tau, 210, 500, "tau:", 0.5);
    
    drawVector3(pos, 210, 20, "pos: ", 5.0);
    drawVector3(vel, 210, 140, "vel: ", 1.0);
    
    drawVector3(pos_diff, 370, 20, "posd: ", 5.0);
    drawVector3(vel_diff, 370, 140, "veld: ", 1.0);
    drawVector3(acc_ref, 370, 260, "aref:", 1.0);
    
    drawMotorPercentage(motor_percentages, throttle, 50, 620);
    
    drawXYZ(ekf_database.mss, ekf_database.gyros, width - width/3, height - height/3, width/3, height/3, 50, ekf_database.start, ekf_database.items, "Gyro");
    drawXYZ(ekf_database.mss, ekf_database.accs, 0, height - height/3, width/3, height/3, 2, ekf_database.start, ekf_database.items, "Acc");
    drawXYZ(ekf_database.mss, ekf_database.mags, width/2 - width/6, height - height/3, width/3, height/3, 0.5, ekf_database.start, ekf_database.items, "Mag");

    plot_diff(pos_diff, width-350, height - height/3 - 900, "Pos Diff ");
    plot_diff(vel_diff, width-350, height - height/3 - 600, "Vel Diff ");
    plot_diff(acc_ref, width-350,  height - height/3 - 300, "Acc Ref ");

    drawStatus(battery, numSV, avg_seq_diff, width - 160, 0);
    drawMessageCount(dt, width - 440, 0);
    drawCommandLine(20, height - 50);
    
    last_draw = current_millis;
    }
}
