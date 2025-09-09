#include <Arduino.h>

#include "config.h"
#include "definitions.h"
#include "debug/debugging.h"
#include "peripherals/peripherlas.h"
#include "peripherals/sensors.h"
#include "peripherals/comms/NRF24.h"
#include "peripherals/gps/M100-5883.h"

#include "control/attitude_controller.h"
#include "control/quat_controller/quat_controller.h"
#include "control/position_controller.h"
#include "control/filter.h"
#include "control/kalman_filter/kalman.h"
#include "peripherals/motors/motors.h"

#include "peripherals/mediator.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////

Temperatures measured_temps; // struct to hold all measured temperatures
Measurements measured_values;  // struct to store all measured values
Matrix3 estimated_DCM;  // matrix to store current estimated attitude
Vector4 q;  // current attitude quaternion
State current_state; // full drone state
AttReference att_ref;
Vector4 q_ref;  // quaternion attitude reference
Vector4 control_action = {0,0,0,0}; 

uint32_t start_time = 0;
float initial_yaw = 0;

Sensors sensors = Sensors();
Communication comm = Communication(SPI); 
AttController att_controller = AttController();
QuatController quat_controller = QuatController();
KalmanFilter k_filter = KalmanFilter();

GPS gps = GPS();
PosController pos_controller = PosController();
Vector3 pos_controller_output = {0,0,0};

Mediator mediator(att_controller, quat_controller, pos_controller, comm);

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  initialize_peripherals();  // initialize SPI, I2C, Serial, pins
  sensors.setup();  // configure sensors, blocking if not successful
  comm.setup_nrf();  // set up nrf24, blocking if not successful
  gps.setup_gps();

  mount_motors();  // mount motor pins
  setup_motor_pio();  // program and initialize pio state machines for motor control
  arm_motors();  // send non-zero throttle to the motors to arm them

  measured_values = sensors.get_measurements_filtered();

  while (sensors.timed_out){
    Serial.println("IMU timeout");
    delay(500);
  }

  // // start of the control loop
  estimated_DCM = acc_mag2DCM(measured_values); // init DCM
  k_filter.init_quat(estimated_DCM);  // correct initialization of quaternion from DCM matrix
  current_state.rpy = DCM2RPY(estimated_DCM);
  initial_yaw = current_state.rpy(2);  // store initial yaw for control
  start_time = millis();
  Serial.println("Setup done");
}

void setup1(){
  while(!start_time){  // wait for setup on the first core to finish
      delay(200);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool loop_telemetry_sent = false;
bool control_telemetry_sent = false;
bool position_hold = false;
Vector3 hold_coordinates = {0,0,0};

void loop() {
    AttReference current_att_ref = att_ref;
    uint8_t motors_on = current_att_ref.motors_on;
    measured_values = sensors.get_measurements_filtered();  // update measurements from gyro, acc, mag and alt (and dt)
    // measured_temps = sensors.get_temperatures();

    if (SAFETY) if (sensors.timed_out || comm.comm_timed_out) motors_on = 0;

    digitalWrite(REDLED_PIN, (sensors.timed_out || comm.comm_timed_out) ? HIGH : LOW);  // light up red if sensors or comms timed out
    digitalWrite(GREENLED_PIN, motors_on > 0 ? HIGH : LOW);  // light up green if motors run

    mediator.update_configs();  // handle new PID config sent from controller

    q = k_filter.predict(measured_values.gyro_vec, measured_values.integration_period);  // model prediction using gyro
    k_filter.track_acc(measured_values.acc_vec, measured_values.integration_period);  // track accelerometer magnitude

    Vector3 unit_acc = normalize(measured_values.acc_vec);
    q = k_filter.fuse_acc(unit_acc);  // fuse accelerometer data

    Vector3 unit_mag = normalize(measured_values.mag_vec);  
    Vector3 est_up = estimated_DCM.Column(2);  // best guess of UP direction
    Vector3 perp_mag = unit_mag - dot(unit_mag, est_up)*est_up;  // horizontal component of magnetic vector
    q = k_filter.fuse_mag(normalize(perp_mag));  // fuse mag

    estimated_DCM = quat2R(q);  // quat to DCM conversion
    current_state.rpy = DCM2RPY(estimated_DCM);
    current_state.omega = measured_values.gyro_vec - k_filter.b; 
    float max_val = k_filter.clamp_variance();  // reduce variance if too big

    bool update = gps.my_update_gps();
    if(update){  // gps data ready
      current_state.pos = gps.get_NWU_pos();
      current_state.vel = gps.get_NWU_speed();
      if (!position_hold && motors_on == 2){
          hold_coordinates = current_state.pos;  // remember the current position#
          position_hold = true;
      }
      if (position_hold && motors_on != 2) position_hold = false;
      if (position_hold){
        pos_controller_output = pos_controller.process(hold_coordinates, current_state, measured_values.integration_period);
        att_ref.roll += pos_controller_output(0);
        att_ref.pitch += pos_controller_output(1);
        att_ref.throttle += 0.01f*pos_controller_output(2);
        control_telemetry_sent = false;
      }
    }
    else if (millis() - gps.last_fix_timestamp > 500) position_hold = false;

    // control_action = att_controller.process(att_ref, current_state, measured_values.integration_period, measured_values.battery); 
    q_ref = rpy2quat({att_ref.roll, att_ref.pitch, att_ref.yaw});
    control_action = quat_controller.process(q_ref, q, measured_values.gyro_vec, att_ref.throttle, measured_values.integration_period);
    if (motors_on == 0) signal_motors(zero_4vector); 
    else signal_motors(control_action);

    analogWrite(EXT_LOAD0_PIN, int(att_ref.throttle * 255));
    analogWrite(EXT_LOAD1_PIN, int(att_ref.throttle * 255));  
    loop_telemetry_sent = false;  // set flag for sending telemetry
}

void loop1() {
  att_ref = comm.update_commands(initial_yaw);  // poll for new commands
  State state_copy = current_state; 

  #if RADIO_TELEMETRY
    if ((millis() - comm.last_ctrl_ms) <= 42) // less than 42 milliseconds from last command ==> we can send telemetry
    { 
      // the nrf24 telemetry takes about 0.9 ms per message
      if (!loop_telemetry_sent){
        comm.send_msg(comm.create_state_msg(state_copy, measured_values.battery, gps.num_sv, comm.avg_diff));
        comm.send_msg(comm.create_ekf_msg(state_copy, measured_values, k_filter.last_inn_mag));
        // comm.send_msg(comm.create_attitude_msg(att_controller.last_reference, att_controller.last_ang_err, att_controller.last_pid_err, att_controller.last_PID_outputs));
        comm.send_msg(comm.create_quattitude_msg(att_ref, quat_controller.omega_des, quat_controller.err_sum, quat_controller.tau, quat_controller.m_p));
        loop_telemetry_sent = true;  // only send once per control loop
      }
      if (!control_telemetry_sent){
          comm.send_msg(comm.create_pos_msg(pos_controller.last_pos_dif, pos_controller.last_vel_diff, pos_controller_output));
          control_telemetry_sent = true;
      }
    }
  #endif
  

  #if SERIAL_TELEMETRY
    if (!loop_telemetry_sent){
      Message msg = comm.create_state_msg(state_copy, measured_values.battery, gps.num_sv, comm.avg_diff);
      Serial.write((uint8_t*)&msg.data, 32);  // send the message to the controller
      msg = comm.create_ekf_msg(state_copy, measured_values, k_filter.last_inn_mag);
      Serial.write((uint8_t*)&msg.data, sizeof(msg));  // send the message to the controller
      // msg = comm.create_attitude_msg(att_controller.last_reference, att_controller.last_ang_err, att_controller.last_pid_err, att_controller.last_PID_outputs);
      // Serial.write((uint8_t*)&msg.data, 32);
      msg = comm.create_quattitude_msg(att_controller.last_reference, quat_controller.last_axis, quat_controller.omega_des, quat_controller.tau, quat_controller.m_p);
      Serial.write((uint8_t*)&msg.data, 32);
      loop_telemetry_sent = true;  // only send once per control loop
    }
    delay(16);
  #endif

}