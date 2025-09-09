#include "motors.h"

// MOTOR NUMBERING DIAGRAM
// TOP VIEW
// FRONT
//---------
//        X
//        ^
//        |
//
//     1     2 
//      \ | /
// <-Y   |||    
//      / | \ 
//     4     3 

// useful definitions

const uint8_t MOTORS_PINS[] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
uint32_t CYCLES_PER_US = 150;
PIO pio = pio0;
uint pio_offset;


void mount_motors(){
  Serial.println("Mounting motors");
  // enable motor pins as outputs
  for (int i=0; i<4; i++) {
    gpio_init(MOTORS_PINS[i]);    // pico SDK code
    gpio_set_dir(MOTORS_PINS[i], GPIO_OUT);
  }  
}

void setup_motor_pio(){
  Serial.println("Setting up PIO");
  // CYCLES_PER_US = clock_get_hz(clk_sys)/1000000;  // needs to get set at runtime
  // initialize pio and all 4 state machines in it for motor control using the oneshot125 protocol
  pio_offset = pio_add_program(pio, &oneshot_program);  // load pio program (shared between all 4 state machines in this pio)
  for (int i = 0; i < 4; i++){
    oneshot_program_init(pio, i, pio_offset, MOTORS_PINS[i]);  // initialize registers in all state machines
    pio_sm_set_enabled(pio, i, true);  // enable sm
  }
}

void pulse_sm(uint8_t sm, uint32_t on_t, uint32_t off_t){
  // This puts on and off cycle counts to the specified state machine (sm), each sm controls one motor pin
  pio_sm_put_blocking(pio, sm, on_t);
  pio_sm_put_blocking(pio, sm, off_t);  // immediately after this call, the sm puts 1 onto its specified pin for on_t cycles and than 0 for at least off_t cycles.
}

void signal_motors(Vector4 mp){
  for (int i = 0; i < 4; i++){
    // uint32_t on_cycles = floor(CYCLES_PER_US*(125 + mp(i)*125)); // get time period between 125 and 250 us corresponding to 0 and 100 throttle. Then convert to cycle count.
    // uint32_t on_cycles = floor(CYCLES_PER_US*(41.0f + mp(i)*42));  //  oneshot42
    uint32_t on_cycles = floor(CYCLES_PER_US*(4.5f + mp(i)*20));  // multishot
    pulse_sm(i, on_cycles, 0);  // put logical 1 onto given motor pin for on_cycles cycles. Use off_cycle equal to 0 to avoid extra delay.
  }
}

void arm_motors(){
  Serial.println("Arming motors");
  Vector4 arm_percentages = {1, 1, 1, 1};
  for (int i = 0; i < 200; i++){
    signal_motors(arm_percentages*0.0f);
    delay(1);
  }

  const int iter = 300;
  for (int i = 0; i < 300; i++){
    signal_motors(arm_percentages*0.8f*float(i/iter));
    delay(1);
  }
  
  for (int i = 0; i < 200; i++){
    signal_motors(arm_percentages*0.0f);
    delay(1);
  }
}
