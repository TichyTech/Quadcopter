#include "peripherals.h"

#define JOY_FS 15.0f
#define ADC_FS 1023.0f


Inputs::Inputs(){

  setup_pins();

  button1_state = 1;
  button2_state = 1;

  pot_reading = 0;

  starting_joy = {504, 504, 503, 520};  // center point for my joysticks (not quite center of the scale)
  current_joy = {0,0,0,0};
}

void Inputs::update_readings(){

  current_joy = read_joysticks();
  pot_reading = (analogRead(POT_PIN))/ADC_FS;
  read_buttons();

}

void Inputs::read_buttons(){
// reset risings of buttons
  button1_rising = 0;
  button2_rising = 0;

  if (digitalRead(BUTTON_L_PIN) && !button1_state){ 
    button1_state = 1;
    button1_rising = 1;
  }
  if (!digitalRead(BUTTON_L_PIN) && button1_state){
    button1_state = 0;
    if (DEBUG) Serial.println("Left button pressed");
  }
  
  if (digitalRead(BUTTON_R_PIN) && !button2_state){ 
    button2_state = 1;
    button2_rising = 1;
  }
  if (!digitalRead(BUTTON_R_PIN) && button2_state){
    button2_state = 0;
    if (DEBUG) Serial.println("Right button pressed");
  }
}

Joysticks Inputs::read_joysticks(){
  Joysticks reading = {0,0,0,0};
  reading.X = analogRead(JOYLX_PIN);
  reading.Y = analogRead(JOYLY_PIN);
  // flip the right joystick as it is rotated 
  reading.X2 = 1023 - analogRead(JOYRX_PIN);
  reading.Y2 = 1023 - analogRead(JOYRY_PIN);
  return reading;
}

Joysticks Inputs::map_joysticks(Joysticks readings){
  Joysticks out = {0,0,0,0};
  float mult = JOY_FS / (0.5f * ADC_FS);  // remapping to +- JOY_FS fullscale
  out = readings.subtract(starting_joy).multiply(mult);  // should be (roughly) +- JOY_FS

  float min_val = 0.5;  // deadband
  if ( abs(out.X) < min_val) out.X = 0; 
  if ( abs(out.Y) < min_val) out.Y = 0; 
  if ( abs(out.X2) < min_val) out.X2 = 0; 
  if ( abs(out.Y2) < min_val) out.Y2 = 0;
  return out;
}

Joysticks Inputs::smooth_joysticks(Joysticks readings, float coeff){
  static Joysticks out = readings;
  out = out.multiply(coeff).add(readings.multiply(1.0f - coeff));  // convex combination of readings and out
  return out;
}

void Inputs::setup_pins(){
  pinMode(POT_PIN, INPUT);
  pinMode(JOYLB_PIN, INPUT);
  pinMode(JOYLY_PIN, INPUT);
  pinMode(JOYLX_PIN, INPUT);
  pinMode(JOYRY_PIN, INPUT);
  pinMode(JOYRX_PIN, INPUT);
  pinMode(BUTTON_L_PIN, INPUT);
  pinMode(BUTTON_R_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(WSLED_PIN, OUTPUT);
}