#include <Arduino.h>
#include <FastLED.h>

#include "comms.h"
#include "serial_comm.h"
#include <comm_structs.h>
#include "config.h"
#include "peripherals.h"

CRGB leds[1];
Inputs input_manager = Inputs();
Communication Comms = Communication();

void setup() {
  
  FastLED.addLeds<WS2812B, WSLED_PIN, GRB>(leds, 1);  // GRB ordering is typical
  Serial.begin(500000);
  while (!Serial){}
  Comms.setup_nrf();

  Serial.println("Setup done");
}

uint8_t motors_mode = 1;
uint8_t motors_on = 0;
Joysticks smooth_joys = {0,0,0,0};
float yaw_diff_setpoint = 0;

uint32_t last_cmd_t = 0;
uint32_t last_telem_t = 0;
uint32_t sequence_num = 0;

void loop() {  

  // indicate telemetry quality with LED brightness
  int32_t telem_delay = millis() - last_telem_t;
  float led_brightness = min(1, max(500 - telem_delay, 0)/500.0f);
  analogWrite(LED1_PIN, int(led_brightness * 255));  // green led indicates signal quality
  
  if ((millis() - last_cmd_t) >= 50){  // send control every 50ms
    // update joystick readings
    input_manager.update_readings();
    if (input_manager.button2_rising) motors_on = !motors_on;
    if (input_manager.button1_rising) motors_mode = 1 + (motors_mode) % 2;
    Joysticks joys = input_manager.current_joy;
    Joysticks mapped_joys = input_manager.map_joysticks(joys);

    // led control
    leds[0] = CHSV(51*(motors_mode-1), 255, int(motors_on*(50 + 80*(input_manager.pot_reading))));
    FastLED.show();

    // compute and send control
    // smooth_joys = input_manager.smooth_joysticks(mapped_joys, 0.7);
    if (abs(mapped_joys.X) < 1) mapped_joys.X = 0;  // discard small angles
    yaw_diff_setpoint -= mapped_joys.X;
    float alt_diff = mapped_joys.Y/100.0f;
    float rpy[3] = {mapped_joys.X2, mapped_joys.Y2, yaw_diff_setpoint};
    ControlMessage ctrl_message(rpy, input_manager.pot_reading, alt_diff, motors_on*motors_mode, ++sequence_num);
    ctrl_message.serialize();
    Comms.send_msg(ctrl_message);
    last_cmd_t = millis();
    if (DEBUG) Serial.println("cmd out"); 
    if (PRINT_COMMANDS) ctrl_message.printTo(Serial);
  }

  // parse config settings from serial and send
  Message msg; 
  if ((millis() - last_cmd_t) >= 40){
    if (parse_serial(msg)){
      Comms.send_msg(msg);
      if (DEBUG) {
        Serial.print("cfg out: ");
        ConfigMessage(msg.data).printTo(Serial);
      }
    }
  }

  // receive message from drone and print to serial 
  Message received_msg;
  if (Comms.receive_msg(received_msg)){
    last_telem_t = millis();
    if (DEBUG) Serial.print("msg in ");
    if (STATE2SERIAL) print_message_to_serial(received_msg);  // send to processing for rendering
  }

}