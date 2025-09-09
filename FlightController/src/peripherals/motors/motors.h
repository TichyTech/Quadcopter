#ifndef MOTORS
#define MOTORS

#include "oneshot.pio.h"
#include <hardware/pio.h>
#include "definitions.h"
#include "config.h"

void mount_motors();
void setup_motor_pio();
void pulse_sm(uint8_t sm, uint32_t on_t, uint32_t off_t);
void signal_motors(Vector4 mp);
void arm_motors();

#endif