#include "config.h"
#include <Arduino.h> 
#include "serial_comm.h"

struct Joysticks{
    float X;
    float Y;
    float X2;
    float Y2;

    Joysticks add(const Joysticks& other) const {
        return {X + other.X, Y + other.Y, X2 + other.X2, Y2 + other.Y2};
    }

    Joysticks multiply(float scalar) const {
        return {X * scalar, Y * scalar, X2 * scalar, Y2 * scalar};
    }

    Joysticks subtract(const Joysticks& other) const {
        return {X - other.X, Y - other.Y, X2 - other.X2, Y2 - other.Y2};
    }

    void print() const {
        Serial.print("(");
        Serial.print(X);
        Serial.print(", ");
        Serial.print(Y);
        Serial.print(", ");
        Serial.print(X2);
        Serial.print(", ");
        Serial.print(Y2);
        Serial.print(")");
    }
};

class Inputs{

    public:

        const Joysticks starting_joy;
        Joysticks current_joy;  // joystick state variable

        float pot_reading;
        uint8_t button1_state;
        uint8_t button2_state;
        bool button1_rising;
        bool button2_rising;

        Inputs();
        void update_readings();
        Joysticks read_joysticks();
        Joysticks map_joysticks(Joysticks readings);
        Joysticks smooth_joysticks(Joysticks readings, float coeff);
        void read_buttons();
        void setup_pins();
};