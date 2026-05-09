#include "config.h"
#include "peripherals/register_handling.h"
#include "algebra.h"
#include "debug/debugging.h"

class AccMag{
    private:
        Vector3 last_acc_vec;
        Vector3 filtered_acc_vec;
        Vector3 last_mag_vec;
        Vector3 filtered_mag_vec;

        Vector3 acc_bias;
        Vector3 mag_bias;
        Matrix3 mag_scale;

        unsigned long last_acc_timestamp;
        unsigned long last_mag_timestamp;
        unsigned long last_temp_timestamp;

        TwoWire &_wire;

    public:
        bool acc_timeout;
        bool mag_timeout;
        bool temp_timeout;

        AccMag(TwoWire &wire = Wire);

        void calibrate_acc();
        void setup_acc();
        void setup_mag();
        Vector3 read_acc();
        Vector3 read_mag();
        Vector3 get_filtered_acc();
        Vector3 get_filtered_mag();
        float read_temp();
};