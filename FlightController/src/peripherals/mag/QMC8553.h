#include "config.h"
#include "peripherals/register_handling.h"
#include "algebra.h"
#include "debug/debugging.h"

class Mag{
    private:
        Vector3 last_mag_vec;
        Vector3 filtered_mag_vec;

        Vector3 mag_bias;
        Matrix3 mag_scale;

        int32_t last_mag_timestamp;

        TwoWire &_wire;

    public:
        bool mag_timeout;

        Mag(TwoWire &wire);

        void setup_mag();
        byte ping_mag();
        void print_id();
        Vector3 read_mag();
        Vector3 get_filtered_mag();
};