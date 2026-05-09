#include "config.h"
#include "peripherals/register_handling.h"
#include "algebra.h"
#include "debug/debugging.h"
#include <SPI.h>

#define ACC_REFRESH_RATE 1000.0f  // Hz
#define ACC_REFRESH_PERIOD (1.0f/ACC_REFRESH_RATE)  // in seconds
#define GYRO_REFRESH_RATE 1000.0f  // Hz
#define GYRO_REFRESH_PERIOD (1.0f/GYRO_REFRESH_RATE)  // in seconds
#define ACC_FS 2.0f  // g
#define GYRO_FS 1000.0f  // dps
#define ACC_LSB (ACC_FS / 32768.0f)  // g/LSB
#define GYRO_LSB (GYRO_FS / 32768.0f)  // dps/LSB

#define ACC_CALIB_SAMPLES 500
#define GYRO_CALIB_SAMPLES 500

#define GYRO_CONFIG0 0x4F  // gyro FS ODR
#define ACCEL_CONFIG0 0x50  // acc FS, ODR
#define GYRO_CONFIG1 0x51  // TEMP BW, gyro filter order
#define GYRO_ACCEL_CONFIG0 0x52  // acc, gyro LPF BW
#define ACCEL_CONFIG1 0x53  // acc filter order

#define ACCEL_CONFIG_STATIC2 0x03
#define ACCEL_CONFIG_STATIC3 0x04
#define ACCEL_CONFIG_STATIC4 0x05

#define PWR_MGMT0 0x4E  // on-off temp, acc, gyro
#define WHO_AM_I_REG 0x75

class Imu{
    private:
        SPIClass &_spi;
        uint8_t _cs_pin;

        Vector3 latest_acc_vec;
        Vector3 filtered_acc_vec;
        Vector3 latest_gyro_vec;
        Vector3 filtered_gyro_vec;
        float latest_temp;

        Vector3 acc_bias;
        Vector3 gyro_bias;

        unsigned long latest_acc_timestamp;
        unsigned long latest_gyro_timestamp;

    public:
        Imu(SPIClass &spi, uint8_t cs_pin);

        void reset();
        void setup_imu();
        void set_bank(uint8_t bank);
        uint8_t who_am_i();

        void calibrate();
        void calibrate_acc();
        void calibrate_gyro();

        void update_imu();
        void update_gyro();
        void update_acc();

        Vector3 get_acc();
        Vector3 get_gyro();
        float get_temp();
        Vector3 get_filtered_acc();
        Vector3 get_filtered_gyro();
};