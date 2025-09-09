#include "config.h"
#include "definitions.h"
#include "debug/debugging.h"
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include <hardware/uart.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>

#define GPS_REFRESH_PERIOD 100 // [ms]

class GPS{

  private:
    Vector3 last_gps_pos;
    SFE_UBLOX_GNSS_SERIAL myGNSS;

  public:
    unsigned long last_fix_timestamp; 

    uint32_t tow;  // [ms]
    int32_t latitude;  // [10^-7 deg]
    int32_t longitude;  // [10^-7 deg]
    int32_t altitude;  // [mm]

    int32_t n_vel; // [mm/s]
    int32_t e_vel; // [mm/s]
    int32_t d_vel; // [mm/s]

    uint16_t pos_DOP;  // scaled by 0.01
    int32_t v_acc; // [mm]
    int32_t h_acc; // [mm]

    uint8_t fix_type;
    uint8_t num_sv;

    bool gps_timeout;

    GPS();
    void setup_gps();
    bool my_update_gps();
    bool update_gps();
    void parse_buffer();
    Vector3 get_NWU_pos();
    Vector3 get_NWU_speed();
};

void setup_DMA();
byte read();
uint16_t available();
