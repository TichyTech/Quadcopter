#include <Arduino.h>
#include "SPI.h"

#define ALT_CS_PIN 13
#define ALT_SPI_CLOCK 1000000

class Altimeter
{

  private:
    SPIClass &_spi;
    uint8_t _cs_pin;

    int32_t t_fine;
    float last_alt_val;
    float filtered_alt_val;
    unsigned long last_alt_timestamp;

    int write_alt8(uint8_t reg, uint8_t val);
    uint8_t read_alt8(uint8_t reg);
    int16_t read_alt16(uint8_t reg);
    int32_t read_alt24(uint8_t reg);

  public:
    Altimeter(SPIClass &spi, uint8_t cs_pin);
    
    void setup_alt();
    float read_alt();
    float get_filtered_alt();
    int32_t read_temp();
    int32_t get_filtered_temp();
    float read_pressure();
    void get_alt_calibration();

};