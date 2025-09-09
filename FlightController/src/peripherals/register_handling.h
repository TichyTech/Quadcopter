#include <Wire.h>
#include <SPI.h>

int write_reg(TwoWire& wire, uint8_t address, uint8_t reg, uint8_t value);
int write_reg(SPIClass &spi, uint8_t cs_pin, uint8_t reg, uint8_t value, uint32_t speed);
int write_reg_verify(SPIClass &spi, uint8_t cs_pin, uint8_t reg, uint8_t value, uint32_t speed);

int read_reg(SPIClass &spi, uint8_t cs_pin, uint8_t reg, uint32_t speed);
