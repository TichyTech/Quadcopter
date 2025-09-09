#include "register_handling.h"

int write_reg(TwoWire& wire, uint8_t address, uint8_t reg, uint8_t value){
  wire.beginTransmission(address);
  wire.write(reg);
  wire.write(value);
  wire.endTransmission();
  return 0;
}

int write_reg(SPIClass& spi, uint8_t cs_pin, uint8_t reg, uint8_t value, uint32_t speed){
  spi.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_pin, LOW);
  spi.transfer16((int16_t)((reg << 8) | value));
  digitalWrite(cs_pin, HIGH); 
  spi.endTransaction(); 
  return 0;
}

int write_reg_verify(SPIClass& spi, uint8_t cs_pin, uint8_t reg, uint8_t value, uint32_t speed){
  spi.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_pin, LOW);
  spi.transfer16((int16_t)((reg << 8) | value));
  digitalWrite(cs_pin, HIGH); 
  spi.endTransaction();

  if (read_reg(spi, cs_pin, reg, speed) != value) {
    Serial.print("Failed to write register 0x");
    Serial.print(reg, HEX);
    Serial.print(" with value 0x");
    Serial.println(value, HEX);
    return 1;
  }
  return 0;
}

int read_reg(SPIClass& spi, uint8_t cs_pin, uint8_t reg, uint32_t speed){
  spi.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_pin, LOW);
  spi.transfer(reg | 0x80); 
  uint8_t ret = spi.transfer(0x00);
  digitalWrite(cs_pin, HIGH); 
  spi.endTransaction(); 
  return ret;
}
