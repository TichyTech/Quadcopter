#include "peripherlas.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"


void initialize_peripherals(){
    // Start serial
    Serial.begin(500000);
    if (DEBUG || SERIAL_TELEMETRY){  // wait until serial is open
        while(!Serial){}
        delay(500);
    }
    Serial.println("Starting up");

    pinMode(REDLED_PIN, OUTPUT);
    pinMode(GREENLED_PIN, OUTPUT);
    pinMode(EXT_LOAD0_PIN, OUTPUT);
    pinMode(EXT_LOAD1_PIN, OUTPUT);
    pinMode(IMU_HEAT_PIN, OUTPUT);
  
    analogWrite(GREENLED_PIN, 20);
  
    Serial.println("Setting up I2C1");
    // Start I2C for GPS magnetometer
    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.begin();
    Wire1.setClock(400000);
  
    Serial.println("Setting up Serial2");
    // Start Serial2 for GPS
    Serial2.setTX(UART1_TX_PIN);
    Serial2.setRX(UART1_RX_PIN);
    Serial2.begin(115200);
  
    Serial.println("Setting up SPI1");
    // Start SPI for sensors
    SPI1.setRX(SPI1_MISO_PIN);
    SPI1.setSCK(SPI1_CLK_PIN);
    SPI1.setTX(SPI1_MOSI_PIN);
    SPI1.begin();
  
    // Start SPI for NRF24
    Serial.println("Setting up SPI0");
    SPI.setRX(SPI0_MISO_PIN);
    SPI.setSCK(SPI0_CLK_PIN);
    SPI.setTX(SPI0_MOSI_PIN);
    SPI.begin();
}