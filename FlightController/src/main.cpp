#include <Arduino.h>
#include "config.h"
#include "logging/ulog.h"
#include <memory>

#include "peripherals/drivers/ICM42605.h"

std::unique_ptr<Logger> logger;
std::unique_ptr<Imu> imu;

uint64_t last_log_time = 0;
uint64_t num_msgs = 0;

constexpr const char accel_name[] = "accel_msg";
constexpr const char gyro_name[]  = "gyro_msg";

using ulog_accel_msg_t = ulog_vector3f_msg_t<accel_name, 0>;
using ulog_gyro_msg_t  = ulog_vector3f_msg_t<gyro_name, 1>;


void setup() 
{
  Serial.begin(115200);
  // while (!Serial) {} 
  Serial.println("Serial initialized");

  Serial.print("Initializing SPI...");

  Serial.println("Setting up SPI");
  SPI.setRX(SPI0_MISO_PIN);
  SPI.setSCK(SPI0_CLK_PIN);
  SPI.setTX(SPI0_MOSI_PIN);
  SPI.begin();

  Serial.println("Setting up SPI1");
  // Start SPI for sensors
  SPI1.setRX(SPI1_MISO_PIN);
  SPI1.setSCK(SPI1_CLK_PIN);
  SPI1.setTX(SPI1_MOSI_PIN);
  SPI1.begin();

  Serial.print("Initializing Logger...");
  logger = std::make_unique<Logger>(SPI0_CS_SD_PIN, SPI);
  logger->write_format_message(ulog_accel_msg_t{}.view());
  logger->write_format_message(ulog_gyro_msg_t{}.view());
  logger->write_subscription(ulog_accel_msg_t{}.view());
  logger->write_subscription(ulog_gyro_msg_t{}.view());

  Serial.print("Initializing Imu...");
  imu = std::make_unique<Imu>(SPI1, SPI1_CS_ICM_PIN);
  imu->setup_imu();

  Serial.print("Logging sensors!");
  last_log_time = millis();
}


void loop() 
{
  if (millis() - last_log_time > 500) {
    logger->flush();
    last_log_time = millis();
    Serial.println(num_msgs / 0.5);
    num_msgs = 0;
  }

  imu->update_imu();
  {
    Vector3 acc_reading = imu->get_acc();
    uint64_t timestamp = micros();
    ulog_accel_msg_t msg = {
      .payload = {
        .data = {
          .timestamp = timestamp,
          .x = acc_reading(0),
          .y = acc_reading(1),
          .z = acc_reading(2)
        }
      }
    };

    logger->write_data_message(msg.view());
  }

  {
    Vector3 gyro_reading = imu->get_gyro();
    uint64_t timestamp = micros();
    ulog_gyro_msg_t msg = {
      .payload = {
        .data = {
          .timestamp = timestamp,
          .x = gyro_reading(0),
          .y = gyro_reading(1),
          .z = gyro_reading(2)
        }
      }
    };

    logger->write_data_message(msg.view());
  }

  num_msgs++;

}