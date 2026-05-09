#ifndef CONFIG_H
#define CONFIG_H

//// Settings
#define SETTLE_DELAY 0
#define ENFORCE_GPS 0
#define SAFETY 1  // set safe mode
#define SERIAL_TELEMETRY 0
#define RADIO_TELEMETRY 1
#define DEBUG 0 // enable debugging messages
#define DEBUG_COMM 0  // enable debugging messages

// Timeout settings
#define COMMAND_TIMEOUT_MS 3000   // 60000 ms
#define SENSOR_TIMEOUT_US 2000  // 2 ms

// filter constants
#define IMU_GYROLPF_RATIO 1.0f  // used to weight the new measurement 
#define IMU_ACCLPF_RATIO 1.0f  // used to weight the new measurement
#define IMU_TEMPLPF_RATIO 0.1f  // used to weight the new measurement

#define ACCMAG_ACCLPF_RATIO 1.0f  // used to weight the new measurement 
#define ACCMAG_MAGLPF_RATIO 1.0f  // used to weight the new measurement

#define MAG_MAGLPF_RATIO 1.0f  // used to weight the new measurement

#define GYROLPF_PID_RATIO 1.0f  // used to weight the new measurement 
#define CF_RATIO 0.95f  // gyro weight in complementary filter
#define ALTLPF_RATIO 0.3f  // used to weight the new measurement
#define BATLPF_RATIO 0.1f  // used to weight the new measurement

// Pin definitions
// Motor ESCs
#define MOTOR1_PIN 2
#define MOTOR2_PIN 8
#define MOTOR3_PIN 17
#define MOTOR4_PIN 28

// I2C pins
#define I2C1_SDA_PIN 6
#define I2C1_SCL_PIN 7

// SPI pins
#define SPI0_MOSI_PIN 19
#define SPI0_MISO_PIN 20
#define SPI0_CLK_PIN 22

#define SPI1_MOSI_PIN 15
#define SPI1_MISO_PIN 12
#define SPI1_CLK_PIN 14

#define SPI0_CS_NRF_PIN 21
#define SPI0_CS_SD_PIN 18

#define SPI1_CS_BMP_PIN 13
#define SPI1_CS_ICM_PIN 9

// UART1 (Serial2) pins
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

// Other peripheral pins
#define ICM_INT_PIN 10
#define ICM_FSYNC_PIN 11
#define NRF_INT_PIN 16
#define NRF_CE_PIN 23
#define USB_DETECT_PIN 24
#define SD_DETECT_PIN 29

// output pins
#define EXT_LOAD0_PIN 26
#define EXT_LOAD1_PIN 0
#define IMU_HEAT_PIN 3

#define REDLED_PIN 25
#define GREENLED_PIN 1

// ADC pins
#define ADC_PIN 27

// Some approximate measurements
#define MOTOR_FORCE 680.0f  // youtube approx value
//#define MOTOR_FORCE 825  // second youtube approx value
#define ROLL_TORQUE_COEFF 0.135f
#define PITCH_TORQUE_COEFF 0.157f
#define IDEAL_VOLTAGE 12.40f
#define DRONE_WEIGHT 950

#define LAT_0 500760090  // Latitude origin [10^-7 deg]
#define LON_0 156797142  // Longitude origin [10^-7 deg]
#define COS_LAT 0.64173255079  // Cosine of the origin latitude [rad] 

#endif
