#include "ICM42605.h"

#define ICM_SPI_CS 9

Imu::Imu(SPIClass &spi, uint8_t cs_pin) : 
    _spi(spi), _cs_pin(cs_pin) 
{
    acc_bias = {0, 0, 0};
    gyro_bias = {0, 0, 0};
    latest_acc_vec = {0, 0, 0};
    filtered_acc_vec = {0, 0, 0};
    latest_gyro_vec = {0, 0, 0};
    filtered_gyro_vec = {0, 0, 0};
    latest_temp = 0;

    latest_acc_timestamp = 0;
    latest_gyro_timestamp = 0;
}

void Imu::setup_imu(){
  Serial.println("Setting up ICM42605 IMU");
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);

  reset();
  set_bank(0);

  uint8_t whoami = who_am_i();
  if (whoami != 0x42){
    Serial.print("ICM42605 has wrong ID: 0x");
    Serial.println(whoami, HEX);
    while(1) {}
  }

  set_bank(2);
  // set ACC AAF to 364 Hz
  write_reg_verify(_spi, _cs_pin, ACCEL_CONFIG_STATIC2, (29U << 1) | (0x01), 1000000);  // acc AAF DELT 29, on
  write_reg_verify(_spi, _cs_pin, ACCEL_CONFIG_STATIC3, (848U & 0xFF), 1000000);  // acc AAF DELT 29, on
  write_reg_verify(_spi, _cs_pin, ACCEL_CONFIG_STATIC4, (0x05 << 4) | (848U >> 8), 1000000);  // acc AAF DELT 29, on
  set_bank(0);

  write_reg_verify(_spi, _cs_pin, GYRO_ACCEL_CONFIG0, (0x06 << 4) | (0x06), 1000000);  // ACC LPF BW 50Hz, GYRO LPF BW 50Hz
  write_reg_verify(_spi, _cs_pin, ACCEL_CONFIG0, (0x03 << 5) | (0x06), 1000000);  // acc fs +-2g, 1kHz
  write_reg_verify(_spi, _cs_pin, GYRO_CONFIG0, (0x01 << 5) | (0x06), 1000000);  // gyro fs +-1000dps, 1kHz
  write_reg_verify(_spi, _cs_pin, PWR_MGMT0, 0x1F, 1000000);  // Turn on temp, gyro and acc
  delayMicroseconds(200);  // per the datasheet we need to wait until further register writes
}

void Imu::reset(){
  set_bank(0);
  write_reg(_spi, _cs_pin, 0x11, 0x01, 1000000);
  delay(1);
}

uint8_t Imu::who_am_i(){
  return read_reg(_spi, _cs_pin, WHO_AM_I_REG, 1000000);
}

void Imu::set_bank(uint8_t bank){
  write_reg(_spi, _cs_pin, 0x76, bank, 1000000);
}

void Imu::calibrate_acc(){
    // assuming stationary IMU, we first obtain the direction of gravity
    Vector3 grav_dir = {0,0,0};
    for (int i = 0; i < ACC_CALIB_SAMPLES; i ++){
        grav_dir += get_acc()*(1.0f/ACC_CALIB_SAMPLES);
        delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 1000);
    }
    Serial.print("Gravity: ");
    printVec3(grav_dir, 2);
    Serial.println();
    grav_dir = normalize(grav_dir);  // get gravity vector of magnitude 1

    // zero out bias and add average of the samples to it
    acc_bias = {0,0,0};
    Vector3 acc_bias_est = {0,0,0};
    for (int i = 0; i < ACC_CALIB_SAMPLES; i ++){
        acc_bias_est += (get_acc() - grav_dir)*(1.0f/ACC_CALIB_SAMPLES);
        delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 1000);
    }
    Vector3 acc_sigma = {0,0,0}; 
    for (int i = 0; i < ACC_CALIB_SAMPLES; i ++){
        Vector3 meas = get_acc() - grav_dir - acc_bias_est;
        Vector3 squares = {meas(0)*meas(0), meas(1)*meas(1), meas(2)*meas(2)};
        acc_sigma += squares*(1.0f/(ACC_REFRESH_PERIOD - 1));
        delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 1000);
    }
    Serial.print("Acc bias: ");
    printVec3(acc_bias_est, 5);
    Serial.print("Acc sigma: ");
    printVec3(acc_sigma, 5);
    Serial.println("Accelerometer calibrated");
    acc_bias = acc_bias_est;
}

void Imu::calibrate_gyro(){
    // zero out bias and add average of the samples to it
    gyro_bias = {0,0,0};
    Vector3 gyro_bias_est = {0,0,0};
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i ++){
        gyro_bias_est += get_gyro()*(1.0f/GYRO_CALIB_SAMPLES);
        delayMicroseconds((GYRO_REFRESH_PERIOD*1000000) + 1000);
    }
    Vector3 gyro_sigma = {0,0,0}; 
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i ++){
        Vector3 meas = get_gyro() - gyro_bias_est;
        Vector3 squares = {meas(0)*meas(0), meas(1)*meas(1), meas(2)*meas(2)};
        gyro_sigma += squares*(1.0f/(GYRO_CALIB_SAMPLES - 1));
        delayMicroseconds((GYRO_REFRESH_PERIOD*1000000) + 1000);
    }
    Serial.print("Gyro bias: ");
    printVec3(gyro_bias_est, 5);
    Serial.print("Gyro sigma: ");
    printVec3(gyro_sigma, 5);
    Serial.println("Gyroscope calibrated");
    gyro_bias = gyro_bias_est;
}

void Imu::calibrate(){
  calibrate_acc();
  calibrate_gyro();
}

void Imu::update_imu(){
  uint32_t micros_now = micros();
  if ((micros_now - latest_acc_timestamp < ACC_REFRESH_PERIOD) && (micros_now - latest_gyro_timestamp < GYRO_REFRESH_PERIOD)) 
    return;

  latest_acc_timestamp = micros_now;
  latest_gyro_timestamp = micros_now;

  // fetch data from the sensor
  uint8_t buf[14];
  _spi.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI1_CS_ICM_PIN, LOW);
  _spi.transfer(0x1D | 0x80); 
  _spi.transfer(nullptr, buf, 14);
  digitalWrite(SPI1_CS_ICM_PIN, HIGH); 
  _spi.endTransaction(); 

  // reinterpret the data
  int16_t data[7];
  for (int i = 0; i < 7; i++) data[i] = (((int16_t)buf[2*i]) << 8) | buf[2*i+1];

  latest_temp = data[0] * 0.00754830917874f + 25;
  latest_acc_vec = {
    -ACC_LSB*data[1], 
    ACC_LSB*data[2], 
    -ACC_LSB*data[3]
  };
  latest_gyro_vec = {
    -GYRO_LSB*data[4], 
    GYRO_LSB*data[5], 
    -GYRO_LSB*data[6]
  };

  latest_acc_vec = latest_acc_vec - acc_bias;
  latest_gyro_vec = latest_gyro_vec - gyro_bias;
}

void Imu::update_acc(){
  uint32_t micros_now = micros();
  if (micros_now - latest_acc_timestamp < ACC_REFRESH_PERIOD) return;
  latest_acc_timestamp = micros_now;

  // fetch data from the sensor
  uint8_t buf[6];
  _spi.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI1_CS_ICM_PIN, LOW);
  _spi.transfer(0x1F | 0x80); 
  _spi.transfer(nullptr, buf, 6);
  digitalWrite(SPI1_CS_ICM_PIN,HIGH); 
  _spi.endTransaction(); 

  // reinterpret the data
  int16_t data[3];
  for (int i = 0; i < 3; i++) data[i] = (((int16_t)buf[2*i]) << 8) | buf[2*i+1];

  latest_acc_vec = {
    -ACC_LSB*data[0], 
    ACC_LSB*data[1], 
    -ACC_LSB*data[2]
  };  
  latest_acc_vec = latest_acc_vec - acc_bias;
}

void Imu::update_gyro(){
  uint32_t micros_now = micros();
  if (micros_now - latest_gyro_timestamp < GYRO_REFRESH_PERIOD) return;
  latest_gyro_timestamp = micros_now;

  // fetch data from the sensor
  uint8_t buf[6];
  _spi.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI1_CS_ICM_PIN, LOW);
  _spi.transfer(0x25 | 0x80); 
  _spi.transfer(nullptr, buf, 6);
  digitalWrite(SPI1_CS_ICM_PIN,HIGH); 
  _spi.endTransaction(); 

  // reinterpret the data
  int16_t data[3];
  for (int i = 0; i < 3; i++) data[i] = (((int16_t)buf[2*i]) << 8) | buf[2*i+1];

  latest_gyro_vec = {
    -GYRO_LSB*data[0], 
    GYRO_LSB*data[1], 
    -GYRO_LSB*data[2]
  };  
  latest_gyro_vec = latest_gyro_vec - gyro_bias;
}

Vector3 Imu::get_acc(){
  if ((micros() - latest_acc_timestamp) > (ACC_REFRESH_PERIOD)) update_acc();
  return latest_acc_vec;
}

Vector3 Imu::get_gyro(){
  if ((micros() - latest_gyro_timestamp) > (GYRO_REFRESH_PERIOD)) update_gyro();
  return latest_gyro_vec;
}

float Imu::get_temp(){
    return latest_temp;
}

Vector3 Imu::get_filtered_acc(){
    if ((micros() - latest_acc_timestamp) < (ACC_REFRESH_PERIOD)) return filtered_acc_vec;
    filtered_acc_vec = get_acc() * IMU_ACCLPF_RATIO + filtered_acc_vec * (1 - IMU_ACCLPF_RATIO);
    return filtered_acc_vec;
}

Vector3 Imu::get_filtered_gyro(){
    if ((micros() - latest_gyro_timestamp) < (GYRO_REFRESH_PERIOD)) return filtered_gyro_vec;
    filtered_gyro_vec = get_gyro() * IMU_GYROLPF_RATIO + filtered_gyro_vec * (1 - IMU_GYROLPF_RATIO);
    return filtered_gyro_vec;
}