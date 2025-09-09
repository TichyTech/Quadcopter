// https://www.st.com/resource/en/datasheet/DM00027543.pdf
// LSM303DLHC accelerometer and magnetometer
#include "LSM303DLHC.h"

#define ACC_ADR 0x19
#define MAG_ADR 0x1E
#define ACC_REFRESH_RATE 400.0f  // Hz
#define ACC_REFRESH_PERIOD (1.0f/ACC_REFRESH_RATE)  // in seconds
#define MAG_REFRESH_RATE 220.0f  // Hz
#define MAG_REFRESH_PERIOD (1.0f/MAG_REFRESH_RATE)  // in seconds
#define ACC_FS 2.0f  // g
#define MAG_FS 1.3f  // gauss

AccMag::AccMag(TwoWire &wire) : _wire(wire) {
  acc_bias = {0,0,0};
  acc_timeout = 0;
  mag_timeout = 0;
  temp_timeout = 0;
  mag_scale = {1,0,0,0,1,0,0,0,1};
  mag_bias = {0,0,0};  // in microtesla = 0.01 gauss

  last_mag_timestamp = 0;
  last_acc_timestamp = 0;
  last_temp_timestamp = 0;
}

void AccMag::setup_acc(){
  Serial.println("Setting up LSM303DLHC accelerometer");
  _wire.beginTransmission(ACC_ADR);
  byte error = _wire.endTransmission();
  
  if(error){
    Serial.println("LSM303DLHC Accelerometer not responding");
    while(1){}
  }
      
 write_reg(_wire, ACC_ADR, 0x20, 0x77); // 400Hz,  enable XYZ
  // write_reg(_wire, ACC_ADR, 0x20, 0x97); // 1344Hz,  enable XYZ
  write_reg(_wire, ACC_ADR, 0x23, 0x88); // block update until read, LSB first, FS +-2g, enable High resolution
  filtered_acc_vec = read_acc();
}


void AccMag::setup_mag(){
  Serial.println("Setting up LSM303DLHC magnetometer");
  _wire.beginTransmission(MAG_ADR);
  byte error = _wire.endTransmission();
  
  if(error){
    Serial.println("LSM303DLHC Magnetometer not responding");
    while(1){}
  }
  
  write_reg(_wire, MAG_ADR, 0x00, 0x9C); // 220 Hz (max)
  write_reg(_wire, MAG_ADR, 0x01, 0x20); // gain 1100 (max), +- 1.3 gauss
  write_reg(_wire, MAG_ADR, 0x02, 0x00); // continuous conversion mode

  filtered_mag_vec = read_mag();
  };


Vector3 AccMag::read_acc(){
  // fetches latest possible measurement from accelerometer unit
  if ((micros() - last_acc_timestamp) < (ACC_REFRESH_PERIOD*1000000)) return last_acc_vec; 
  else last_acc_timestamp = micros();
  
  _wire.beginTransmission(ACC_ADR);
  _wire.write(0x28 | (1 << 7));
  _wire.endTransmission();
  _wire.requestFrom(ACC_ADR, (byte)6);
  while (_wire.available()<6) {
    if ((micros() - last_acc_timestamp) > SENSOR_TIMEOUT_US){
      acc_timeout = 1;
      return zero_3vector;
    }
  }
  
  uint8_t xla = _wire.read();
  uint8_t xha = _wire.read();
  uint8_t yla = _wire.read();
  uint8_t yha = _wire.read();
  uint8_t zla = _wire.read();
  uint8_t zha = _wire.read();

  // l,h x,y,z upper 12 bits
  Vector3 acc_vals;
  acc_vals(0) = float((int16_t)(xha << 8 | xla)>>4);
  acc_vals(1) = -float((int16_t)(yha << 8 | yla)>>4);
  acc_vals(2) = -float((int16_t)(zha << 8 | zla)>>4);
  acc_vals = acc_vals*(ACC_FS/2048) - acc_bias;

  last_acc_vec = acc_vals; 
  return acc_vals;
}

Vector3 AccMag::read_mag(){
  // fetches latest possible measurement from magnetometer unit
  if ((micros() - last_mag_timestamp) < (MAG_REFRESH_PERIOD * 1000000)) return last_mag_vec; 
  else last_mag_timestamp = micros();
  
  _wire.beginTransmission(MAG_ADR);
  _wire.write(0x03 | (1 << 7));
  _wire.endTransmission();
  _wire.requestFrom(MAG_ADR, (byte)6);
  while (_wire.available()<6) {
    if ((micros() - last_mag_timestamp) > SENSOR_TIMEOUT_US){
      mag_timeout = 1;
      return zero_3vector;
    }  
  }
  
  uint8_t xhm = _wire.read();
  uint8_t xlm = _wire.read();
  uint8_t zhm = _wire.read();
  uint8_t zlm = _wire.read();
  uint8_t yhm = _wire.read();
  uint8_t ylm = _wire.read();

  // h,l x,z,y lower 16 bits
  Vector3 mag_vals;
  mag_vals(0) = float((int16_t)(xhm << 8 | xlm));  
  mag_vals(1) = -float((int16_t)(yhm << 8 | ylm));
  mag_vals(2) = -float((int16_t)(zhm << 8 | zlm));
  mag_vals = mag_scale*(mag_vals*(MAG_FS/(2048)) - mag_bias*0.01f);  // corrected readings

  last_mag_vec = mag_vals;
  return mag_vals;
}

Vector3 AccMag::get_filtered_acc(){
  if ((micros() - last_acc_timestamp) < (ACC_REFRESH_PERIOD*1000000)) return filtered_acc_vec; 
  Vector3 acc_reading = read_acc();
  filtered_acc_vec = acc_reading * ACCMAG_ACCLPF_RATIO + filtered_acc_vec * (1 - ACCMAG_ACCLPF_RATIO);
  return filtered_acc_vec;
};

Vector3 AccMag::get_filtered_mag(){
  if ((micros() - last_mag_timestamp) < (MAG_REFRESH_PERIOD * 1000000)) return filtered_mag_vec; 
  Vector3 mag_reading = read_mag();
  filtered_mag_vec = mag_reading * ACCMAG_MAGLPF_RATIO + filtered_mag_vec * (1 - ACCMAG_MAGLPF_RATIO);
  return filtered_mag_vec;
};

float AccMag::read_temp(){
  // fetches latest possible measurement from temperature sensor
  _wire.beginTransmission(MAG_ADR);
  _wire.write(0x31 | (1 << 7));
  _wire.endTransmission();
  _wire.requestFrom(MAG_ADR, (byte)2);
  last_temp_timestamp = micros();
  while (_wire.available() < 2) {
    if ((micros() - last_temp_timestamp) > SENSOR_TIMEOUT_US){
      temp_timeout = 1;
      Serial.println("Temperature sensor timed out");
      return 0.0f;
    }
  }
  
  uint8_t tha = _wire.read();
  uint8_t tla = _wire.read();

  float temp = float(160 + ((int16_t)(tha << 8 | tla) >> 4)) * 0.125f;
  return temp;
}

void AccMag::calibrate_acc(){  // sensor needs to be perpendicular to gravity vector
  // assuming stationary IMU, we first obtain the direction of gravity
  Vector3 grav_dir = {0,0,0};
  delay(500);
  for (int i = 0; i < 50; i ++){
    grav_dir += read_acc()/50.0f;
    delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 100);
  }
  grav_dir = normalize(grav_dir);  // get gravity vector of magnitude 1

  // zero out bias and add average of the samples to it
  acc_bias = {0,0,0};
  for (int i = 0; i < 100; i ++){
    acc_bias += (read_acc() - grav_dir)/100.0f;
    delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 100);
  }
  Vector3 acc_sigma = {0,0,0}; 
  for (int i = 0; i < 100; i ++){
    Vector3 meas = read_acc() - grav_dir - acc_bias;
    Vector3 squares = {meas(0)*meas(0), meas(1)*meas(1), meas(2)*meas(2)};
    acc_sigma += squares*(1.0f/99.0f);
    delayMicroseconds((ACC_REFRESH_PERIOD*1000000) + 100);
  }
  Serial.print("Acc sigma: ");
  printVec3(acc_sigma, 5);
  Serial.println("Accelerometer calibrated");
}
