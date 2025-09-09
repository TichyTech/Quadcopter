// https://www.st.com/resource/en/datasheet/DM00027543.pdf
// LSM303DLHC accelerometer and magnetometer
#include "QMC8553.h"

#define QMC_MAG_ADR 0x0D
#define QMC_MAG_REFRESH_RATE 200.0f  // Hz
#define QMC_MAG_REFRESH_PERIOD (1.0f/QMC_MAG_REFRESH_RATE)  // in seconds
#define QMC_MAG_FS 2.0f  // gauss

Mag::Mag(TwoWire &wire) : _wire(wire) {
  mag_timeout = 0;
  mag_scale = {1.002, 0.003, 0.007, 0.003, 0.996, 0.006, 0.007, 0.006, 1.002};
  mag_bias = {1.34, 21.46, -2.35};  // in microtesla = 0.01 gauss

  last_mag_timestamp = 0;
}

void Mag::setup_mag(){
  Serial.println("Setting up QMC8553 magnetometer");
  while(ping_mag()){
    Serial.println("Magnetometer QMC8553 not responding");
    #if ENFORCE_GPS == 0
    break;  // do not wait for magnetometer if GPS is not enforced
    #endif
    delay(500);
  }
  
  write_reg(_wire, QMC_MAG_ADR, 0x09, 0x0D); // 512 samples, 2G FS, 200 Hz, Continuous mode
  write_reg(_wire, QMC_MAG_ADR, 0x0A, 0x01); // enable address roll-over
  write_reg(_wire, QMC_MAG_ADR, 0x0B, 0x01); // This is due to datasheet and is supposed to reset the magnetic field

  filtered_mag_vec = read_mag();
  };

byte Mag::ping_mag()
{
  _wire.beginTransmission(QMC_MAG_ADR);
  return _wire.endTransmission();
}

void Mag::print_id(){
  _wire.beginTransmission(QMC_MAG_ADR);
  _wire.write(0x0D);
  _wire.endTransmission();
  _wire.requestFrom(QMC_MAG_ADR, (byte)1);
  while (_wire.available()<1) {}
  uint8_t chip_id = _wire.read();
  Serial.print("Chip id: ");
  Serial.println(chip_id);  
}

Vector3 Mag::read_mag(){
  // fetches latest possible measurement from magnetometer unit
  if ((micros() - last_mag_timestamp) < (QMC_MAG_REFRESH_PERIOD * 1000000)) return last_mag_vec; 
  else last_mag_timestamp = micros();
  
  _wire.beginTransmission(QMC_MAG_ADR);
  _wire.write(0x00);
  _wire.endTransmission();
  _wire.requestFrom(QMC_MAG_ADR, (byte)6);
  while (_wire.available()<6) {
    if ((micros() - last_mag_timestamp) > SENSOR_TIMEOUT_US){
      mag_timeout = 1;
      return zero_3vector;
    }  
  }
  uint8_t xlm = _wire.read();
  uint8_t xhm = _wire.read();
  uint8_t ylm = _wire.read();
  uint8_t yhm = _wire.read();
  uint8_t zlm = _wire.read();
  uint8_t zhm = _wire.read();

  // h,l x,z,y lower 16 bits
  Vector3 mag_vals;
  mag_vals(0) = float((int16_t)(xhm << 8 | xlm));
  mag_vals(1) = -float((int16_t)(yhm << 8 | ylm));
  mag_vals(2) = -float((int16_t)(zhm << 8 | zlm));
  mag_vals = mag_scale*(mag_vals*(QMC_MAG_FS/32768) - mag_bias*0.01f);  // corrected readings

  last_mag_vec = mag_vals;
  return mag_vals;
}

Vector3 Mag::get_filtered_mag(){
  if ((micros() - last_mag_timestamp) < (QMC_MAG_REFRESH_PERIOD * 1000000)) return filtered_mag_vec; 
  Vector3 mag_reading = read_mag();
  filtered_mag_vec = mag_reading * MAG_MAGLPF_RATIO + filtered_mag_vec * (1 - MAG_MAGLPF_RATIO);
  return filtered_mag_vec;
};