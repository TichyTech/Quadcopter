#include "sensors.h"

#define ADC_TO_VOLTS 0.0053173828125f // 6.6 * 3.3V / 4096 scaling factor for BATT ADC
#define ADC_OFFSET 0

Battery::Battery(){
  filtered_val = 0;
}

float Battery::get_voltage(){
  float voltage = ADC_TO_VOLTS * (analogRead(ADC_PIN) - ADC_OFFSET);
  return voltage;
}

float Battery::get_filtered_voltage(){
  float current_reading = get_voltage();
  filtered_val = current_reading * BATLPF_RATIO + filtered_val*(1 - BATLPF_RATIO);
  return filtered_val;
}

Sensors::Sensors() : 
    imu(SPI1, SPI1_CS_ICM_PIN),
    magnetometer(Wire1),
    accmag(Wire1),
    altimeter(SPI1, ALT_CS_PIN)
{
  battery = Battery();
  timed_out = false;
  accmag_temp_timed_out = false;
}

void Sensors::setup(){
  Serial.println("Setting up peripherals");

  imu.setup_imu(); // blocking in failure
  accmag.setup_acc();  // blocking in failure
  accmag.setup_mag();  // blocking in failure
  magnetometer.setup_mag();  // blocking in failure if ENFORCE_GPS
  altimeter.setup_alt();  // blocking in failure

  Serial.println("Mounting ADC");
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);
  
  Serial.println("All sensors ready!");

  if(SETTLE_DELAY) delay(5000); //delay the calibration process a bit to allow drone to settle on the ground
  imu.calibrate(); // 3 sec bias calibration
}

Measurements Sensors::get_measurements(){  // init all readings to bypass low pass filters in update_measurements()
  Measurements new_m;
  // new_m.mag_vec = accmag.read_mag();
  new_m.mag_vec = magnetometer.read_mag();
  imu.update_imu();
  new_m.gyro_vec = imu.get_gyro();
  new_m.acc_vec = imu.get_acc(); 
  new_m.altimeter = altimeter.read_alt();
  new_m.battery = battery.get_voltage();
  new_m.integration_period = update_integration_period();  // first integration period is invalid!!

  #if ENFORCE_GPS == 1
  timed_out = magnetometer.mag_timeout;
  #endif

  return new_m;
}

Measurements Sensors::get_measurements_filtered(){
  Measurements new_m;
  imu.update_imu();
  new_m.acc_vec = imu.get_filtered_acc(); 
  // new_m.acc_vec = accmag.get_filtered_acc(); 
  new_m.gyro_vec = imu.get_filtered_gyro();
  new_m.mag_vec = magnetometer.get_filtered_mag();
  // new_m.mag_vec = accmag.get_filtered_mag();
  new_m.altimeter = altimeter.get_filtered_alt();  // low pass filter for altimeter
  new_m.battery = battery.get_filtered_voltage();
  new_m.integration_period = update_integration_period();  // record microseconds since last measurement

  #if ENFORCE_GPS == 1
  timed_out = magnetometer.mag_timeout;
  #endif

  return new_m;
}

Temperatures Sensors::get_temperatures(){
  Temperatures temps;
  temps.imu_temp = imu.get_temp();
  temps.rp_temp = analogReadTemp();
  temps.alt_temp = float(altimeter.read_temp())*0.01f;
  temps.mag_temp = accmag.read_temp();
  accmag_temp_timed_out = accmag.temp_timeout;
  return temps;
}

float Sensors::update_integration_period(){
  // first integration period is invalid!!
  static uint32_t last_timestamp = 0;  
  uint32_t current_timestamp = micros();
  float integration_period = float(current_timestamp - last_timestamp)*(1/1000000.0);
  last_timestamp = current_timestamp;
  return integration_period;
}
