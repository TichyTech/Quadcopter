#include "definitions.h"
#include "config.h"

#include "peripherals/drivers/LSM303DLHC.h"
#include "peripherals/drivers/BMP280.h"
#include "peripherals/drivers/QMC8553.h"
#include "peripherals/drivers/ICM42605.h"

/**
 * Class for managing battery readout and filtering
 */
class Battery{
  private:
    float m_filtered_val;  // filtered voltage value
  public:
    Battery();
    float get_voltage();  // read current battery voltage from ADC
    float get_filtered_voltage();  // read and low-pass filter battery voltage from ADC
};

class Sensors{
	private:
		Imu imu;
		AccMag accmag;
		Mag magnetometer;
		Altimeter altimeter;
    Battery battery;
	public: 
		bool timed_out;
    bool accmag_temp_timed_out;

		Sensors();
		void setup();
		Measurements get_measurements();
		Measurements get_measurements_filtered();
    Temperatures get_temperatures();
    float update_integration_period();
};
