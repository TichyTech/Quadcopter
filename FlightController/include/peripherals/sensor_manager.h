#include "definitions.h"
#include "config.h"

#include "peripherals/drivers/LSM303DLHC.h"
#include "peripherals/drivers/BMP280.h"
#include "peripherals/drivers/QMC8553.h"
#include "peripherals/drivers/ICM42605.h"

class Battery{
  private:
    float filtered_val;
  public:
    Battery();
    float get_voltage();
    float get_filtered_voltage();
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
