# About: 

This repositary contains my personal Quadcopter Flight Controller implementation from scratch. Despite the numerous open-source flight controllers out there, I wanted to understand and create my own, just for the fun of it. 
Also, I enjoy 3D design and 3D printing and so the whole drone frame is 3D printed :).

<div align="center">
<img src="https://github.com/user-attachments/assets/2a7008c6-4aea-4ef0-93c5-a2a06ec92134" alt="Drone" width="640"/>
</div>

# System Architecture

The whole project relies on platformio for dependencies and compilation for ease of use.

The [flight controller](FlightController) runs on the two cores of the Pi Pico. 
The first core is used to poll the sensors for newest readings, estimate the attitude of the drone using quaternion EKF and based on the commands and PID controllers, 
stabilize the drone by driving the four motor ESCs.
The second core is used strictly for communication - receiving commands and sending back telemetry via the NRF24L01 to the [remote controller](RemoteController).

The [drone plotter](DronePlotter) is then a simple visualization application written using [Processing](https://processing.org/), which pulls data from the Remote Controller over serial interface.

## Main Components

### Flight Controller

- Raspberry Pi Pico - main MCU
- NRF24L01 (SPI) - communication
- L3G4200D (I2C) - 3 axis gyroscope
- LSM303DLHC (I2C) - 3 axis accelerometer + 3 axis magnetometer
- BMP280 (I2C) - barometer

These components are all soldered onto a single printed circuit board together with some resistors, leds, buttons and headers.
<div align="center">
<img src="https://github.com/user-attachments/assets/e0b6734f-1725-4d4c-80f0-add37cce425e" alt="Schematic" height="200"/>
<img src="https://github.com/user-attachments/assets/b0ca1b17-aefe-4f4c-81d1-9dcd8bb294eb" alt="BoardFront" height="200"/>
<img src="https://github.com/user-attachments/assets/ec872b25-bdf1-488c-b9b2-42578bd2057d" alt="BoardBack" height="200"/>
</div>

### Onboard electronics

- 3S 1200mAh LiPo battery
- 4x Little Bee BLHeli_S ESC 30A
- 4x 2212 1000KV BLDC motors
- 4x 1045 propellers
- PDB with BEC 5V

Together with the Flight controller board, these are placed in a 3D printed frame.
<div align="center">
<img src="https://github.com/user-attachments/assets/3bdda922-ef74-47cc-b8ec-75419c08cc3b" alt="Electronics" width="300"/>
</div>

### Remote Controller

- Arduino Nano - main MCU
- NRF24L01 (SPI) - communication
- potentiometer - throttle
- 2x XY joysticks - Roll, Pitch, Yaw control
- 2x buttons - Motors On/Off, etc..

Again, these are all soldered onto a single PCB and encased in a 3D printed case.

<div align="center">
<img src="https://github.com/user-attachments/assets/248d8148-2569-4644-9dd4-d5fad3795bd8" alt="Remote" height="200"/>
<img src="https://github.com/user-attachments/assets/9f530088-0f82-439b-b7c9-2765e080a28f" alt="Remote" height="200"/>
</div>

And finally, when all put together, it flies!!! 

<div align="center">
<img src="SucessfulFlight.gif" alt="BoardBack" height="300"/>
</div>
