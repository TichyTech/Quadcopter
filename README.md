# About: 

This is my DIY quadcopter project, that builds on my previous [work](https://github.com/TichyTech/QuadcopterFlightController).

<div align="center">
<img src="docs/QuadRenderProfile.png" alt="Quadcopter" width="640"/>
</div>

Since I wanted to understand the electronics/robotics product design process from idea to manufacturing, I designed my own flight controller board, with my own flight controller software and housed it in a custom 3D printed frame that I designed.

Look, it flies!!

<div align="center">
<img src="docs/FlightDay2.gif" alt="Quadcopter" width="640"/>
</div>

# Project Overview

### Custom 3D printed frame

It took a lot of iterations to get this 3D printed frame right. In the previous design attempts, I struggled with motor caused vibrations creeping their way into the IMU and while the Gyroscope was still usable, the Accelerometer was not able to keep the drone perfectly upright. Therefore I made the drone a lot more sturdy, while also adding vibration dampers under the flight controller to help shield it from noise. The vibration damping system consists of these 4 foam cylinders mounted between a base plate and the flight controller pcb as shown here:

<div align="center">
<img src="docs/VibrationDamperRender.png" alt="FC_PCB" width="640"/>
</div>

This contraption is friction fitted and screwed into the main quadcopter body and also works to conceal and fix the wiring and escs below it.

<div align="center">
<img src="docs/VibrationDamperInBody.png" alt="FC_PCB" width="640"/>
</div>

The rest of the frame was designed so that:
1) It can house all the necessary components
2) Only screws, nuts and friction is necessary for assembly
3) It can be 3D printed well on an FDM printer
4) It can be assembled and dissasembled fairly comfortably
5) Does not look like rubish

And as an extra bonus, I tried to make the wings downward foldable, so I can store it easily. In the end, after many iterations I converged to this:

<div align="center">
<img src="docs/CADscreen.png" alt="CAD" height="260"/>
<img src="docs/CAD_SLice_annotated.png" alt="CAD_Inside" height="260"/>
</div>

And here, enjoy some renders:

<div align="center">
<img src="docs/QuadRenderFront.png" alt="QuadRenderFlight" width="480"/>
<img src="docs/QuadRenderSide.png" alt="QuadRenderSide" width="480"/>
<img src="docs/QuadRenderBack.png" alt="QuadRenderFlight" width="480"/>
<img src="docs/QuadRenderFlight.png" alt="QuadRenderFlight" width="480"/>
</div>

### Custom PCB

In order to have complete control over what is happening in the flight controller, I selected and sourced necessary SMD components and designed a custom [flight controller pcb](https://github.com/TichyTech/rp2350-flight-controller). I ordered the double sided PCB from a manufacturer as well as the components and put it all together using solder paste and hot plate. This was my first time soldering on this scale, but it turned out pretty good I would say!

<div align="center">
<img src="docs/FC_PCB.jpg" alt="FC_PCB" width="640"/>
</div>

To my surprise, the PCB design did not have any major flaws except for the fact, that the RT6150B chip I used for 3.3V power supply to stay close to the [Pi Pico 2 Design](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf) is basically impossible to source nowadays. I was lucky and found that I had all the necessary leads exposed, so that I could simply design a cute tiny solder-on module to bypass the original power supply using a linear regulator instead. This power module is based around the AP7366-33W5 linear regulator IC. I just added some decoupling capacitors and an indication LED to it. Here is the module and its placement on the flight controller PCB:

<div align="center">
<img src="docs/AP7366module.jpg" alt="PowerModule" height="320"/>
<img src="docs/AP7366module_onPCB.jpg" alt="PowerModulePlacement" height="320"/>
</div>

And finally, blink example!

<div align="center">
<img src="docs/FC_PCB_Blink.gif" alt="Blinking" height="320"/>
</div>

To save some space, I also designed a custom power distribution board, which is really just  some copper to distribute the battery power and a 5V DC-Buck Converter based on the AP3211 IC to step down the battery voltage and power my flight controller. This chip is capable of providing up to 1.5A according to the datasheet!

<div align="center">
<img src="docs/PDB_schematic.png" alt="PDB_schematic" height="320"/>
<img src="docs/PDB_PCB.jpg" alt="PDB_photo" height="320"/>
</div>

The traces are kept unmasked, so I can put solder on them, decrease their resistance and thus reduce the voltage drop and heat they produce.

### Software overview
The whole project relies on platformio for dependencies and compilation for ease of use. The software is split in to three main parts: 
- [Flight Controller](FlighController)
- [Remote Controller](RemoteController)
- [Drone Plotter](DronePlotter)

The Flight Controller runs directly on the Quadcopter's main computer and serves as the main brain of this beast. The Remote Controller then runs on an Arduino Nano hooked up to NRF24 radio and some joysticks and buttons. The Drone Plotter, written in [Processing](https://processing.org/) runs on a laptop and visualises the quadcopter communication, sent by the Remote Controller over USB (through Serial communication).

On the lowest level of the Flight Controller, there are drivers that handle basic configuration of and communication with peripheral devices. These work using underlying communication protocols: 

| Component| Protocol |
|-|-|
| NRF24 (Radio) | SPI |
| ICM-42605 (Gyro + Acc) | SPI |
| BMP280 (Barometer) | SPI |
| LSM303DLHC (Mag + Acc) | I2C |
| M100 (GPS) | UART |
| HMC5883L  (Mag) | I2C |
| ESC | Oneshot42 |

On a higher level, the first RP2350 core performes the following four functions continuously:
1) Poll sensors for measurements
2) Estimate current drone attitude using quaternion EKF
3) Compute required motor control using a PID controller
4) Communicate motor controls to ESCs

The second RP2350 core then performs these two functions:
1) Receive reference state and configuration information via the NRF24 radio (from remote controller)
2) Send telemetry to remote controller

In the future, the second core will also be used for logging onto the onboard SD card.

# Hardware details

### Drivetrain
| Component|Specs |
|-|-|
| Motor | 2212 1000 KV Outrunner Motor |
| ESC   | Little Bee Blheli_S 40A 3-4S 8bit ESC |
| Propeller | 1045 Nylon Propellers |
| Battery | 3S 1300mAh 60C LiPo |

### Flight Controller

Flight Controller PCB highlights:
- Powering and Programming through USB-C even when battery powered
- RP2350 dual ARM core microcontroller running at 150MHz
- ICM-42605 Gyroscope and Accelerometer over SPI capable of up to 8kHz sampling rate
- NRF24L01+ with Low Noise Amplifier and Power Amplifier

For more info, see [https://github.com/TichyTech/rp2350-flight-controller](https://github.com/TichyTech/rp2350-flight-controller).

# Some more Photos

Here is the assembled quadcopter: 

<div align="center">
<img src="docs/QuadcopterPhotoProfile.jpg" alt="QuadCopterProfile" width="480"/>
<img src="docs/QuadcopterPhotoInsides.jpg" alt="QuadCopterInside" width="480"/>
</div>

And here from the bottom, where the battery resides under the neatly designed sliding cover:

<div align="center">
<img src="docs/QuadPhotoBotOpen.jpg" alt="QuadBotOpen" width="480"/>
<img src="docs/QuadPhotoBot.jpg" alt="QuadBot" width="480"/>
</div>

Also, due to the addition of Blue Leds to the arms (which are a real pain to put there), it looks pretty sweet at night too:

<div align="center">
<img src="docs/QuadPhotoProfileNight.jpg" alt="QuadCopterNight" height="320"/>
<img src="docs/QuadFlightNight.gif" alt="QuadCopterNightFlight" height="320"/>
</div>