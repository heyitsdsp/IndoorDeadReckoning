# IndoorDeadReckoning

<p align="center">
    <img src = "https://www.phunware.com/wp-content/uploads/2018/04/lbs-bluedot.gif" width="300" height="300" />
</p>

## Objective
To create an indoor navigation system based on dead reckoning, to track objects in a complex indoor environment. The critical concepts used in this project are as follows

* Yaw estimation (absolute heading)
* Step detection
* Step length estimation
* Altitude determination

## Software needed to replicate the project

* [Arduino IDE version 2.3.4](https://www.arduino.cc/en/software)
* [Processing version 4.3.1](https://processing.org/download) 

Please note that this project is not being updated for subsequent versions of the softwares used. Version errors often accompany new versions of software. In such a case, the recommended (last known working) versions of the softwares are mentioned for 1:1 replication. 

## Hardware needed to replicate the project

* [Arduino Nano 33 BLE](https://docs.arduino.cc/hardware/nano-33-ble/)
* [Arduino Nano 33 IoT](https://docs.arduino.cc/hardware/nano-33-iot/) (Used only as a BLE Beacon)
* [Gravity BMX160+BMP388 10 DOF Sensor SEN0252](https://wiki.dfrobot.com/Gravity%3A%20BMX160%2BBMP388%2010%20DOF%20Sensor%20SKU%3A%20SEN0252)

## Circuit Diagram

![Circuit](Circuit.png)

## Using the code

Upload the code from [Arduino/LSM9DS1_AHRS](https://github.com/heyitsdsp/IndoorDeadReckoning/tree/main/Arduino/LSM9DS1_AHRS) onto the Arduino Nano 33 BLE and upload the code from [Arduino/BLE_PerIoT](https://github.com/heyitsdsp/IndoorDeadReckoning/tree/main/Arduino/BLE_PerIoT) onto the Arduino Nano 33 IoT. 

Reset the Arduino Nano 33 BLE and let the device stay absolutely flat on a surface (a table is recommended). Open the Serial Monitor and follow the instructions to calibrate the magnetometer. You will need to move your setup in a figure-8 motion repeatedly, making sure to show the magnetometer all sides. This is shown in the image below.

<p align="center">
    <img src = "https://community.theta360.guide/uploads/default/original/2X/b/b1f52f295d3f2d1721cb59833be067c51ecf89e4.jpeg" width="400" height="200" />
</p>

Once the magnetometer is calibrated, set the device on the table again. After about 3 seconds, shake the device. If you have outputs in the Serial Monitor, then you are ready to open the Visualization.pde sketch from the [Visualization](https://github.com/heyitsdsp/IndoorDeadReckoning/tree/main/Visualization) folder and run it in the Processing IDE. 

Click on any square to set your start point. Once the GUI elements come to life, you should see a navy blue marker on the map where you clicked. This is the start point. Take steps with the device in hand in any direction and your marker in the 2D map will move along with you.

## Credits and libraries

The Madgwick filter and implementation of the driver for the LSM9DS1 on the Nano 33 BLE is wholly utilized from [github.com/radiohound/Nano33BLE](https://github.com/radiohound/Nano33BLE)

The BMP388 driver code is based on the examples provided by [Adafruit-BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX)

The BLE interfacing code is based on the examples provided by [Arduino-BLE](https://github.com/arduino-libraries/ArduinoBLE)

The Filters implemented are based on the examples provided by [EmotiBit_ArduinoFilters](https://github.com/EmotiBit/EmotiBit_ArduinoFilters)

To set up the BMX160, we use the code provided by [DFRobot-BMX160](https://github.com/DFRobot/DFRobot_BMX160)



