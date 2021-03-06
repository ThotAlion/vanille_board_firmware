# Firmware of the peripheral board of quadruped Vanille robot

The Vanille robot is a quadruped DIY made with 12DOF brushless motors controlled by [Tinymovr cards](https://tinymovr.com/).

![](./images/IMG_3785.JPG)![](./images/IMG_3828.JPG)![](./images/IMG_3968.JPG)

The Tinymovr gives access to a great quantity of sensors (relative position very accurate, speed, equivalent current - torque, temperature of the boards...) but they are not sufficient. In this robot we identified this sensors to add :

- absolute position sensors to identify the zero of each articulation
- relative inertial measurements with respect to gravity

## Absolute position sensors vs Mechanical stop zeroing

The position given by the Tinymovr board is the absolute position of the rotor bell of the brushless motor. But in the Vanille quad, there is a 9:1 belt reductor. As you have to make several turn to pilot the final axis of the actuator, you cannot, from scratch, know the position of the actuator.
To guess the zero of the motors, we have several solutions :

- When you switch on the robot, the robot is constrained, during an external procedure in a precise position, geomtrically known. Then, push a button of "ZERO" to save the position value as offset with respect to the current value.
- When you switch on the robot, the robot moves by himself articulation by articulation to touch mechanical stop. To detect mechanical stop, you can use switches or measure the current inside the motor. When it reaches a certain value, you can deduce the offset to apply to the current arbitrary offset.
- You can add another absolute encoder on the final axis of the actuator. Then you knwo exactly its position. If the relative position is different to the absolute position, then correct the offset with this computed difference. This algorithm can be executed very regurlarly if the "belts slide".
- Finally, over 12 degrees of freedom, you can choose a bit all of the previous solutions. Considering one leg, there are the articulations hip-x, hip-y and knee-y. If you do not know the hip-x angle, reaching the mechanical stops can be very dangerous since the robot can lie on ground. Knowing this angle by an absolute encoder is very useful. On the hip-y and knee-y, it could be also useful but you have to run a cable to the absolute sensors. In such a mechanical architecture, the robot shall have light legs and makes very dynamic moves. The method using mechanical stops can then be more reliable.

The absolute magnetic encoders chosen are the same used by Tinymovr board : the [MA702 of Monolithicpower](https://www.monolithicpower.com/en/ma702.html) It is magnetic encoder giving the accurate absolute angle of a magnet turning in front from 0 (0deg) to 65535(360deg). 

![](https://www.mev-elektronik.com/isotope/m/maxxx_sideshaft.jpg)

The data is sent via SPI bus. If there are multiple MA702, plug them on the same SPI bus with different Chip select pins. The chip select is to be specified in the SPI master code. The default SPI frequency is 10MHz, as the wires shall be longs and the data packet is very small, do not hesitate to reduce the frequency to avoid noise. 10kHz is largely sufficient.

Take care about magnet mounting, it shall be tunable ! As the absolute position is to be measured, the zero of the magnet shall not be inside the operational zone. It is possible to manage the discontinuity between 0 (0deg) and 65535(360deg) during runtime but not at initialization. The objective of this magnet is to remove ambiguity about "on which turn are we ?" If the zero is inside the operational zone, we are on two turns... impossible to guess at the beginning.

![System schematic](./absolute_angle.svg)

## Inertial measurement

Giving the absolute angles of the main body of the robot needs an IMU. Finding the good component for this kind of very dynamic condition with a good amount of reliability is not very easy. We tested the following products :

- [Naveol IMU V1.0](http://www.naveol.com/index.php?menu=product&p=2) : very expensive, very accurate but for very slow motions
- [Sparkfun RAZOR IMU](https://www.sparkfun.com/products/retired/14001?_ga=2.143596407.331610333.1619851408-1697925503.1614425594) : not so expensive, a good IMU giving data via UART bus
- [Yoctopuce Yocto-3D](http://www.yoctopuce.com/EN/products/usb-position-sensors/yocto-3d) : a good IMU for dynamic moves but USB connectivity does not allow Arduino management
- [MPU6050](https://www.gotronic.fr/art-accelerometre-et-gyroscope-3-axes-mpu6050-20238.htm) : Accurate IMU and very cheap, but the I??C communication is not standard for absolute quaternion measurement. Several crashes. Ideal if you need only angle rates and accelerations. Bad documentation.
- [Bosch BNO055](https://www.gotronic.fr/art-module-9-dof-ada2472-23896.htm) : Is the same format of MPU6050 but far more robust, good accuracy and good reactivity. **The best tested for now.**

The IMU chosen is then the BNO055.

![](https://ae01.alicdn.com/kf/HTB17PfxKbGYBuNjy0Foq6AiBFXam.jpg)

## Sensor board

To gather all the data to send to the main computation board (with ROS2) we can use a classical Arduino. To be at ease and have the ability to add future peripherals, the board chosen is [Arduino Nano Every](https://store.arduino.cc/arduino-nano-every) very simple and powerful.

![](./images/IMG_3988.jpg)

Library used :
- For the [MA702](https://github.com/monolithicpower/MagAlpha-Arduino-Library)
- For the [BNO055](https://github.com/adafruit/Adafruit_BNO055)

![System schematic](./draft.svg)

Here is the ascii art schematic : 

```
                      +-----+
         +------------| USB |------------+
         |            +-----+            |
  MAG702 | [ ]D13/CLK        MISO/D12[ ] | MAG702   
         | [ ]3.3V           MOSI/D11[ ]~| MAG702  
         | [ ]V.ref     ___    SS/D10[ ]~|   
         | [ ]A0/D14   / N \       D9[ ]~| 
         | [ ]A1/D15  /  A  \      D8[ ] | 
         | [ ]A2/D16  \  N  /      D7[ ] | 
         | [ ]A3/D17   \_0_/       D6[ ]~| 
     BNO | [ ]A4/D18/SDA           D5[ ]~| 
     BNO | [ ]A5/D19/SCL           D4[ ] |   
         | [ ]A6                   D3[ ]~| CSFRONTRIGHTHIPX  
         | [ ]A7                   D2[ ] | CSFRONTLEFTHIPX 
         | [ ]5V                  GND[ ] |     
         | [ ]RST                 RST[ ] |   
         | [ ]GND                 RX1[ ] | UPBOARD  
         | [ ]Vin                 TX1[ ] | UPBOARD  
         |                               |
         |                               |
         | NANO EVERY                    |
         +-------------------------------+
```

And the code for just 2 legs is [here](./vanille_board_firmware/vanille_board_firmware.ino)

The data are send on the Serial port 1 at 115200 Bauds in a JSON format. Having the signal at 100Hz is sufficient for now and the JSON data rate is cumfortable (10ms)

## ROS2 side
To the ROS2 side, a node is being developed to publish on topics absolute angle measurement (JointState format) and IMU measurement (Imu format)
TBC

