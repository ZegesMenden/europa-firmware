# europa-firmware

This repository contains the europa-firmware source code written by cameron kullberg for the europa R2 flight computer system.

Europa is capable of autonomous propulsive landing, using a custom algorithm called G-FIELD (Guidance for Fixed Impulse Eficcient Landing Diverts), and uses in-flight simulation along with live trajectory planning to divert the rocket and bleed energy for a smooth and (ideally) repeatable landing, regardless of the ignition delay / performance variation of a solid rocket motor.
For more implememntation details on G-FIELD, see control.h

Europa fuses multiple sensors into a state estimation using complimentary filters and three linear asynchronous Kalman filters running at 100Hz - for the implementation details of the sensor fusion, see navigation.h 

This code runs on a raspberry pi RP2040 with LSM6DSO32 IMU, BMP581 barometer, LIS2MDL magnetometer, and ch32v003 RISC-v core for GPIO peripherals.
The current GNC revision is for the propulsive landing rocket.
