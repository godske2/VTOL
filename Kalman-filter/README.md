# Extended Kalman Filter
Kalman filter for the VTOL project.
The Arduino code is adapted to a Bosch BNO055 10 DOF IMU. The main board to run this is the Arduino Nano 33 IoT.
The structure of the files are as follows:
* EKF.h - header file for running the Kalman filter algorithm and all the matrix math going on
* EKF_samd.ino - main file for running and tuning the filter (adapted to the samd architecture) 
* IMU.cpp - Data request for the Bosch BNO055 (Can be downloaded inside Arduino IDE)
* IMU.h - References the IMU.cpp function calls (Can be downloaded inside Arduino IDE)
* MKRIMU.h - Specific model library for the BNO055 (Breakout board from arduino, can be downloaded inside Arduino IDE)
