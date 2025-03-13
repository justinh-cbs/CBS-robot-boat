Connect the Polulu IMU to the Jetson Nano with the following pins:

VCC = 17
Ground = 20
Data = 27
Clock = 28

* Pins are board numbering, not Broadcom SOC channel (GPIO numbering)


To calibrate the magnetometer, run:

ros2 run imu calibrate_magnetometer

It will collect values while you rotate the robot, and save a calibration file.
