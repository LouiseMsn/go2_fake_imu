## Fake IMU for Go 2 (unitree)

This code fakes the accelerometer and gyoscope part of the ```/lowstate``` ros2 topic of the Go2 robot.

For now the linear acceleration and angular velocity are decoupled.

### Usage
The run time of the IMU can be changed in RUN_TIME, the frequency of publishing can be changed in PUBLISHING_FREQ.