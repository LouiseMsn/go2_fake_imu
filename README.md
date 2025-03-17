## Fake IMU for Go 2 (unitree)

This code fakes the ```/lowstate``` ros2 topic published by the Go2 robot.

Parts of the topic currently faked by this code:
- lowstate_msg.imu_state.accelerometer
- lowstate_msg.imu_state.gyroscope
- lowstate_msg.imu_state.quaternion
- lowstate_msg.tick


### Usage
For now the accelerometer, gyroscope and relative noise values are set directly in the code and will need to be changed there (l95 of fake_imu.cpp)
```C++
            // "pure" value of accelerometer
            a_.value << 0,0,0;  

            // mean of accelerometer noise
            a_.noise_mean << 0,0,0;

            // standard deviation of accelerometer noise
            a_.noise_standard_dev << 0,0,0;

            // mean of accelerometer bias
            a_.bias_mean << 0,0,0;

            // standard deviation of accelerometer bias
            a_.bias_standard_dev <<0,0,0;
            
            // "pure" gyroscope value
            g_.value <<  M_PI*0.1,0, 0; //M_PI*0.1 = 180° if the runtime is 10secs

            // mean of gyroscope value
            g_.noise_mean << 0,0,0;

            //standard deviation of gyroscope value
            g_.noise_standard_dev << 0,0,0;
```

Launch with: 
``` bash
ros2 launch go2_fake_imu fake_imu.launch.py
```

Some options are available when launching:
|Name|Default|Values|Description|
|:--:|:--:|:--:|:--:|
|gravity|1|1 or 0|Adds gravity to accelerometer readings|
|debug|0|1 or 0|Prints current acceleration & acceleration norm and linear speed sent as well as current rotation |
|pub_freq|500| [0; +inf[|Frequency at which the fake imu will publish on `/lowstate` topic|
|run_time|10|[0; +inf[| Run time of the IMU, will stop publishing once timer is reached|