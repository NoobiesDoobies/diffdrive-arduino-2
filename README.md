# diffbot_control

This controller uses the `ros2_control` and `ros2_controllers` framework. The `diffbot_control` library serves as the hardware interface by receiving serial data from two Arduinos. The first Arduino is used for motor control and reading interface, while the second one is used for reading IMU data.


The available serial requests are:

```cpp
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define READ_IMU       'i'
#define CALIBRATE_IMU  'j'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
```

The diffdrive controllers are described in `bringup/config/diffbot_controllers.yaml`, which is the `diff_drive_controller/DiffDriveController` controller. It is spawned by `controller_manager` in `bringup/launch/diffbot.launch.py`.

This code is an extension of the [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) repository, specifically [example_2](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2).