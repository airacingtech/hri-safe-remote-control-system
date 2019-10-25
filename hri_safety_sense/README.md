# To compile ROS 2 driver:
```
colcon build --packages-select hri_safety_sense_interfaces
colcon build --packages-select hri_safety_sense
```

# To run driver:
```
. install/setup.bash
ros2 run hri_safety_sense safe_remote_control
```

# ROS 2 parameters:

`serial`: Serial port, `/dev/ttyACM0` by default
`serial_speed`: Serial port speed, `115200` by default
`set_priority`: Set priority, `false` by default

