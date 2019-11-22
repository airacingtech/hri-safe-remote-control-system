# To compile ROS driver:
```
catkin build hri_safety_sense
```

# To read from joystick, modify permissions to port:
```
sudo chmod 666 <port>
```
where port is for example `/dev/ttyACM1`.

# Set port in ROS parameter if different from default:
```
rosparam set /VscProcess/serial <port>
```
where `/VscProcess` is the ROS node name.

# Optionally, set frame ID of joystick message to a custom string:
```
rosparam set /VscProcess/frame_id /joystick
```

# To run driver:
```
. devel/setup.bash
roscore
rosrun hri_safety_sense safe_remote_control
```

# ROS parameters:

`serial`: Serial port, `/dev/ttyACM0` by default
`serial_speed`: Serial port speed, `115200` by default
`set_priority`: Set priority, `false` by default
`frame_id`: Frame ID, `/srcs` for Safe Remote Control System by default

# ROS topics:

`/safety/emergency_stop`: Boolean indicating whether the emergency stop is pressed
`/joy`: Status of joystick and buttons.

Example joystick message:
```
$ rostopic echo /joy
header:
  stamp:
    sec: 1574194053
    nanosec: 406973681
  frame_id: /joystick
axes: [-1023.0, -296.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]
```

`axes` fields are in this order for the 2 joysticks:
Left X, Left Y, Left Z, Right X, Right Y, Right Z
Note that Z axis is on the back of the controller.

`buttons` fields are in this order for the 8 buttons:
Left down, Left right, Left up, Left left, Right down, Right right, Right up, Right left

# Troubleshoot

Joystick goes to sleep automatically after some time. After turning it back on, press button 1 as indicated on the joystick screen to go into remote mode. Then values will show on `/joy` topic.
