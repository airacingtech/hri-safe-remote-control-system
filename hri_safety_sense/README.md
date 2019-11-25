# To compile ROS driver:
```
catkin build hri_safety_sense
```

# To read from joystick, modify permissions to port:
```
sudo chmod 666 <port>
```
where port is for example `/dev/ttyACM1`.

# To run driver:
```
. devel/setup.bash
roscore
roslaunch hri_safety_sense safe_remote_control.launch [serial:=/dev/ttyACM1 frame_id:=/joystick]
```

Optionally pass in serial port and/or joystick frame ID, if different from default values in launch file.

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
  seq: 892
  stamp: 
    secs: 1574381173
    nsecs: 920995966
  frame_id: "/joystick"
axes: [-0.21114370226860046, -1.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]
```

`axes` fields are in this order for the 2 joysticks:
Left X, Left Y, Left Z, Right X, Right Y, Right Z
Note that Z axis is on the back of the controller.

`buttons` fields are in this order for the 8 buttons:
Left down, Left right, Left up, Left left, Right down, Right right, Right up, Right left

# Troubleshoot

Joystick goes to sleep automatically after some time. After turning it back on, press button 1 as indicated on the joystick screen to go into remote mode. Then values will show on `/joy` topic.
