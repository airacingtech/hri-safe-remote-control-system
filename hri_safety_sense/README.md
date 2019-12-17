# To compile ROS 2 driver:
```
colcon build --packages-select hri_c_driver hri_safety_sense_srvs hri_safety_sense
. install/setup.bash
```

# To read from joystick, set up the port:

One way is to set up the udev rules for the device.
This automatically detects the port that the device is connected to, and
creates a symbolic link at `/dev/hri_vsc`.
```
sudo cp cfg/99-hrivsc.rules /etc/udev/rules.d/99-hrivsc.rules
sudo udevadm control --reload-rules
```
Then unplug and replug in the USB, or run this:
```
sudo udevadm trigger
```

A second way is to figure out the port manually, and modify the permissions to the port:
```
sudo chmod 666 <port>
```
where port is for example `/dev/ttyACM1`.

# To run driver:
```
ros2 run hri_safety_sense hri_joystick_node [__params:=cfg/params.yaml]
```

Or equivalently, use the launch file:
```
ros2 launch hri_safety_sense hri_joystick_node_launch.py
```

# ROS 2 parameters:

`serial`: Serial port, `/dev/ttyACM0` by default
`serial_speed`: Serial port speed, `115200` by default
`set_priority`: Set priority, `false` by default
`frame_id`: Frame ID, `/srcs` for Safe Remote Control System by default

These can be manually changed in `cfg/params.yaml`.

# ROS 2 topics:

`/safety/emergency_stop`: Boolean indicating whether the emergency stop is pressed
`/joy`: Status of joystick and buttons.

Example joystick message:
```
$ ros2 topic echo /joy
header:
  stamp:
    sec: 1574194053
    nanosec: 406973681
  frame_id: /srcs
axes: [-0.21114370226860046, -1.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0]
```

`axes` fields are in this order for the 2 joysticks:
Left X, Left Y, Left Z, Right X, Right Y, Right Z
Note that Z axis is on the back of the controller.

`buttons` fields are in this order for the 8 buttons:
Left down, Left right, Left up, Left left, Right down, Right right, Right up, Right left

# Troubleshoot

Joystick goes to sleep automatically after some time. After turning it back on,
press button 1 as indicated on the joystick screen to go into remote mode.
Then values will show on `/joy` topic.

If you get an error about serial port I/O error or cannot open serial port,
check that the port name is correct.
After USB connection, if the SRC is powered on, and the E-stop is not down, the
VSC light should flash green.
If VSC is flashing red under these conditions, wait a minute or two after
plugging in the USB for the VSC to be ready. Before it is ready, the device ID
might be 2a99:b003, as opposed to the expected 2a99:d002. The udev rules will
automatically set up the symbolic name after the VSC is ready, so that it finds
the expected device ID.
