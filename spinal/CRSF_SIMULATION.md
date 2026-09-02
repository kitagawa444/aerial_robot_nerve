# CRSF simulation in Gazebo

Gazebo starts `crsf_simulator.py` by default. The simulator creates
`/tmp/spinal_crsf_sim`, writes valid packed CRSF frames at 100 Hz, and feeds the
existing `CrsfPosixSerialTransport`. Consequently, CRC validation, channel
decoding, RC authority, teleoperation events, the flight supervisor, and flight
controllers use the same path as UART3 on the real Spinal board.

No joystick, ELRS transmitter, or ELRS receiver is required.

## Automated flight test

```bash
ros2 launch mini_quadrotor crsf_gazebo_test.launch.py
```

The test checks CRSF connection, ARM, TAKEOFF, forward/yaw stick input, LAND,
altitude gain, landing completion, and automatic disarm. It also verifies the
Gazebo `fc/communication_status` and `/diagnostics` output, including simulated
CRSF link quality and an explicit link-loss/recovery cycle. Micro-USB/XRCE-DDS
and protocol ACK are reported as disabled because the Gazebo FC uses the
in-process transport. Gazebo shuts down when the test completes.

## Manual flight commands

Start the normal Gazebo simulation, then call the simulated CRSF controls from
another terminal:

```bash
ros2 service call /mini_quadrotor/rc/sim/arm std_srvs/srv/Trigger '{}'
ros2 service call /mini_quadrotor/rc/sim/takeoff std_srvs/srv/Trigger '{}'
ros2 service call /mini_quadrotor/rc/sim/land std_srvs/srv/Trigger '{}'
ros2 service call /mini_quadrotor/rc/sim/force_landing std_srvs/srv/Trigger '{}'
ros2 service call /mini_quadrotor/rc/sim/halt std_srvs/srv/Trigger '{}'
```

Continuous stick input uses axes in `lateral, forward, vertical, yaw` order:

```bash
ros2 topic pub -r 20 /mini_quadrotor/rc/sim/joy sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.3, 0.0, 0.2]}"
```

The axes automatically return to neutral 0.5 seconds after publishing stops.
Link loss and recovery can also be tested:

```bash
ros2 service call /mini_quadrotor/rc/sim/link std_srvs/srv/SetBool '{data: false}'
ros2 service call /mini_quadrotor/rc/sim/link std_srvs/srv/SetBool '{data: true}'
```

Set `simulate_crsf:=false` when launching `bringup_launch.py` to disable the
pseudo-terminal receiver.
