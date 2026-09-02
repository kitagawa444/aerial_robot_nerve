# Spinal Link architecture

## Runtime topology

```text
Spinal STM32
  Flight control / supervisor / health / ESKF
  Micro XRCE-DDS Client (no rcl, rclc, or rmw)
  USART1, 921600 baud
          |
          | micro-USB -> FT232 -> USART1
          v
MicroXRCEAgent on aircraft Linux (ROS-independent process)
          |
          v
Fast DDS domain 0
  aerial/spinal_link/<message>  One configured DDS topic per message
          |
          +-- gcs_gateway (ROS-independent)
          |     MAVLink 2 / UDP -> Wi-Fi GCS
          |
          +-- spinal_ros2_bridge (optional)
                ROS 2 topics for aerial_robot_base, rqt, rosbag, and RViz

ELRS receiver -> CRSF -> USART3 -> Spinal flight control directly
```

OpenHD is intentionally not implemented in this change. It can later be added
as another MAVLink transport behind `gcs_gateway`, without changing the
Spinal wire protocol or flight control.

## Ownership boundaries

- Spinal owns flight state, command arbitration, failsafe, health monitoring,
  estimation, attitude control, and position control.
- `MicroXRCEAgent` only bridges the serial DDS-XRCE session into Fast DDS. It
  contains no ROS node and makes no flight decisions.
- `gcs_gateway` translates neutral Spinal Link frames to and from MAVLink.
  It does not link against ROS 2.
- `spinal_ros2_bridge` is the only new component that depends on `rclcpp`.
  Removing it disables ROS access without changing the FC or Wi-Fi GCS path.
- The bridge preserves the existing flight configuration topics and the rqt
  parameter UI. Config Flash, flight-parameter commit/reload, normal reboot,
  and ROM-bootloader requests are neutral request/response frames on the DDS
  link; ROS services terminate at the bridge rather than inside Spinal.
- CRSF remains independent of USART1, DDS, ROS, Wi-Fi, and the aircraft Linux
  computer.

Configuration and parameter snapshots use fixed-width wire structures. They
are deliberately not raw STM32 C++ structures, because `size_t`, padding, and
alignment differ between Cortex-M7 and the Linux host. Compile-time layout
checks and conversion tests protect this boundary.

## DDS topic and delivery configuration

`config/spinal_link_topics.yaml` is the single source of truth for the STM32
and Linux DDS entities. Each message independently selects its topic name,
direction, enabled state, reliability, durability, and history depth. Topics
that are not required by a target can be removed from both endpoints by setting
`enabled: false` and regenerating the shared table:

```bash
python3 spinal/scripts/generate_spinal_link_topics.py \
  --config spinal/config/spinal_link_topics.yaml \
  --output spinal/mcu_project/lib/My_Lib/communication/spinal_link_topics.generated.h
```

The build fails if the generated table is stale. High-rate IMU, state,
setpoint, and heartbeat traffic uses best effort with a short history. Flight
status and management traffic use reliable DDS.

For safety-sensitive host-to-Spinal messages, `application_ack: true` adds a
nonzero request ID, a `PROTOCOL_ACK`, timeout/retry handling, and source-aware
deduplication. DDS reliability handles packet delivery; the application ACK
confirms that Spinal decoded and attempted the request. A retry with the same
source, message ID, and request ID is acknowledged again without executing the
operation twice. These features can be selected independently per message in
the same YAML file, subject to the generator's safety validation.

## Build and run

Build the workspace packages normally. The STM32 build downloads the pinned
Micro XRCE-DDS Client sources directly; the old generated micro-ROS archive is
not used.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select spinal_firmware spinal
source install/setup.bash
```

Install the standalone Agent once. The optional argument selects its install
prefix; without it the script uses the user's local data directory.

```bash
src/aerial_robot_nerve/spinal/scripts/build_micro_xrce_agent.sh
export PATH="$HOME/.local/share/aerial_robot/micro_xrce_agent/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/share/aerial_robot/micro_xrce_agent/lib:${LD_LIBRARY_PATH:-}"
```

Launch the Agent, Wi-Fi GCS Gateway, and ROS 2 Bridge together:

```bash
ros2 launch spinal spinal_link.launch.py \
  dev:=/dev/ttyUSB0 \
  use_ros2_bridge:=true \
  wifi_interface:=wlan0 \
  mavlink_remote_address:=255.255.255.255 \
  mavlink_remote_port:=14550
```

Set `use_ros2_bridge:=false` for a ROS-independent runtime. Although the launch
command itself uses ROS launch as a process supervisor, the Agent and Gateway
remain ordinary non-ROS executables and can instead be started by systemd:

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 -v 4
MAVLINK_BIND_ADDRESS=0.0.0.0 \
MAVLINK_BIND_PORT=14550 \
MAVLINK_REMOTE_ADDRESS=255.255.255.255 \
MAVLINK_REMOTE_PORT=14550 \
gcs_gateway
```

The XRCE participant is currently fixed to DDS domain 0 in the firmware. Keep
`spinal_link_dds_domain_id:=0` unless the firmware participant domain is changed at
the same time.

## MAVLink support in this change

The Gateway sends `HEARTBEAT`, `LOCAL_POSITION_NED`, `HIGHRES_IMU`,
`BATTERY_STATUS`, and `STATUSTEXT`. It accepts arm/disarm, takeoff, land,
flight termination, the project force-land command (`31010`), and local-NED
position targets. Accepted flight commands receive `IN_PROGRESS` immediately
and a final `COMMAND_ACK` after Spinal reports the supervisor result.

Any valid inbound MAVLink packet refreshes Spinal's network-link heartbeat.
The optional ROS 2 Bridge maintains the ROS-link heartbeat separately, so Wi-Fi
link health and ROS graph health do not overwrite each other.

The current ROS compatibility surface includes flight commands and ACKs,
external state, position setpoints, airframe/gimbal/controller configuration,
health configuration, Application/Config/Flight Parameter status, the rqt
Config Flash and parameter services, normal reboot, and `/enter_bootloader`.
Peripheral-specific calibration and servo-monitor traffic remains in the
existing ROS-side tools and is not forwarded through Spinal Link
in this change.

## Communication monitoring

The ROS-independent `CommunicationMonitor` combines locally observed DDS and
ACK statistics with the ROS, network, and CRSF states reported by Spinal. The
optional ROS bridge publishes the typed result and standard diagnostics:

```text
<robot namespace>/fc/communication_status
/diagnostics
```

`wifi_interface` selects the Linux interface whose operational state and IPv4
assignment are checked. Leave it empty when the vehicle has no Wi-Fi interface,
as in a local simulation. Wi-Fi interface state and MAVLink GCS activity remain
separate fields so an associated interface is not mistaken for a working GCS
session.

The rqt plugin is available under **Plugins > aerial robot > Spinal Link
Monitor**. It shows Micro-USB/XRCE-DDS, ROS, Wi-Fi/MAVLink, ELRS/CRSF, and ACK
delivery in one table. If Micro-USB data becomes stale, remote RC and network
states are displayed as unknown rather than incorrectly declaring those
independent links disconnected.
