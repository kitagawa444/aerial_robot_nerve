# Setup

## Create ROS2 workspace
! NOTE: When building this package while building `aerial_robot_base`, skip creating a new workspace and clone the repo in the existing workspace. 
```bash
source /opt/ros/humble/setup.bash
mkdir -p uros_ws/src && cd uros_ws
```
## Clone repository
```bash
# Add repository to the workspace
vcs import src <<EOF
repositories:
  aerial_robot_nerve:
    type: git
    url: 'https://github.com/ut-dragon-lab/aerial_robot_nerve.git'
    version: master
EOF
# Install depended repositories for spinal
vcs import src < src/aerial_robot_nerve/spinal.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```
## Generate micro ROS libraries
! If you are using a Docker container to run ROS 2, please execute the following outside the container on your local machine, as it itself relies on pulling a Docker image.

The following code requires Docker to be installed on your system.
```bash
python3 src/aerial_robot_nerve/spinal/scripts/make_microros_libraries.py --support_rtos
```
Then, all files required by micro ROS will be generated and placed in the appropriate directory within the STM32 project.

## Build and update the STM32 firmware from ROS 2

The `spinal_firmware` package cross-compiles the STM32H743 firmware as part of a
colcon build. Run the explicit setup target once in the PC or persistent Docker
environment before building both packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select spinal_firmware \
  --cmake-target setup_stm32_environment
colcon build --packages-select spinal_firmware spinal
source install/setup.bash
```

When the programmer bundle is mounted at `/opt/st/programmer` in Docker, add
`--cmake-args -DSTM32_PROGRAMMER=/opt/st/programmer/bin/STM32_Programmer_CLI`
to the setup command. The target then validates the mounted copy instead of
trying to install another copy inside the container.

The setup target is idempotent. It installs any missing compiler and C/C++
runtime apt packages. On x86_64 it installs the official STM32CubeProgrammer
2.23 bundle through ST's `cube` bundle CLI. On aarch64 it downloads the
official STM32CubeProgrammer 2.23 ARM64 Debian package from the project Google
Drive, verifies its pinned SHA-256, and installs it with apt. Apt may ask for
the local sudo password. The separate setup remains necessary because
the standard rosdep database has no
key for Ubuntu's `libstdc++-arm-none-eabi-newlib` package. It is deliberately
not part of the default build, so an ordinary `colcon build` never runs sudo,
downloads tools, or changes the environment. Non-mutating checks are available:

```bash
colcon build --packages-select spinal_firmware --cmake-target check_stm32_tools
colcon build --packages-select spinal_firmware --cmake-target check_stm32_programmer
```

Invoking `setup_stm32_environment` explicitly downloads and installs the ST
package and accepts its bundled SLA0048 license prompt. The package retains the
license and third-party notices under `/usr/bin/doc` and `/usr/share/doc`.

The installed artifacts are:

```text
install/spinal_firmware/share/spinal_firmware/firmware/spinal.elf
install/spinal_firmware/share/spinal_firmware/firmware/spinal.bin
install/spinal_firmware/share/spinal_firmware/firmware/spinal.hex
```

This board's microUSB connector is routed through its FT232 to STM32 USART1; it
is not connected directly to the STM32 USB DFU pins. The tested STM32H743 Rev V
reports ROM bootloader `0x92`. A first `GET ID` can be NACKed while the loader
is starting, but retrying establishes a connection and UART programming plus
byte-for-byte verification succeeds at 115200 baud. The remaining limitation is
exit: after `GO`, this ROM revision retains the volatile system-memory boot
address and returns to ROM. Its RAM `GO` and bootloader reset commands also do
not provide a usable software-only exit on the tested unit. UART updates thus
need one power cycle after successful verification. ST-LINK/SWD remains the
verified default for a completely unattended update on this board revision.

On x86_64, the setup script finds ST's `cube` CLI on `PATH`, through
`STM32_CUBE_CLI`, or inside the official STM32 VS Code extension pack, and runs:

```bash
cube bundle install --yes programmer@2.23.0
```

The repository pins the shared ARM64 package URL and SHA-256, so a new VIM4
requires no manual CubeProgrammer download. A local package can still override
the shared artifact:

```bash
colcon build --packages-select spinal_firmware \
  --cmake-args -DSTM32_PROGRAMMER_DEB=/path/to/stm32cubeprogrammer_2.23.0_arm64.deb \
  --cmake-target setup_stm32_environment

# Or, override both URL and checksum together:
colcon build --packages-select spinal_firmware \
  --cmake-args \
    -DSTM32_PROGRAMMER_DEB_URL='https://...' \
    -DSTM32_PROGRAMMER_DEB_SHA256='<sha256>' \
  --cmake-target setup_stm32_environment
```

The x86_64 bundle is installed below
`~/.local/share/stm32cube/bundles/programmer/2.23.0`. In a disposable Docker
container, pass the FT232 and current ST-LINK USB device and mount that bundle
read-only. This leaves the VIM4 unchanged:

```bash
docker run --rm \
  --device=/dev/ttyUSB0 \
  --device=/dev/bus/usb/BBB/DDD \
  --mount type=bind,src=${HOME}/.local/share/stm32cube/bundles/programmer/2.23.0,dst=/opt/st/programmer,readonly \
  ...
```

Get `BBB/DDD` from the `Bus` and `Device` columns of `lsusb` for USB ID
`0483:3748`. The number can change after reconnecting ST-LINK.

Add the machine's user to the group that owns the serial device (normally
`dialout`). Use the stable `/dev/serial/by-id/...` path when possible:

```bash
ros2 run spinal flash_firmware.py \
  --interface swd \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_XXXXXXXX-if00-port0
```

The default installed firmware is `spinal.bin`. A programmer outside `PATH` can
be selected with `--programmer /path/to/STM32_Programmer_CLI`.

Build and flash can be run as one explicit CMake target. Configure the stable
FT232 path and the programmer mounted in Docker on the first invocation:

```bash
colcon build --packages-select spinal_firmware \
  --cmake-args \
    -DSTM32_FLASH_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_XXXXXXXX-if00-port0 \
    -DSTM32_PROGRAMMER=/opt/st/programmer/bin/STM32_Programmer_CLI \
  --cmake-target flash_stm32
```

`flash_stm32` is an alias of the verified `flash_stm32_swd` target. It first
builds `spinal.bin`, then programs the board. It is not an `ALL` target; normal
builds never reset or rewrite attached hardware. `flash_stm32_uart` selects the
FT232/ROM path. The H743 ROM loader can acknowledge UART activation before it
is ready to accept `GET ID`, so the updater performs non-destructive connection
probes for up to 30 seconds before it permits Flash erase or programming.

On the VIM4, the complete setup and UART build/flash flow is:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select spinal_firmware \
  --cmake-target setup_stm32_environment

colcon build --packages-select spinal_firmware \
  --cmake-args \
    -DSTM32_FLASH_BACKEND=cubeprogrammer \
    -DSTM32_FLASH_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_XXXXXXXX-if00-port0 \
  --cmake-target flash_stm32_uart
```

The updater performs this sequence:

1. Calls `/enter_bootloader` (`std_srvs/srv/Trigger`).
2. The STM32 latches every motor output at idle and acknowledges the request.
3. The STM32 changes the volatile CM7 boot-address shadow (`SYSCFG->UR2`) to
   system memory and resets, keeping the board quiescent while it is rewritten.
4. The updater releases the FT232 from `micro_ros_agent`. For UART it waits for
   a successful ROM-loader `GET ID`; for SWD it first restores the volatile boot
   address to `0x08000000`.
5. It programs and verifies `spinal.bin`, then starts the application and
   restarts the agent. It waits for `/enter_bootloader` to reappear before
   reporting completion. On the tested ROM `0x92`, UART `GO` returns to ROM;
   the updater therefore reports that programming succeeded but a power cycle
   is required instead of claiming a complete update.

For a UART update, power-cycle the board only after CubeProgrammer prints
`Download verified successfully`.
When the updater prints
`waiting for application service /enter_bootloader`, unplug the board's
microUSB power, wait about one second, and reconnect it.

If exactly one `micro_ros_agent` process owns the serial port, the updater can
stop and restart it automatically. For a systemd-managed agent, pass explicit
commands so systemd does not immediately reclaim the port:

```bash
ros2 run spinal flash_firmware.py \
  --interface swd \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_XXXXXXXX-if00-port0 \
  --agent-stop-command "sudo systemctl stop micro-ros-agent" \
  --agent-start-command "sudo systemctl start micro-ros-agent"
```

For an explicit UART experiment on a board already in system memory, use:

```bash
ros2 run spinal flash_firmware.py \
  --interface uart \
  --skip-bootloader-request \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_XXXXXXXX-if00-port0
```

The first installation of firmware containing `/enter_bootloader`, and recovery
from corrupted or non-booting firmware, need ST-LINK. The updater writes only
the application image and does not request a mass erase, preserving the
configuration sector at `0x081e0000`. Fully unattended updates without ST-LINK
require either a board revision exposing BOOT0 and NRST to the host, or a custom
Flash-resident bootloader that does not enter the factory ROM loader.

## ExpressLRS / CRSF receiver on UART3

The STM32H7 v2 configuration enables `CRSF_RC_INPUT` on UART3. UART3 is set to
420000 baud, 8-N-1, uninverted, full duplex. Because GPS previously used the
same UART, `GPS_FLAG` and `CRSF_RC_INPUT` are mutually exclusive compile-time
options in `configs/STM32H7_v2/config.h`.

Wire the ExpressLRS receiver to the v2 board's J7 connector as follows:

| Receiver pad | J7 pin | STM32 signal |
| --- | ---: | --- |
| TX | 1 | USART3_RX / PD9 |
| RX | 2 | USART3_TX / PD8 (optional until telemetry is implemented) |
| GND | 3 | GND |
| 5V | 4 | +5V |

The receiver's TX and the board's RX must be crossed. The firmware validates
the CRSF CRC8 before accepting a frame, decodes all 16 packed channels, and
marks the link disconnected when no valid RC channel frame arrives for 100 ms.
It does not feed RC values directly into flight control or motor outputs.

The following root topics are published and also relayed by
`spinal_namespace_bridge` into the configured robot namespace:

```text
/rc/joy           sensor_msgs/msg/Joy  # axes[0]..axes[15], normalized -1..1
/rc/connected     std_msgs/msg/Bool
/rc/link_quality  std_msgs/msg/UInt8   # uplink link quality, 0..100
```

After building, flashing, and starting the micro-ROS agent, verify reception
before connecting the values to flight control:

```bash
ros2 topic echo /rc/connected
ros2 topic hz /rc/joy
ros2 topic echo /rc/joy --once
```

Channel order and transmitter switch assignment are intentionally left in the
native CRSF order. Mapping roll, pitch, yaw, throttle, arm, and kill switches
is a separate safety layer and should include an explicit failsafe policy.

### Nano TX without a handset

A PC or VIM4 can provide the handset-side CRSF stream through the Nano TX V2
USB serial port. The standalone script does not require ROS to be running. Its
safe defaults use the conventional AETR order: CH1, CH2, and CH4 are centred,
CH3 (throttle) is low, and CH5 through CH16 are low.

Keep the Nano TX antenna attached whenever the module is powered. With motors
disconnected, reconnect the Nano TX USB cable so it leaves Wi-Fi mode, then run:

```bash
python3 spinal/scripts/send_crsf_channels.py

# After building and sourcing the ROS 2 workspace, the installed form is:
ros2 run spinal send_crsf_channels.py
```

The QinHeng USB serial device is detected automatically. An explicit stable
device path can also be used:

```bash
ros2 run spinal send_crsf_channels.py \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_XXXXXXXX-if00
```

The default handset-side baud rate is `115200`, which the ExpressLRS TX module
auto-detects. Although a module-bay CRSF connection commonly uses `400000`, the
Nano TX V2 QinHeng USB CDC bridge stalled when that non-standard rate was
requested on the tested Ubuntu host. It sustained the default 250 Hz stream at
`115200`.

Values are raw 11-bit CRSF values (`172` low, `992` centre, `1811` high).
Override individual one-based channels with repeatable `--set` arguments:

```bash
# Show the generated frame without using hardware.
ros2 run spinal send_crsf_channels.py --dry-run --set 1=1100

# Send for ten seconds, then stop and let the receiver enter failsafe.
ros2 run spinal send_crsf_channels.py --duration 10 --set 1=1100
```

Pressing Ctrl-C stops the CRSF stream. Stopping the stream must be treated as a
receiver failsafe; this script is only a fixed-channel bring-up utility and is
not a flight-control input or an arming implementation.

## Generate & Build micro ROS agent
```bash
source install/setup.bash  # setup.zsh if using zsh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```
