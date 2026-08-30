# XIAO RX UART bridge

This PlatformIO project turns a Seeed Studio XIAO RP2040 into a bench adapter
for inspecting the UART of an ExpressLRS receiver from a PC. It is intentionally
separate from `spinal`: production firmware connects the receiver directly to
an STM32 UART, while this adapter is for protocol bring-up and radio-in-the-loop
simulation.

## Wiring

For normal receive-only operation, only connect the receiver TX line:

| ExpressLRS receiver | XIAO RP2040 |
| --- | --- |
| TX | D7 / Serial1 RX |
| GND | GND |

Connect receiver RX to XIAO D6 / Serial1 TX only while using the wired
configuration bridge described below.

Power the receiver with the voltage required by its manufacturer and share the
ground with the XIAO. Do not connect a 5 V supply to either UART signal pin.
Keep the transmitter antenna attached whenever its RF section is powered.

## Receive-only CRSF adapter

The default environment forwards receiver bytes to the PC without parsing them:

```text
PC USB CDC <- XIAO Serial1 <- ExpressLRS receiver UART
```

Bytes written by the PC are discarded and never sent to the receiver. This
keeps FC/PC telemetry payloads out of the receiver UART. It uses the standard
420000-baud CRSF receiver output used by ExpressLRS Normal mode. Build and
upload it with:

```bash
pio run -e crsf_rx_only
pio run -e crsf_rx_only -t upload
```

Use the stable device entry under `/dev/serial/by-id/` when passing the USB CDC
port to the simulation receiver. The bridge deliberately leaves CRSF framing,
CRC validation, channel decoding, and failsafe handling to the same parser used
by the flight-controller path.

This only disables the UART payload path toward the receiver. Configure the
ExpressLRS TX for Normal mode with `Telem Ratio: Off`, enable the receiver's
`Force telemetry off` option, and disable automatic receiver Wi-Fi separately.
The XIAO firmware alone cannot disable the receiver's RF transmitter.

The old receive-only native MAVLink bridge remains available for non-RF bench
work as `mavlink_rx_only`, but it must not be used with a non-certified receiver
because ExpressLRS native MAVLink mode forces a 1:2 telemetry ratio.

## Wired configuration bridge

For ExpressLRS Configurator receiver flashing, use the bidirectional bridge
that starts at the CRSF rate and follows changes to the USB CDC baud rate. This
allows the reset-to-bootloader stage at 420000 baud and the subsequent ESP
upload stage to use the same COM port:

```bash
pio run -e elrs_config_bridge
pio run -e elrs_config_bridge -t upload
```

Connect both crossed UART lines only for this operation:

| ExpressLRS receiver | XIAO RP2040 |
| --- | --- |
| TX | D7 / Serial1 RX |
| RX | D6 / Serial1 TX |
| GND | GND |

The older fixed-rate bidirectional environment remains available for tools
that explicitly require 460800 baud:

```bash
pio run -e uart_config_bridge
pio run -e uart_config_bridge -t upload
```

Restore `crsf_rx_only` and disconnect receiver RX from XIAO D6 before running
the receive-only test.

## Legacy CRSF HID mode

The `crsf_hid` environment retains the original receiver test workflow. It
decodes CRSF RC channel frames at 420000 baud and exposes eight axes and eight
buttons as a USB HID gamepad:

```bash
pio run -e crsf_hid
pio run -e crsf_hid -t upload
```

The first four HID axes correspond to CRSF channels 1 through 4. The remaining
axes correspond to channels 6 through 9. Channels 5 and 10 through 16 are
converted into eight buttons using the CRSF midpoint.

The CDC port remains available in this mode. Sending data to it temporarily
switches the firmware into a bidirectional UART passthrough, which can be used
with the ExpressLRS Configurator's Betaflight passthrough flashing method.
