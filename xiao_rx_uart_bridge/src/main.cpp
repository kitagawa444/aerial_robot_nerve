#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#if (defined(UART_RX_ONLY_MODE) + defined(UART_BRIDGE_MODE) + defined(UART_DIAGNOSTIC_MODE) + defined(CRSF_HID_MODE)) > 1
#error "Select exactly one XIAO RX adapter mode"
#elif !defined(UART_RX_ONLY_MODE) && !defined(UART_BRIDGE_MODE) && !defined(UART_DIAGNOSTIC_MODE) && !defined(CRSF_HID_MODE)
#error "No XIAO RX adapter mode selected"
#endif

#if defined(UART_RX_ONLY_MODE) || defined(UART_BRIDGE_MODE)

#ifndef BRIDGE_UART_BAUD
#define BRIDGE_UART_BAUD 460800
#endif

namespace
{

constexpr size_t FORWARD_BUFFER_SIZE = 64U;

#ifdef UART_FOLLOW_USB_BAUD
uint32_t current_uart_baud = BRIDGE_UART_BAUD;

void followUsbBaudRate()
{
  cdc_line_coding_t line_coding{};
  tud_cdc_get_line_coding(&line_coding);

  const uint32_t requested_baud = line_coding.bit_rate;
  if (requested_baud < 9600U || requested_baud > 1000000U ||
      requested_baud == current_uart_baud)
  {
    return;
  }

  Serial1.flush();
  Serial1.end();
  Serial1.begin(requested_baud, SERIAL_8N1);
  current_uart_baud = requested_baud;
}
#endif

template<typename Input, typename Output>
void forwardAvailable(Input& input, Output& output)
{
  uint8_t buffer[FORWARD_BUFFER_SIZE];
  size_t count = 0U;

  while (count < sizeof(buffer) && input.available() > 0)
  {
    const int byte = input.read();
    if (byte < 0)
    {
      break;
    }
    buffer[count++] = static_cast<uint8_t>(byte);
  }

  if (count > 0U)
  {
    output.write(buffer, count);
  }
}

}  // namespace

void setup()
{
  Serial.begin(BRIDGE_UART_BAUD);
  Serial1.setTX(D6);
  Serial1.setRX(D7);
#ifdef UART_INVERT_RX
  Serial1.setInvertRX(true);
#endif
  Serial1.begin(BRIDGE_UART_BAUD, SERIAL_8N1);
}

void loop()
{
#ifdef UART_FOLLOW_USB_BAUD
  // ExpressLRS Configurator starts the CRSF reset at 420000 baud, then opens
  // the same COM port at the ESP bootloader upload baud. Mirror the USB CDC
  // line coding so both stages reach the receiver at the requested rate.
  followUsbBaudRate();
#endif

#ifdef UART_BRIDGE_MODE
  forwardAvailable(Serial, Serial1);
#else
  // Deliberately discard host-to-receiver bytes. This prevents FC/GCS MAVLink
  // telemetry payloads from reaching the ExpressLRS receiver UART.
  while (Serial.available() > 0)
  {
    Serial.read();
  }
#endif
  forwardAvailable(Serial1, Serial);
}

#endif  // UART_RX_ONLY_MODE || UART_BRIDGE_MODE

#ifdef UART_DIAGNOSTIC_MODE

namespace
{

uint32_t uart_byte_count = 0U;
uint32_t sampled_edge_count = 0U;
uint32_t last_report_ms = 0U;
int last_rx_level = HIGH;

}  // namespace

void setup()
{
  Serial.begin(115200);
  pinMode(D7, INPUT);
  last_rx_level = digitalRead(D7);
  Serial1.setTX(D6);
  Serial1.setRX(D7);
  Serial1.begin(BRIDGE_UART_BAUD, SERIAL_8N1);
}

void loop()
{
  const int rx_level = digitalRead(D7);
  if (rx_level != last_rx_level)
  {
    ++sampled_edge_count;
    last_rx_level = rx_level;
  }

  while (Serial1.available() > 0)
  {
    Serial1.read();
    ++uart_byte_count;
  }

  const uint32_t now = millis();
  if (now - last_report_ms >= 500U)
  {
    last_report_ms = now;
    Serial.print("rx_level=");
    Serial.print(last_rx_level);
    Serial.print(" sampled_edges=");
    Serial.print(sampled_edge_count);
    Serial.print(" uart_bytes=");
    Serial.println(uart_byte_count);
  }
}

#endif  // UART_DIAGNOSTIC_MODE

#ifdef CRSF_HID_MODE

namespace
{

constexpr uint32_t CRSF_BAUD_RATE = 420000U;
constexpr size_t CRSF_CHANNEL_COUNT = 16U;
constexpr size_t CRSF_RC_PAYLOAD_SIZE = 22U;
constexpr size_t CRSF_MAX_FRAME_SIZE = 64U;
constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8U;
constexpr uint8_t CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16U;
constexpr uint16_t CRSF_SWITCH_MIDPOINT = 0x03FFU;
constexpr uint32_t CRSF_FRAME_GAP_US = 300U;
constexpr uint32_t PASSTHROUGH_IDLE_TIMEOUT_MS = 2000U;

#define TUD_HID_REPORT_DESC_CRSF_GAMEPAD(...) \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
  HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD), \
  HID_COLLECTION(HID_COLLECTION_APPLICATION), \
  __VA_ARGS__ \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
  HID_USAGE(HID_USAGE_DESKTOP_X), \
  HID_USAGE(HID_USAGE_DESKTOP_Y), \
  HID_USAGE(HID_USAGE_DESKTOP_Z), \
  HID_USAGE(HID_USAGE_DESKTOP_RX), \
  HID_USAGE(HID_USAGE_DESKTOP_RY), \
  HID_USAGE(HID_USAGE_DESKTOP_RZ), \
  HID_USAGE(HID_USAGE_DESKTOP_SLIDER), \
  HID_USAGE(HID_USAGE_DESKTOP_DIAL), \
  HID_LOGICAL_MIN(0), \
  HID_LOGICAL_MAX_N(0x07FF, 2), \
  HID_REPORT_COUNT(8), \
  HID_REPORT_SIZE(16), \
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON), \
  HID_USAGE_MIN(1), \
  HID_USAGE_MAX(8), \
  HID_LOGICAL_MIN(0), \
  HID_LOGICAL_MAX(1), \
  HID_REPORT_COUNT(8), \
  HID_REPORT_SIZE(1), \
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_COLLECTION_END

struct __attribute__((packed)) GamepadReport
{
  uint16_t axes[8];
  uint8_t buttons;
};

const uint8_t hid_report_descriptor[] = {
  TUD_HID_REPORT_DESC_CRSF_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
uint8_t rx_buffer[CRSF_MAX_FRAME_SIZE + 2U]{};
size_t rx_position = 0U;
size_t expected_frame_size = 0U;
uint32_t last_byte_time_us = 0U;
GamepadReport gamepad{};
bool report_ready = false;

uint8_t crc8DvbS2(const uint8_t* data, size_t size)
{
  uint8_t crc = 0U;
  while (size-- > 0U)
  {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 0x80U) != 0U
        ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
        : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

void unpackChannels(const uint8_t* payload, uint16_t* channels)
{
  for (size_t channel = 0U; channel < CRSF_CHANNEL_COUNT; ++channel)
  {
    const size_t bit_offset = channel * 11U;
    const size_t byte_offset = bit_offset / 8U;
    const uint8_t shift = static_cast<uint8_t>(bit_offset % 8U);

    uint32_t packed = payload[byte_offset];
    if (byte_offset + 1U < CRSF_RC_PAYLOAD_SIZE)
    {
      packed |= static_cast<uint32_t>(payload[byte_offset + 1U]) << 8U;
    }
    if (byte_offset + 2U < CRSF_RC_PAYLOAD_SIZE)
    {
      packed |= static_cast<uint32_t>(payload[byte_offset + 2U]) << 16U;
    }
    channels[channel] = static_cast<uint16_t>((packed >> shift) & 0x07FFU);
  }
}

void decodeFrame()
{
  if (expected_frame_size != 26U ||
      rx_buffer[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER ||
      rx_buffer[2] != CRSF_FRAME_TYPE_RC_CHANNELS_PACKED)
  {
    return;
  }

  const size_t crc_index = expected_frame_size - 1U;
  if (crc8DvbS2(&rx_buffer[2], crc_index - 2U) != rx_buffer[crc_index])
  {
    return;
  }

  uint16_t channels[CRSF_CHANNEL_COUNT]{};
  unpackChannels(&rx_buffer[3], channels);

  gamepad.buttons = 0U;
  for (size_t i = 0U; i < 4U; ++i)
  {
    gamepad.axes[i] = channels[i];
  }
  for (size_t i = 0U; i < 4U; ++i)
  {
    gamepad.axes[i + 4U] = channels[i + 5U];
  }

  if (channels[4] > CRSF_SWITCH_MIDPOINT)
  {
    gamepad.buttons |= 0x01U;
  }
  for (size_t i = 9U; i < CRSF_CHANNEL_COUNT; ++i)
  {
    if (channels[i] > CRSF_SWITCH_MIDPOINT)
    {
      gamepad.buttons |= static_cast<uint8_t>(1U << (i - 8U));
    }
  }
  report_ready = true;
}

void receiveCrsf()
{
  while (Serial1.available() > 0)
  {
    const int byte = Serial1.read();
    if (byte < 0)
    {
      break;
    }

    last_byte_time_us = micros();
    if (rx_position < sizeof(rx_buffer))
    {
      rx_buffer[rx_position++] = static_cast<uint8_t>(byte);
    }
    else
    {
      rx_position = 0U;
      expected_frame_size = 0U;
      continue;
    }

    if (rx_position == 2U)
    {
      const uint8_t body_size = rx_buffer[1];
      expected_frame_size = static_cast<size_t>(body_size) + 2U;
      if (body_size < 2U || expected_frame_size > sizeof(rx_buffer))
      {
        rx_position = 0U;
        expected_frame_size = 0U;
      }
    }

    if (expected_frame_size > 0U && rx_position == expected_frame_size)
    {
      decodeFrame();
      rx_position = 0U;
      expected_frame_size = 0U;
    }
  }

  if (rx_position > 0U &&
      static_cast<uint32_t>(micros() - last_byte_time_us) > CRSF_FRAME_GAP_US)
  {
    rx_position = 0U;
    expected_frame_size = 0U;
  }
}

void runFirmwarePassthrough()
{
  if (Serial.available() <= 0)
  {
    return;
  }

  uint32_t last_activity_ms = millis();
  do
  {
    while (Serial.available() > 0)
    {
      Serial1.write(Serial.read());
      last_activity_ms = millis();
    }
    while (Serial1.available() > 0)
    {
      Serial.write(Serial1.read());
      last_activity_ms = millis();
    }
  }
  while (static_cast<uint32_t>(millis() - last_activity_ms) <
         PASSTHROUGH_IDLE_TIMEOUT_MS);
}

}  // namespace

void setup()
{
  usb_hid.setPollInterval(1U);
  usb_hid.setReportDescriptor(hid_report_descriptor, sizeof(hid_report_descriptor));
  usb_hid.begin();

  while (!TinyUSBDevice.mounted())
  {
    delay(1U);
  }

  Serial.begin(CRSF_BAUD_RATE);
  Serial1.begin(CRSF_BAUD_RATE, SERIAL_8N1);
}

void loop()
{
  receiveCrsf();
  runFirmwarePassthrough();

  if (report_ready && usb_hid.ready())
  {
    usb_hid.sendReport(0U, &gamepad, sizeof(gamepad));
    report_ready = false;
  }
}

#endif  // CRSF_HID_MODE
