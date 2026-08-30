#include "rc/crsf_teleop.h"

#include <array>
#include <gtest/gtest.h>

namespace
{

constexpr uint16_t LOW = 172U;
constexpr uint16_t HIGH = 1811U;

std::array<uint16_t, crsf::CHANNEL_COUNT> released_channels()
{
  std::array<uint16_t, crsf::CHANNEL_COUNT> channels{};
  channels.fill(LOW);
  return channels;
}

TEST(CrsfTeleop, EmitsArmOnlyOnRisingEdgeAfterRelease)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();

  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 0U).arm);
  channels[4] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 1U).arm);
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 2U).arm);

  channels[4] = LOW;
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 3U).arm);
  channels[4] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 4U).arm);
}

TEST(CrsfTeleop, RequiresReleaseAfterLinkEstablishment)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();
  channels[4] = HIGH;

  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 0U).arm);
  channels[4] = LOW;
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 1U).arm);
  channels[4] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 2U).arm);
}

TEST(CrsfTeleop, TakeoffRequiresModifierAndAction)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();
  (void)interpreter.update(channels.data(), channels.size(), true, 0U);

  channels[5] = HIGH;
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 1U).takeoff);
  channels[6] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 2U).takeoff);
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 3U).takeoff);
}

TEST(CrsfTeleop, DisconnectResetsEdgeArming)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();
  (void)interpreter.update(channels.data(), channels.size(), true, 0U);
  channels[4] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 1U).arm);

  EXPECT_FALSE(interpreter.update(nullptr, 0U, false, 2U).arm);
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 3U).arm);
}

TEST(CrsfTeleop, LandingRequiresRightAndSquare)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();
  (void)interpreter.update(channels.data(), channels.size(), true, 0U);

  channels[8] = HIGH;
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 1U).land);
  channels[9] = HIGH;
  EXPECT_TRUE(interpreter.update(channels.data(), channels.size(), true, 2U).land);
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 3U).land);
}

TEST(CrsfTeleop, StopForcesLandingThenHaltsAfterHold)
{
  crsf::TeleopInterpreter interpreter;
  auto channels = released_channels();
  (void)interpreter.update(channels.data(), channels.size(), true, 0U);

  channels[7] = HIGH;
  auto events = interpreter.update(channels.data(), channels.size(), true, 10U);
  EXPECT_TRUE(events.force_landing);
  EXPECT_FALSE(events.halt);

  events = interpreter.update(channels.data(), channels.size(), true, 1009U);
  EXPECT_FALSE(events.halt);
  events = interpreter.update(channels.data(), channels.size(), true, 1010U);
  EXPECT_TRUE(events.halt);
  EXPECT_FALSE(interpreter.update(channels.data(), channels.size(), true, 1200U).halt);
}

}  // namespace
