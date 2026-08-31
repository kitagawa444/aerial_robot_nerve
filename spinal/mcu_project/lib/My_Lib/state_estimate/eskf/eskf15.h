#pragma once

#include <cstddef>
#include <cstdint>

#include "math/AP_Math.h"

class Eskf15
{
public:
  static constexpr size_t STATE_SIZE = 15;

  struct Noise
  {
    float accelerometer{ 0.35f };
    float gyroscope{ 0.02f };
    float accelerometer_bias{ 0.005f };
    float gyroscope_bias{ 0.0005f };
  };

  struct State
  {
    ap::Vector3f position{};
    ap::Vector3f velocity{};
    ap::Quaternion attitude{};
    ap::Vector3f accelerometer_bias{};
    ap::Vector3f gyroscope_bias{};
  };

  enum Validity : uint8_t
  {
    ATTITUDE_VALID = 1U << 0,
    VERTICAL_POSITION_VALID = 1U << 1,
    HORIZONTAL_POSITION_VALID = 1U << 2,
    VELOCITY_VALID = 1U << 3
  };

  Eskf15();

  void reset();
  void setNoise(const Noise &noise) { noise_ = noise; }
  const Noise &noise() const { return noise_; }

  bool predict(const ap::Vector3f &accelerometer, const ap::Vector3f &gyroscope, float dt);
  bool resetPosition(const ap::Vector3f &position, const ap::Vector3f &variance);
  bool resetVelocity(const ap::Vector3f &velocity, const ap::Vector3f &variance);
  bool fusePosition(const ap::Vector3f &position, const ap::Vector3f &variance, float gate_sigma = 5.0f);
  bool fuseVelocity(const ap::Vector3f &velocity, const ap::Vector3f &variance, float gate_sigma = 5.0f);
  bool fuseAltitude(float altitude, float variance, float gate_sigma = 5.0f);
  bool fuseAttitude(const ap::Quaternion &attitude, const ap::Vector3f &variance, float gate_sigma = 5.0f);

  const State &state() const { return state_; }
  const float *covariance() const { return covariance_; }
  float covariance(size_t row, size_t column) const;
  uint8_t validity() const { return validity_; }
  bool valid(Validity flag) const { return (validity_ & flag) != 0U; }
  bool initialized() const { return initialized_; }
  uint32_t rejectedMeasurementCount() const { return rejected_measurement_count_; }

private:
  static constexpr size_t POSITION_INDEX = 0;
  static constexpr size_t VELOCITY_INDEX = 3;
  static constexpr size_t ATTITUDE_INDEX = 6;
  static constexpr size_t ACCELEROMETER_BIAS_INDEX = 9;
  static constexpr size_t GYROSCOPE_BIAS_INDEX = 12;

  State state_{};
  Noise noise_{};
  float covariance_[STATE_SIZE * STATE_SIZE]{};
  uint8_t validity_{ 0U };
  bool initialized_{ false };
  uint32_t rejected_measurement_count_{ 0U };

  static bool finiteVector_(const ap::Vector3f &value);
  static float positiveVariance_(float variance);
  static void skew_(const ap::Vector3f &value, float output[3][3]);

  void initializeCovariance_();
  void propagateCovariance_(const ap::Matrix3f &rotation, const ap::Vector3f &specific_force,
                            const ap::Vector3f &angular_rate, float dt);
  bool scalarUpdate_(const float measurement_jacobian[STATE_SIZE], float innovation, float variance, float gate_sigma);
  void injectError_(const float correction[STATE_SIZE]);
  void resetCovarianceBlock_(size_t state_index, const ap::Vector3f &variance);
  void symmetrizeCovariance_();
};
