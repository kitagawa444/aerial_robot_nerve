#include "state_estimate/eskf/eskf15.h"

#include <cmath>
#include <cstring>

namespace
{
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float MIN_VARIANCE = 1.0e-8f;
constexpr float MAX_DT = 0.05f;
}

Eskf15::Eskf15() { reset(); }

void Eskf15::reset()
{
  state_ = State{};
  state_.attitude.initialise();
  validity_ = 0U;
  initialized_ = false;
  rejected_measurement_count_ = 0U;
  initializeCovariance_();
}

float Eskf15::covariance(size_t row, size_t column) const
{
  if (row >= STATE_SIZE || column >= STATE_SIZE) return 0.0f;
  return covariance_[row * STATE_SIZE + column];
}

bool Eskf15::predict(const ap::Vector3f &accelerometer, const ap::Vector3f &gyroscope, float dt)
{
  if (!finiteVector_(accelerometer) || !finiteVector_(gyroscope) || !std::isfinite(dt) || dt <= 0.0f || dt > MAX_DT)
  {
    return false;
  }

  const ap::Vector3f specific_force = accelerometer - state_.accelerometer_bias;
  const ap::Vector3f angular_rate = gyroscope - state_.gyroscope_bias;

  ap::Matrix3f rotation;
  state_.attitude.rotation_matrix(rotation);
  const ap::Vector3f acceleration_world = rotation * specific_force - ap::Vector3f(0.0f, 0.0f, GRAVITY_MPS2);

  state_.position += state_.velocity * dt + acceleration_world * (0.5f * dt * dt);
  state_.velocity += acceleration_world * dt;

  ap::Quaternion delta_attitude;
  delta_attitude.from_axis_angle(angular_rate * dt);
  state_.attitude *= delta_attitude;
  state_.attitude.normalize();

  propagateCovariance_(rotation, specific_force, angular_rate, dt);
  initialized_ = true;
  return true;
}

bool Eskf15::resetPosition(const ap::Vector3f &position, const ap::Vector3f &variance)
{
  if (!finiteVector_(position) || !finiteVector_(variance)) return false;

  state_.position = position;
  resetCovarianceBlock_(POSITION_INDEX, variance);
  validity_ |= HORIZONTAL_POSITION_VALID | VERTICAL_POSITION_VALID;
  initialized_ = true;
  return true;
}

bool Eskf15::resetVelocity(const ap::Vector3f &velocity, const ap::Vector3f &variance)
{
  if (!finiteVector_(velocity) || !finiteVector_(variance)) return false;

  state_.velocity = velocity;
  resetCovarianceBlock_(VELOCITY_INDEX, variance);
  validity_ |= VELOCITY_VALID;
  initialized_ = true;
  return true;
}

bool Eskf15::fusePosition(const ap::Vector3f &position, const ap::Vector3f &variance, float gate_sigma)
{
  if (!finiteVector_(position) || !finiteVector_(variance)) return false;

  bool fused = true;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    float jacobian[STATE_SIZE]{};
    jacobian[POSITION_INDEX + axis] = 1.0f;
    fused = scalarUpdate_(jacobian, position[axis] - state_.position[axis], positiveVariance_(variance[axis]),
                          gate_sigma) &&
            fused;
  }

  if (fused)
  {
    validity_ |= HORIZONTAL_POSITION_VALID | VERTICAL_POSITION_VALID;
  }
  return fused;
}

bool Eskf15::fuseVelocity(const ap::Vector3f &velocity, const ap::Vector3f &variance, float gate_sigma)
{
  if (!finiteVector_(velocity) || !finiteVector_(variance)) return false;

  bool fused = true;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    float jacobian[STATE_SIZE]{};
    jacobian[VELOCITY_INDEX + axis] = 1.0f;
    fused = scalarUpdate_(jacobian, velocity[axis] - state_.velocity[axis], positiveVariance_(variance[axis]),
                          gate_sigma) &&
            fused;
  }

  if (fused) validity_ |= VELOCITY_VALID;
  return fused;
}

bool Eskf15::fuseAltitude(float altitude, float variance, float gate_sigma)
{
  if (!std::isfinite(altitude) || !std::isfinite(variance)) return false;

  float jacobian[STATE_SIZE]{};
  jacobian[POSITION_INDEX + 2] = 1.0f;
  const bool fused = scalarUpdate_(jacobian, altitude - state_.position.z, positiveVariance_(variance), gate_sigma);
  if (fused) validity_ |= VERTICAL_POSITION_VALID;
  return fused;
}

bool Eskf15::fuseAttitude(const ap::Quaternion &attitude, const ap::Vector3f &variance, float gate_sigma)
{
  if (attitude.is_nan() || !finiteVector_(variance)) return false;

  ap::Quaternion measurement = attitude;
  measurement.normalize();
  ap::Quaternion error = state_.attitude.inverse() * measurement;
  error.normalize();
  if (error.q1 < 0.0f)
  {
    error.q1 = -error.q1;
    error.q2 = -error.q2;
    error.q3 = -error.q3;
    error.q4 = -error.q4;
  }

  const ap::Vector3f innovation(2.0f * error.q2, 2.0f * error.q3, 2.0f * error.q4);
  bool fused = true;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    float jacobian[STATE_SIZE]{};
    jacobian[ATTITUDE_INDEX + axis] = 1.0f;
    fused = scalarUpdate_(jacobian, innovation[axis], positiveVariance_(variance[axis]), gate_sigma) && fused;
  }

  if (fused) validity_ |= ATTITUDE_VALID;
  return fused;
}

bool Eskf15::finiteVector_(const ap::Vector3f &value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

float Eskf15::positiveVariance_(float variance) { return variance > MIN_VARIANCE ? variance : MIN_VARIANCE; }

void Eskf15::skew_(const ap::Vector3f &value, float output[3][3])
{
  output[0][0] = 0.0f;
  output[0][1] = -value.z;
  output[0][2] = value.y;
  output[1][0] = value.z;
  output[1][1] = 0.0f;
  output[1][2] = -value.x;
  output[2][0] = -value.y;
  output[2][1] = value.x;
  output[2][2] = 0.0f;
}

void Eskf15::initializeCovariance_()
{
  std::memset(covariance_, 0, sizeof(covariance_));
  for (size_t axis = 0; axis < 3; ++axis)
  {
    covariance_[(POSITION_INDEX + axis) * STATE_SIZE + POSITION_INDEX + axis] = 10.0f;
    covariance_[(VELOCITY_INDEX + axis) * STATE_SIZE + VELOCITY_INDEX + axis] = 1.0f;
    covariance_[(ATTITUDE_INDEX + axis) * STATE_SIZE + ATTITUDE_INDEX + axis] = 0.25f;
    covariance_[(ACCELEROMETER_BIAS_INDEX + axis) * STATE_SIZE + ACCELEROMETER_BIAS_INDEX + axis] = 0.04f;
    covariance_[(GYROSCOPE_BIAS_INDEX + axis) * STATE_SIZE + GYROSCOPE_BIAS_INDEX + axis] = 0.01f;
  }
}

void Eskf15::propagateCovariance_(const ap::Matrix3f &rotation, const ap::Vector3f &specific_force,
                                  const ap::Vector3f &angular_rate, float dt)
{
  float transition[STATE_SIZE * STATE_SIZE]{};
  for (size_t index = 0; index < STATE_SIZE; ++index) transition[index * STATE_SIZE + index] = 1.0f;

  for (size_t axis = 0; axis < 3; ++axis)
  {
    transition[(POSITION_INDEX + axis) * STATE_SIZE + VELOCITY_INDEX + axis] = dt;
  }

  float force_skew[3][3]{};
  float rate_skew[3][3]{};
  skew_(specific_force, force_skew);
  skew_(angular_rate, rate_skew);

  for (size_t row = 0; row < 3; ++row)
  {
    for (size_t column = 0; column < 3; ++column)
    {
      float rotation_force_skew = 0.0f;
      for (size_t inner = 0; inner < 3; ++inner)
      {
        rotation_force_skew += rotation[row][inner] * force_skew[inner][column];
      }
      transition[(VELOCITY_INDEX + row) * STATE_SIZE + ATTITUDE_INDEX + column] = -rotation_force_skew * dt;
      transition[(VELOCITY_INDEX + row) * STATE_SIZE + ACCELEROMETER_BIAS_INDEX + column] = -rotation[row][column] * dt;
      transition[(ATTITUDE_INDEX + row) * STATE_SIZE + ATTITUDE_INDEX + column] += -rate_skew[row][column] * dt;
      if (row == column)
      {
        transition[(ATTITUDE_INDEX + row) * STATE_SIZE + GYROSCOPE_BIAS_INDEX + column] = -dt;
      }
    }
  }

  float intermediate[STATE_SIZE * STATE_SIZE]{};
  float propagated[STATE_SIZE * STATE_SIZE]{};
  for (size_t row = 0; row < STATE_SIZE; ++row)
  {
    for (size_t column = 0; column < STATE_SIZE; ++column)
    {
      for (size_t inner = 0; inner < STATE_SIZE; ++inner)
      {
        intermediate[row * STATE_SIZE + column] += transition[row * STATE_SIZE + inner] *
                                                   covariance_[inner * STATE_SIZE + column];
      }
    }
  }

  for (size_t row = 0; row < STATE_SIZE; ++row)
  {
    for (size_t column = 0; column < STATE_SIZE; ++column)
    {
      for (size_t inner = 0; inner < STATE_SIZE; ++inner)
      {
        propagated[row * STATE_SIZE + column] += intermediate[row * STATE_SIZE + inner] *
                                                 transition[column * STATE_SIZE + inner];
      }
    }
  }

  std::memcpy(covariance_, propagated, sizeof(covariance_));
  const float accelerometer_noise = noise_.accelerometer * noise_.accelerometer * dt;
  const float gyroscope_noise = noise_.gyroscope * noise_.gyroscope * dt;
  const float accelerometer_bias_noise = noise_.accelerometer_bias * noise_.accelerometer_bias * dt;
  const float gyroscope_bias_noise = noise_.gyroscope_bias * noise_.gyroscope_bias * dt;
  for (size_t axis = 0; axis < 3; ++axis)
  {
    covariance_[(VELOCITY_INDEX + axis) * STATE_SIZE + VELOCITY_INDEX + axis] += accelerometer_noise;
    covariance_[(ATTITUDE_INDEX + axis) * STATE_SIZE + ATTITUDE_INDEX + axis] += gyroscope_noise;
    covariance_[(ACCELEROMETER_BIAS_INDEX + axis) * STATE_SIZE + ACCELEROMETER_BIAS_INDEX + axis] +=
        accelerometer_bias_noise;
    covariance_[(GYROSCOPE_BIAS_INDEX + axis) * STATE_SIZE + GYROSCOPE_BIAS_INDEX + axis] += gyroscope_bias_noise;
  }
  symmetrizeCovariance_();
}

bool Eskf15::scalarUpdate_(const float measurement_jacobian[STATE_SIZE], float innovation, float variance,
                           float gate_sigma)
{
  float covariance_times_jacobian[STATE_SIZE]{};
  for (size_t row = 0; row < STATE_SIZE; ++row)
  {
    for (size_t column = 0; column < STATE_SIZE; ++column)
    {
      covariance_times_jacobian[row] += covariance_[row * STATE_SIZE + column] * measurement_jacobian[column];
    }
  }

  float innovation_variance = variance;
  for (size_t index = 0; index < STATE_SIZE; ++index)
  {
    innovation_variance += measurement_jacobian[index] * covariance_times_jacobian[index];
  }
  if (!std::isfinite(innovation_variance) || innovation_variance <= MIN_VARIANCE) return false;

  const float gate = gate_sigma > 0.0f ? gate_sigma : 1.0f;
  if (innovation * innovation > gate * gate * innovation_variance)
  {
    ++rejected_measurement_count_;
    return false;
  }

  float correction[STATE_SIZE]{};
  for (size_t index = 0; index < STATE_SIZE; ++index)
  {
    correction[index] = covariance_times_jacobian[index] * innovation / innovation_variance;
  }

  float jacobian_times_covariance[STATE_SIZE]{};
  for (size_t column = 0; column < STATE_SIZE; ++column)
  {
    for (size_t row = 0; row < STATE_SIZE; ++row)
    {
      jacobian_times_covariance[column] += measurement_jacobian[row] * covariance_[row * STATE_SIZE + column];
    }
  }

  for (size_t row = 0; row < STATE_SIZE; ++row)
  {
    const float kalman_gain = covariance_times_jacobian[row] / innovation_variance;
    for (size_t column = 0; column < STATE_SIZE; ++column)
    {
      covariance_[row * STATE_SIZE + column] -= kalman_gain * jacobian_times_covariance[column];
    }
  }

  injectError_(correction);
  symmetrizeCovariance_();
  return true;
}

void Eskf15::injectError_(const float correction[STATE_SIZE])
{
  state_.position += ap::Vector3f(correction[POSITION_INDEX], correction[POSITION_INDEX + 1],
                                  correction[POSITION_INDEX + 2]);
  state_.velocity += ap::Vector3f(correction[VELOCITY_INDEX], correction[VELOCITY_INDEX + 1],
                                  correction[VELOCITY_INDEX + 2]);

  ap::Quaternion attitude_correction;
  attitude_correction.from_axis_angle(
      ap::Vector3f(correction[ATTITUDE_INDEX], correction[ATTITUDE_INDEX + 1], correction[ATTITUDE_INDEX + 2]));
  state_.attitude *= attitude_correction;
  state_.attitude.normalize();

  state_.accelerometer_bias += ap::Vector3f(correction[ACCELEROMETER_BIAS_INDEX],
                                            correction[ACCELEROMETER_BIAS_INDEX + 1],
                                            correction[ACCELEROMETER_BIAS_INDEX + 2]);
  state_.gyroscope_bias += ap::Vector3f(correction[GYROSCOPE_BIAS_INDEX], correction[GYROSCOPE_BIAS_INDEX + 1],
                                        correction[GYROSCOPE_BIAS_INDEX + 2]);
}

void Eskf15::resetCovarianceBlock_(size_t state_index, const ap::Vector3f &variance)
{
  for (size_t axis = 0; axis < 3; ++axis)
  {
    const size_t index = state_index + axis;
    for (size_t other = 0; other < STATE_SIZE; ++other)
    {
      covariance_[index * STATE_SIZE + other] = 0.0f;
      covariance_[other * STATE_SIZE + index] = 0.0f;
    }
    covariance_[index * STATE_SIZE + index] = positiveVariance_(variance[axis]);
  }
}

void Eskf15::symmetrizeCovariance_()
{
  for (size_t row = 0; row < STATE_SIZE; ++row)
  {
    for (size_t column = row + 1; column < STATE_SIZE; ++column)
    {
      const float value = 0.5f * (covariance_[row * STATE_SIZE + column] + covariance_[column * STATE_SIZE + row]);
      covariance_[row * STATE_SIZE + column] = value;
      covariance_[column * STATE_SIZE + row] = value;
    }
    float &diagonal = covariance_[row * STATE_SIZE + row];
    if (!std::isfinite(diagonal) || diagonal < MIN_VARIANCE) diagonal = MIN_VARIANCE;
  }
}
