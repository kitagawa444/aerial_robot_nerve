/*
******************************************************************************
* File Name          : state_estimate.h
* Description        : state(attitude, altitude, pos) estimate core (NO ROS)
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __STATE_ESTIMATE_H
#define __STATE_ESTIMATE_H

#ifndef SIMULATION
#include "config.h"
#include "sensors/baro/baro_ms5611.h"
#include "sensors/gps/gps_ublox.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#else

#endif

#include <cmath>

#include "state_estimate/attitude/attitude_estimate.h"
#include "state_estimate/eskf/eskf15.h"

class StateEstimate
{
public:
  StateEstimate() = default;
  ~StateEstimate() = default;

#ifdef SIMULATION
  void init()
  {
    attitude_estimate_flag_ = true;
    attitude_estimator_.init();
    eskf_.reset();
    last_imu_update_time_ms_ = 0U;
  }
#else
  void init(IMU *imu, Baro *baro, GPS *gps)
  {
    imu_ = imu;
    baro_ = baro;
    gps_ = gps;

    if (imu_ == nullptr)
    {
      attitude_estimate_flag_ = false;
    }
    else
    {
      attitude_estimate_flag_ = true;
      attitude_estimator_.init(imu_, gps_);
    }

    altitude_estimate_flag_ = (baro_ != nullptr);
    pos_estimate_flag_ = (gps_ != nullptr);

    eskf_.reset();
    last_imu_update_time_ms_ = HAL_GetTick();
  }
#endif  // SIMULATION

#ifdef SIMULATION
  void update(uint32_t now_ms)
  {
    attitude_estimator_.update();
    updateEskf_(now_ms, attitude_estimator_.getAccVec(), attitude_estimator_.getGyroVec());
    updateDirectStateFreshness_();
  }
#else
  void update()
  {
    if (attitude_estimate_flag_ && imu_ != nullptr && imu_->getUpdate())
    {
      const ap::Vector3f accelerometer = imu_->getAcc();
      const ap::Vector3f gyroscope = imu_->getGyro();
      attitude_estimator_.update();
      updateEskf_(HAL_GetTick(), accelerometer, gyroscope);
      updateDirectStateFreshness_();
    }

    if (altitude_estimate_flag_ && baro_ != nullptr && baro_->getUpdate())
    {
      (void)eskf_.fuseAltitude(baro_->getAltitude(), barometer_variance_);
      baro_->setUpdate(false);
    }
  }
#endif

  AttitudeEstimate *getAttEstimator() { return &attitude_estimator_; }
  Eskf15 *getEskf() { return &eskf_; }
  const Eskf15 *getEskf() const { return &eskf_; }

  void setDirectStateEnabled(bool enabled)
  {
    direct_state_enabled_ = enabled;
    attitude_estimator_.useGroundTruth(enabled);
  }

  bool directStateEnabled() const { return direct_state_enabled_; }

  bool applyDirectExternalState(const ap::Vector3f &position, const ap::Vector3f &position_variance,
                                const ap::Vector3f &velocity, const ap::Vector3f &velocity_variance,
                                const ap::Quaternion &attitude, const ap::Vector3f &attitude_variance,
                                const ap::Vector3f &angular_velocity, uint8_t field_mask, uint32_t now_ms)
  {
    static constexpr uint8_t POSITION = 1U;
    static constexpr uint8_t VELOCITY = 2U;
    static constexpr uint8_t ATTITUDE = 4U;

    if ((field_mask & (POSITION | VELOCITY | ATTITUDE)) == 0U) return false;
    if ((field_mask & POSITION) != 0U && (!finiteVector_(position) || !finiteVector_(position_variance))) return false;
    if ((field_mask & VELOCITY) != 0U && (!finiteVector_(velocity) || !finiteVector_(velocity_variance))) return false;
    if ((field_mask & ATTITUDE) != 0U &&
        (!finiteQuaternion_(attitude) || !finiteVector_(attitude_variance) || !finiteVector_(angular_velocity)))
    {
      return false;
    }

    if ((field_mask & POSITION) != 0U)
    {
      direct_state_.position = position;
      for (size_t axis = 0; axis < 3; ++axis) direct_covariance_[axis] = positiveVariance_(position_variance[axis]);
      direct_validity_ |= Eskf15::HORIZONTAL_POSITION_VALID | Eskf15::VERTICAL_POSITION_VALID;
    }

    if ((field_mask & VELOCITY) != 0U)
    {
      if (last_direct_velocity_time_ms_ != 0U)
      {
        const uint32_t elapsed_ms = now_ms - last_direct_velocity_time_ms_;
        if (elapsed_ms > 0U && elapsed_ms <= 100U)
        {
          direct_linear_acceleration_world_ = (velocity - direct_state_.velocity) /
                                              (static_cast<float>(elapsed_ms) * 0.001f);
        }
      }
      direct_state_.velocity = velocity;
      last_direct_velocity_time_ms_ = now_ms;
      for (size_t axis = 0; axis < 3; ++axis)
      {
        direct_covariance_[3U + axis] = positiveVariance_(velocity_variance[axis]);
      }
      direct_validity_ |= Eskf15::VELOCITY_VALID;
    }

    if ((field_mask & ATTITUDE) != 0U)
    {
      direct_state_.attitude = attitude;
      direct_state_.attitude.normalize();
      direct_angular_rate_ = angular_velocity;
      for (size_t axis = 0; axis < 3; ++axis)
      {
        direct_covariance_[6U + axis] = positiveVariance_(attitude_variance[axis]);
      }
      ap::Matrix3f rotation;
      direct_state_.attitude.rotation_matrix(rotation);
      attitude_estimator_.setGroundTruthStates(rotation, angular_velocity);
      direct_validity_ |= Eskf15::ATTITUDE_VALID;
    }

    last_direct_state_time_ms_ = now_ms;
    setDirectStateEnabled(true);
    return true;
  }

  const Eskf15::State &outputState() const { return direct_state_enabled_ ? direct_state_ : eskf_.state(); }

  bool outputInitialized() const { return direct_state_enabled_ ? direct_validity_ != 0U : eskf_.initialized(); }

  float outputCovariance(size_t index) const
  {
    if (index >= Eskf15::STATE_SIZE) return 0.0f;
    return direct_state_enabled_ ? direct_covariance_[index] : eskf_.covariance(index, index);
  }

  uint32_t outputRejectedMeasurementCount() const
  {
    return direct_state_enabled_ ? 0U : eskf_.rejectedMeasurementCount();
  }

  bool fuseExternalPosition(const ap::Vector3f &position, const ap::Vector3f &variance)
  {
    const bool reacquire = externalMeasurementExpired_(last_external_position_time_ms_);
    const bool fused = reacquire ? eskf_.resetPosition(position, variance) : eskf_.fusePosition(position, variance);
    if (fused) last_external_position_time_ms_ = last_imu_update_time_ms_;
    return fused;
  }

  bool fuseExternalVelocity(const ap::Vector3f &velocity, const ap::Vector3f &variance)
  {
    const bool reacquire = externalMeasurementExpired_(last_external_velocity_time_ms_);
    const bool fused = reacquire ? eskf_.resetVelocity(velocity, variance) : eskf_.fuseVelocity(velocity, variance);
    if (fused) last_external_velocity_time_ms_ = last_imu_update_time_ms_;
    return fused;
  }

  bool fuseExternalAttitude(const ap::Quaternion &attitude, const ap::Vector3f &variance)
  {
    return eskf_.fuseAttitude(attitude, variance);
  }

  bool fuseAltitude(float altitude, float variance) { return eskf_.fuseAltitude(altitude, variance); }

  ap::Vector3f getLinearAccelerationWorld() const
  {
    return direct_state_enabled_ ? direct_linear_acceleration_world_ : linear_acceleration_world_;
  }

  ap::Vector3f getAngularRate() const
  {
    return direct_state_enabled_ ? direct_angular_rate_ :
                                   attitude_estimator_.getGyroVec() - eskf_.state().gyroscope_bias;
  }

  uint8_t stateValidity(uint32_t external_timeout_ms = 1000U) const
  {
    if (direct_state_enabled_)
    {
      if (directMeasurementExpired_(external_timeout_ms)) return 0U;
      return direct_validity_;
    }

    uint8_t validity = eskf_.validity();
    if (externalMeasurementExpired_(last_external_position_time_ms_, external_timeout_ms))
    {
      validity &= static_cast<uint8_t>(~(Eskf15::HORIZONTAL_POSITION_VALID | Eskf15::VERTICAL_POSITION_VALID));
    }
    if (externalMeasurementExpired_(last_external_velocity_time_ms_, external_timeout_ms))
    {
      validity &= static_cast<uint8_t>(~Eskf15::VELOCITY_VALID);
    }
    return validity;
  }

  bool positionControlStateValid(uint32_t external_timeout_ms = 1000U) const
  {
    const uint8_t validity = stateValidity(external_timeout_ms);
    return (validity & (Eskf15::ATTITUDE_VALID | Eskf15::HORIZONTAL_POSITION_VALID | Eskf15::VERTICAL_POSITION_VALID |
                        Eskf15::VELOCITY_VALID)) == (Eskf15::ATTITUDE_VALID | Eskf15::HORIZONTAL_POSITION_VALID |
                                                     Eskf15::VERTICAL_POSITION_VALID | Eskf15::VELOCITY_VALID);
  }

  void setBarometerVariance(float variance)
  {
    if (variance > 0.0f) barometer_variance_ = variance;
  }

#ifndef SIMULATION
  IMU *getImu() { return imu_; }
  Baro *getBaro() { return baro_; }
  GPS *getGPS() { return gps_; }
#endif

  bool attitudeEnabled() const { return attitude_estimate_flag_; }
  bool altitudeEnabled() const { return altitude_estimate_flag_; }
  bool posEnabled() const { return pos_estimate_flag_; }

private:
  static bool finiteVector_(const ap::Vector3f &value)
  {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
  }

  static bool finiteQuaternion_(const ap::Quaternion &value)
  {
    if (!std::isfinite(value.q1) || !std::isfinite(value.q2) || !std::isfinite(value.q3) || !std::isfinite(value.q4))
    {
      return false;
    }
    const float norm_squared = value.q1 * value.q1 + value.q2 * value.q2 + value.q3 * value.q3 + value.q4 * value.q4;
    return norm_squared > 1.0e-6f;
  }

  static float positiveVariance_(float variance) { return variance > 1.0e-6f ? variance : 1.0e-6f; }

  bool directMeasurementExpired_(uint32_t timeout_ms) const
  {
    if (last_direct_state_time_ms_ == 0U) return true;
    const int32_t elapsed_ms = static_cast<int32_t>(last_imu_update_time_ms_ - last_direct_state_time_ms_);
    return elapsed_ms > static_cast<int32_t>(timeout_ms);
  }

  void updateDirectStateFreshness_()
  {
    // Position control will reject the stale state.  Use the onboard AHRS for
    // attitude stabilization during the resulting force landing instead of
    // holding the last external attitude indefinitely.
    if (direct_state_enabled_ && directMeasurementExpired_(1000U)) attitude_estimator_.useGroundTruth(false);
  }

  bool externalMeasurementExpired_(uint32_t measurement_time_ms, uint32_t timeout_ms = 1000U) const
  {
    if (measurement_time_ms == 0U) return true;
    const int32_t elapsed_ms = static_cast<int32_t>(last_imu_update_time_ms_ - measurement_time_ms);
    return elapsed_ms > static_cast<int32_t>(timeout_ms);
  }

  void updateEskf_(uint32_t now_ms, const ap::Vector3f &accelerometer, const ap::Vector3f &gyroscope)
  {
    if (last_imu_update_time_ms_ == 0U)
    {
      last_imu_update_time_ms_ = now_ms;
      return;
    }

    const uint32_t elapsed_ms = now_ms - last_imu_update_time_ms_;
    last_imu_update_time_ms_ = now_ms;
    if (elapsed_ms == 0U) return;

    const float dt = static_cast<float>(elapsed_ms) * 0.001f;
    if (!eskf_.predict(accelerometer, gyroscope, dt)) return;

    ap::Matrix3f rotation;
    eskf_.state().attitude.rotation_matrix(rotation);
    linear_acceleration_world_ = rotation * (accelerometer - eskf_.state().accelerometer_bias) -
                                 ap::Vector3f(0.0f, 0.0f, 9.80665f);

    const ap::Quaternion attitude = attitude_estimator_.getQuaternion();
    const ap::Vector3f attitude_variance(0.02f, 0.02f, 0.04f);
    (void)eskf_.fuseAttitude(attitude, attitude_variance, 10.0f);
  }

private:
#ifndef SIMULATION
  IMU *imu_ = nullptr;
  Baro *baro_ = nullptr;
  GPS *gps_ = nullptr;
#endif

  uint32_t last_imu_update_time_ms_{ 0 };
  uint32_t last_external_position_time_ms_{ 0 };
  uint32_t last_external_velocity_time_ms_{ 0 };
  uint32_t last_direct_state_time_ms_{ 0 };
  uint32_t last_direct_velocity_time_ms_{ 0 };

  AttitudeEstimate attitude_estimator_;
  Eskf15 eskf_;
  float barometer_variance_{ 0.25f };
  ap::Vector3f linear_acceleration_world_{};
  Eskf15::State direct_state_{};
  float direct_covariance_[Eskf15::STATE_SIZE]{};
  ap::Vector3f direct_linear_acceleration_world_{};
  ap::Vector3f direct_angular_rate_{};
  uint8_t direct_validity_{ 0U };
  bool direct_state_enabled_{ false };

  bool attitude_estimate_flag_{ false };
  bool altitude_estimate_flag_{ false };
  bool pos_estimate_flag_{ false };
};

#endif  // __STATE_ESTIMATE_H
