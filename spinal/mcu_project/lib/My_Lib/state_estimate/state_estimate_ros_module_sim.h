#pragma once

#ifdef SIMULATION

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <spinal_msgs/msg/external_state_measurement.hpp>
#include <spinal_msgs/msg/imu.hpp>
#include <spinal_msgs/msg/state_estimate.hpp>

#include "state_estimate/state_estimate.h"

class StateEstimateRosModuleSim
{
public:
  StateEstimateRosModuleSim() = default;

  void init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  {
    node_ = node;

    estimator_.init();

    imu_pub_ = node_->create_publisher<spinal_msgs::msg::Imu>("imu", 1);
    state_pub_ = node_->create_publisher<spinal_msgs::msg::StateEstimate>("state_estimate", 1);
    external_state_sub_ = node_->create_subscription<spinal_msgs::msg::ExternalStateMeasurement>(
        "external_state_measurement", rclcpp::SensorDataQoS(),
        [this](const spinal_msgs::msg::ExternalStateMeasurement::SharedPtr msg)
        {
          if (!msg) return;
          std::lock_guard<std::mutex> lock(external_state_mutex_);
          pending_external_state_ = *msg;
          external_state_pending_ = true;
        });
    last_imu_pub_time_ms_ = nowMs_();
    last_state_pub_time_ms_ = last_imu_pub_time_ms_;
  }

  void update()
  {
    estimator_.update(nowMs_());
    applyPendingExternalState_();
    publishImuIfNeeded_();
    publishStateIfNeeded_();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> getImuPub() { return imu_pub_; }
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::StateEstimate>> getStatePub()
  {
    return state_pub_;
  }

  StateEstimate *getEstimator() { return &estimator_; }

private:
  static constexpr uint8_t IMU_PUB_INTERVAL_MS = 5;
  static constexpr uint8_t STATE_PUB_INTERVAL_MS = 20;

  uint32_t nowMs_() const { return static_cast<uint32_t>(node_->get_clock()->now().nanoseconds() / 1000000ULL); }

  void publishImuIfNeeded_()
  {
    if (!node_ || !imu_pub_) return;

    auto *att = estimator_.getAttEstimator();
    if (!att) return;

    const bool updated = att->consumeUpdated();
    if (!updated) return;

    const uint32_t now_ms = nowMs_();
    if (now_ms - last_imu_pub_time_ms_ < IMU_PUB_INTERVAL_MS) return;
    last_imu_pub_time_ms_ = now_ms;

    imu_msg_.stamp = node_->get_clock()->now();

    const ap::Vector3f mag = att->getMagVec();
    const ap::Vector3f acc = att->getAccVec();
    const ap::Vector3f gyro = att->getGyroVec();

    imu_msg_.mag[0] = mag.x;
    imu_msg_.mag[1] = mag.y;
    imu_msg_.mag[2] = mag.z;
    imu_msg_.acc[0] = acc.x;
    imu_msg_.acc[1] = acc.y;
    imu_msg_.acc[2] = acc.z;
    imu_msg_.gyro[0] = gyro.x;
    imu_msg_.gyro[1] = gyro.y;
    imu_msg_.gyro[2] = gyro.z;

    const ap::Quaternion q = att->getQuaternion();
    imu_msg_.quaternion[0] = q[1];
    imu_msg_.quaternion[1] = q[2];
    imu_msg_.quaternion[2] = q[3];
    imu_msg_.quaternion[3] = q[0];

    imu_pub_->publish(imu_msg_);
  }

  void publishStateIfNeeded_()
  {
    if (!node_ || !state_pub_ || !estimator_.outputInitialized()) return;
    const uint32_t now_ms = nowMs_();
    if (now_ms - last_state_pub_time_ms_ < STATE_PUB_INTERVAL_MS) return;
    last_state_pub_time_ms_ = now_ms;

    const Eskf15::State &state = estimator_.outputState();
    const ap::Vector3f acceleration = estimator_.getLinearAccelerationWorld();
    const ap::Vector3f angular_velocity = estimator_.getAngularRate();
    state_msg_.stamp = node_->get_clock()->now();
    state_msg_.validity = estimator_.stateValidity();
    for (size_t axis = 0; axis < 3; ++axis)
    {
      state_msg_.position[axis] = state.position[axis];
      state_msg_.velocity[axis] = state.velocity[axis];
      state_msg_.acceleration[axis] = acceleration[axis];
      state_msg_.angular_velocity[axis] = angular_velocity[axis];
      state_msg_.accelerometer_bias[axis] = state.accelerometer_bias[axis];
      state_msg_.gyroscope_bias[axis] = state.gyroscope_bias[axis];
    }
    state_msg_.attitude[0] = state.attitude.q2;
    state_msg_.attitude[1] = state.attitude.q3;
    state_msg_.attitude[2] = state.attitude.q4;
    state_msg_.attitude[3] = state.attitude.q1;
    for (size_t index = 0; index < Eskf15::STATE_SIZE; ++index)
    {
      state_msg_.covariance_diagonal[index] = estimator_.outputCovariance(index);
    }
    state_msg_.rejected_measurements = estimator_.outputRejectedMeasurementCount();
    state_pub_->publish(state_msg_);
  }

  void applyPendingExternalState_()
  {
    spinal_msgs::msg::ExternalStateMeasurement msg;
    {
      std::lock_guard<std::mutex> lock(external_state_mutex_);
      if (!external_state_pending_) return;
      msg = pending_external_state_;
      external_state_pending_ = false;
    }

    if (msg.estimate_mode == spinal_msgs::msg::ExternalStateMeasurement::SPINAL_MODE_GROUND_TRUTH)
    {
      (void)estimator_.applyDirectExternalState(
          ap::Vector3f(msg.position[0], msg.position[1], msg.position[2]),
          ap::Vector3f(msg.position_variance[0], msg.position_variance[1], msg.position_variance[2]),
          ap::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]),
          ap::Vector3f(msg.velocity_variance[0], msg.velocity_variance[1], msg.velocity_variance[2]),
          ap::Quaternion(msg.attitude[3], msg.attitude[0], msg.attitude[1], msg.attitude[2]),
          ap::Vector3f(msg.attitude_variance[0], msg.attitude_variance[1], msg.attitude_variance[2]),
          ap::Vector3f(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]), msg.field_mask,
          nowMs_());
      return;
    }

    if (msg.estimate_mode != spinal_msgs::msg::ExternalStateMeasurement::SPINAL_MODE_EGOMOTION &&
        msg.estimate_mode != spinal_msgs::msg::ExternalStateMeasurement::SPINAL_MODE_EXPERIMENT)
    {
      return;
    }

    estimator_.setDirectStateEnabled(false);
    if ((msg.field_mask & spinal_msgs::msg::ExternalStateMeasurement::POSITION) != 0U)
    {
      (void)estimator_.fuseExternalPosition(
          ap::Vector3f(msg.position[0], msg.position[1], msg.position[2]),
          ap::Vector3f(msg.position_variance[0], msg.position_variance[1], msg.position_variance[2]));
    }
    if ((msg.field_mask & spinal_msgs::msg::ExternalStateMeasurement::VELOCITY) != 0U)
    {
      (void)estimator_.fuseExternalVelocity(
          ap::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]),
          ap::Vector3f(msg.velocity_variance[0], msg.velocity_variance[1], msg.velocity_variance[2]));
    }
    if ((msg.field_mask & spinal_msgs::msg::ExternalStateMeasurement::ATTITUDE) != 0U)
    {
      (void)estimator_.fuseExternalAttitude(
          ap::Quaternion(msg.attitude[3], msg.attitude[0], msg.attitude[1], msg.attitude[2]),
          ap::Vector3f(msg.attitude_variance[0], msg.attitude_variance[1], msg.attitude_variance[2]));
    }
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  StateEstimate estimator_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> imu_pub_;
  spinal_msgs::msg::Imu imu_msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::StateEstimate>> state_pub_;
  spinal_msgs::msg::StateEstimate state_msg_;
  rclcpp::Subscription<spinal_msgs::msg::ExternalStateMeasurement>::SharedPtr external_state_sub_;
  spinal_msgs::msg::ExternalStateMeasurement pending_external_state_;
  std::mutex external_state_mutex_;
  bool external_state_pending_{ false };

  uint32_t last_imu_pub_time_ms_{ 0 };
  uint32_t last_state_pub_time_ms_{ 0 };
};

#endif  // SIMULATION
