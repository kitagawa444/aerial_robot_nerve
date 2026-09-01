#pragma once

#ifndef SIMULATION

#include <ros_utils/ros_module_base.hpp>
#include <rmw_microros/rmw_microros.h>

#include <spinal_msgs/msg/imu.h>
#include <spinal_msgs/msg/external_state_measurement.h>
#include <spinal_msgs/msg/state_estimate.h>
#include <spinal_msgs/srv/mag_declination.h>

#include "state_estimate/state_estimate.h"

class StateEstimateRosModule final : public RosModuleBase
{
public:
  StateEstimateRosModule()
    : RosModuleBase(RosModuleEntityCapacity().max_subscriptions(1).max_publishers(2).max_services(1).max_timers(0))
  {
  }

  void init_hw(IMU *imu, Baro *baro, GPS *gps, osMutexId *state_mutex = nullptr, bool attitude_enabled = true,
               bool height_enabled = true, bool position_enabled = true)
  {
    state_mutex_ = state_mutex;
    estimator_.init(imu, baro, gps, attitude_enabled, height_enabled, position_enabled);
  }

  void create_entities(rcl_node_t &node) override;
  void update() override;

  StateEstimate *getStateEstimateCore() { return &estimator_; }
  void publish() override;
  uint32_t millisToNextPublish() const;

private:
  static constexpr uint8_t IMU_PUB_INTERVAL_MS = 5;     // 200Hz
  static constexpr uint8_t STATE_PUB_INTERVAL_MS = 20;  // 50Hz
  uint32_t last_imu_pub_time_ms_{ 0 };
  uint32_t last_state_pub_time_ms_{ 0 };

  // core
  StateEstimate estimator_;

  // pub
  rcl_publisher_t imu_pub_{};
  spinal_msgs__msg__Imu imu_msg_{};
  rcl_publisher_t state_pub_{};
  spinal_msgs__msg__StateEstimate state_msg_{};

  rcl_subscription_t external_state_sub_{};
  spinal_msgs__msg__ExternalStateMeasurement external_state_msg_{};
  spinal_msgs__msg__ExternalStateMeasurement pending_external_state_msg_{};
  osMutexId *state_mutex_{ nullptr };
  bool external_state_pending_{ false };

  // service
  rcl_service_t mag_declination_srv_{};
  spinal_msgs__srv__MagDeclination_Request mag_declination_req_{};
  spinal_msgs__srv__MagDeclination_Response mag_declination_res_{};

  // trampoline
  static StateEstimateRosModule *instance_;
  static void magDeclinationCallbackStatic(const void *req_msg, void *res_msg);
  static void externalStateCallbackStatic(const void *msgin);

  void magDeclinationCallback(const spinal_msgs__srv__MagDeclination_Request &req,
                              spinal_msgs__srv__MagDeclination_Response &res);
  void applyPendingExternalState();
  void lockState();
  void unlockState();
};

#endif  // !SIMULATION
