#ifndef SIMULATION

#include "state_estimate/state_estimate_ros_module.h"

#include <rcl/error_handling.h>
#include <string.h>

StateEstimateRosModule *StateEstimateRosModule::instance_ = nullptr;
uint32_t now_ms_test;

void StateEstimateRosModule::create_entities(rcl_node_t &node)
{
  reserve_entities();

  // publisher
  (void)init_publisher_default(node, imu_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Imu), "imu");

  (void)init_publisher_default(node, state_pub_, ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, StateEstimate),
                               "state_estimate");

  spinal_msgs__msg__ExternalStateMeasurement__init(&external_state_msg_);
  spinal_msgs__msg__ExternalStateMeasurement__init(&pending_external_state_msg_);
  spinal_msgs__msg__StateEstimate__init(&state_msg_);

  (void)init_subscription_default(node, external_state_sub_,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ExternalStateMeasurement),
                                  "external_state_measurement", &external_state_msg_,
                                  &StateEstimateRosModule::externalStateCallbackStatic, ON_NEW_DATA);

  spinal_msgs__srv__MagDeclination_Request__init(&mag_declination_req_);
  spinal_msgs__srv__MagDeclination_Response__init(&mag_declination_res_);

  (void)init_service_default(node, mag_declination_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, MagDeclination),
                             "mag_declination", &mag_declination_req_, &mag_declination_res_,
                             &magDeclinationCallbackStatic);

  instance_ = this;
  last_imu_pub_time_ms_ = HAL_GetTick();
  last_state_pub_time_ms_ = last_imu_pub_time_ms_;
}

void StateEstimateRosModule::externalStateCallbackStatic(const void *msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;
  const auto *msg = reinterpret_cast<const spinal_msgs__msg__ExternalStateMeasurement *>(msgin);
  instance_->lockState();
  instance_->pending_external_state_msg_ = *msg;
  instance_->external_state_pending_ = true;
  instance_->unlockState();
}

void StateEstimateRosModule::update()
{
  lockState();
  estimator_.update();
  unlockState();
  applyPendingExternalState();
}

void StateEstimateRosModule::applyPendingExternalState()
{
  spinal_msgs__msg__ExternalStateMeasurement msg{};
  lockState();
  if (!external_state_pending_)
  {
    unlockState();
    return;
  }
  msg = pending_external_state_msg_;
  external_state_pending_ = false;

  if (msg.estimate_mode == spinal_msgs__msg__ExternalStateMeasurement__SPINAL_MODE_GROUND_TRUTH)
  {
    (void)estimator_.applyDirectExternalState(
        ap::Vector3f(msg.position[0], msg.position[1], msg.position[2]),
        ap::Vector3f(msg.position_variance[0], msg.position_variance[1], msg.position_variance[2]),
        ap::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]),
        ap::Vector3f(msg.velocity_variance[0], msg.velocity_variance[1], msg.velocity_variance[2]),
        ap::Quaternion(msg.attitude[3], msg.attitude[0], msg.attitude[1], msg.attitude[2]),
        ap::Vector3f(msg.attitude_variance[0], msg.attitude_variance[1], msg.attitude_variance[2]),
        ap::Vector3f(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]), msg.field_mask,
        HAL_GetTick());
    unlockState();
    return;
  }

  if (msg.estimate_mode != spinal_msgs__msg__ExternalStateMeasurement__SPINAL_MODE_EGOMOTION &&
      msg.estimate_mode != spinal_msgs__msg__ExternalStateMeasurement__SPINAL_MODE_EXPERIMENT)
  {
    unlockState();
    return;
  }

  estimator_.setDirectStateEnabled(false);

  if ((msg.field_mask & spinal_msgs__msg__ExternalStateMeasurement__POSITION) != 0U)
  {
    (void)estimator_.fuseExternalPosition(
        ap::Vector3f(msg.position[0], msg.position[1], msg.position[2]),
        ap::Vector3f(msg.position_variance[0], msg.position_variance[1], msg.position_variance[2]));
  }
  if ((msg.field_mask & spinal_msgs__msg__ExternalStateMeasurement__VELOCITY) != 0U)
  {
    (void)estimator_.fuseExternalVelocity(
        ap::Vector3f(msg.velocity[0], msg.velocity[1], msg.velocity[2]),
        ap::Vector3f(msg.velocity_variance[0], msg.velocity_variance[1], msg.velocity_variance[2]));
  }
  if ((msg.field_mask & spinal_msgs__msg__ExternalStateMeasurement__ATTITUDE) != 0U)
  {
    (void)estimator_.fuseExternalAttitude(
        ap::Quaternion(msg.attitude[3], msg.attitude[0], msg.attitude[1], msg.attitude[2]),
        ap::Vector3f(msg.attitude_variance[0], msg.attitude_variance[1], msg.attitude_variance[2]));
  }
  unlockState();
}

void StateEstimateRosModule::lockState()
{
  if (state_mutex_ != nullptr && *state_mutex_ != nullptr) osMutexWait(*state_mutex_, osWaitForever);
}

void StateEstimateRosModule::unlockState()
{
  if (state_mutex_ != nullptr && *state_mutex_ != nullptr) osMutexRelease(*state_mutex_);
}

uint32_t StateEstimateRosModule::millisToNextPublish() const
{
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t elapsed_ms = now_ms - last_imu_pub_time_ms_;

  if (elapsed_ms >= IMU_PUB_INTERVAL_MS) return 0;
  return IMU_PUB_INTERVAL_MS - elapsed_ms;
}

void StateEstimateRosModule::magDeclinationCallbackStatic(const void *req_msg, void *res_msg)
{
  if (!instance_) return;
  if (!req_msg || !res_msg) return;

  instance_->magDeclinationCallback(*reinterpret_cast<const spinal_msgs__srv__MagDeclination_Request *>(req_msg),
                                    *reinterpret_cast<spinal_msgs__srv__MagDeclination_Response *>(res_msg));
}

void StateEstimateRosModule::magDeclinationCallback(const spinal_msgs__srv__MagDeclination_Request &req,
                                                    spinal_msgs__srv__MagDeclination_Response &res)
{
  // defaults
  res.success = false;

  lockState();
  AttitudeEstimate *att = estimator_.getAttEstimator();
  if (!att)
  {
    unlockState();
    return;
  }

  switch (req.command)
  {
    case spinal_msgs__srv__MagDeclination_Request__GET_DECLINATION:
      res.data = att->getMagDeclination();
      res.success = true;
      break;

    case spinal_msgs__srv__MagDeclination_Request__SET_DECLINATION:
      att->setMagDeclination(req.data);
      res.success = true;
      break;

    default:
      break;
  }
  unlockState();
}

void StateEstimateRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  const uint32_t now_ms = HAL_GetTick();
  now_ms_test = now_ms;

  lockState();
  AttitudeEstimate *att = estimator_.getAttEstimator();
  if (!att)
  {
    unlockState();
    return;
  }

  const uint64_t t_ms = rmw_uros_epoch_millis();

  if (now_ms - last_imu_pub_time_ms_ >= IMU_PUB_INTERVAL_MS && att->consumeUpdated())
  {
    last_imu_pub_time_ms_ = now_ms;
    const ap::Vector3f mag = att->getMagVec();
    const ap::Vector3f acc = att->getAccVec();
    const ap::Vector3f gyro = att->getGyroVec();
    const ap::Quaternion q = att->getQuaternion();
    imu_msg_.stamp.sec = (int32_t)(t_ms / 1000ULL);
    imu_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);
    imu_msg_.mag[0] = mag.x;
    imu_msg_.mag[1] = mag.y;
    imu_msg_.mag[2] = mag.z;
    imu_msg_.acc[0] = acc.x;
    imu_msg_.acc[1] = acc.y;
    imu_msg_.acc[2] = acc.z;
    imu_msg_.gyro[0] = gyro.x;
    imu_msg_.gyro[1] = gyro.y;
    imu_msg_.gyro[2] = gyro.z;
    imu_msg_.quaternion[0] = q[1];
    imu_msg_.quaternion[1] = q[2];
    imu_msg_.quaternion[2] = q[3];
    imu_msg_.quaternion[3] = q[0];
    (void)rcl_publish(&imu_pub_, &imu_msg_, nullptr);
  }

  if (!estimator_.outputInitialized() || now_ms - last_state_pub_time_ms_ < STATE_PUB_INTERVAL_MS)
  {
    unlockState();
    return;
  }
  last_state_pub_time_ms_ = now_ms;
  const Eskf15::State &state = estimator_.outputState();
  const ap::Vector3f acceleration = estimator_.getLinearAccelerationWorld();
  const ap::Vector3f angular_velocity = estimator_.getAngularRate();
  state_msg_.stamp.sec = (int32_t)(t_ms / 1000ULL);
  state_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);
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
  (void)rcl_publish(&state_pub_, &state_msg_, nullptr);
  unlockState();
}

#endif  // !SIMULATION
