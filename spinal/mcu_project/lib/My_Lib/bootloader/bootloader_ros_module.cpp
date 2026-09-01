#ifndef SIMULATION

#include "bootloader/bootloader_ros_module.h"

#include <rosidl_runtime_c/string_functions.h>

#include "bootloader/system_bootloader.h"
#include "flight_control/flight_control.h"
#include "thruster/board/thruster_manager.h"

BootloaderRosModule *BootloaderRosModule::instance_ = nullptr;

void BootloaderRosModule::init_hw(ThrusterManager *thruster, FlightControl *flight_control)
{
  thruster_ = thruster;
  flight_control_ = flight_control;
}

void BootloaderRosModule::create_entities(rcl_node_t &node)
{
  reserve_entities();
  instance_ = this;

  const bool request_initialized = std_srvs__srv__Trigger_Request__init(&request_);
  const bool response_initialized = std_srvs__srv__Trigger_Response__init(&response_);
  const bool reboot_request_initialized = std_srvs__srv__Trigger_Request__init(&reboot_request_);
  const bool reboot_response_initialized = std_srvs__srv__Trigger_Response__init(&reboot_response_);
  messages_initialized_ = request_initialized && response_initialized && reboot_request_initialized &&
                          reboot_response_initialized;

  if (!messages_initialized_)
  {
    if (request_initialized)
    {
      std_srvs__srv__Trigger_Request__fini(&request_);
    }
    if (response_initialized)
    {
      std_srvs__srv__Trigger_Response__fini(&response_);
    }
    if (reboot_request_initialized) std_srvs__srv__Trigger_Request__fini(&reboot_request_);
    if (reboot_response_initialized) std_srvs__srv__Trigger_Response__fini(&reboot_response_);
    return;
  }

  (void)init_service_default(node, enter_bootloader_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                             "enter_bootloader", &request_, &response_,
                             &BootloaderRosModule::enterBootloaderCallbackStatic_);

  (void)init_service_default(node, reboot_srv_, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "fc/reboot",
                             &reboot_request_, &reboot_response_, &BootloaderRosModule::rebootCallbackStatic_);
}

void BootloaderRosModule::destroy_entities(rcl_node_t &node)
{
  RosModuleBase::destroy_entities(node);
  if (messages_initialized_)
  {
    std_srvs__srv__Trigger_Request__fini(&request_);
    std_srvs__srv__Trigger_Response__fini(&response_);
    std_srvs__srv__Trigger_Request__fini(&reboot_request_);
    std_srvs__srv__Trigger_Response__fini(&reboot_response_);
    request_ = std_srvs__srv__Trigger_Request{};
    response_ = std_srvs__srv__Trigger_Response{};
    reboot_request_ = std_srvs__srv__Trigger_Request{};
    reboot_response_ = std_srvs__srv__Trigger_Response{};
    messages_initialized_ = false;
  }
}

void BootloaderRosModule::update()
{
  if (!reset_pending_.load(std::memory_order_acquire))
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if (static_cast<int32_t>(now - reset_at_ms_) >= 0)
  {
    if (application_reset_) SystemBootloader::request_application_reset();
    SystemBootloader::request_and_reset();
  }
}

void BootloaderRosModule::enterBootloaderCallbackStatic_(const void *request, void *response)
{
  (void)request;
  auto *result = static_cast<std_srvs__srv__Trigger_Response *>(response);

  if (instance_ == nullptr || instance_->thruster_ == nullptr)
  {
    result->success = false;
    (void)rosidl_runtime_c__String__assign(&result->message, "bootloader is not initialized");
    return;
  }

  // Latch all motor outputs at idle before acknowledging the update request.
  instance_->thruster_->stopOutputs();
  instance_->reset_at_ms_ = HAL_GetTick() + kResetDelayMs;
  instance_->application_reset_ = false;
  instance_->reset_pending_.store(true, std::memory_order_release);

  result->success = true;
  (void)rosidl_runtime_c__String__assign(&result->message, "motor outputs stopped; ROM bootloader reset scheduled");
}

void BootloaderRosModule::rebootCallbackStatic_(const void *request, void *response)
{
  (void)request;
  auto *result = static_cast<std_srvs__srv__Trigger_Response *>(response);
  if (instance_ == nullptr || instance_->thruster_ == nullptr || instance_->flight_control_ == nullptr)
  {
    result->success = false;
    (void)rosidl_runtime_c__String__assign(&result->message, "reboot service is not initialized");
    return;
  }

  if (instance_->flight_control_->getSupervisor().status().arming_state == FlightArmingState::ARMED)
  {
    result->success = false;
    (void)rosidl_runtime_c__String__assign(&result->message, "reboot rejected while armed");
    return;
  }

  instance_->thruster_->stopOutputs();
  instance_->reset_at_ms_ = HAL_GetTick() + kResetDelayMs;
  instance_->application_reset_ = true;
  instance_->reset_pending_.store(true, std::memory_order_release);
  result->success = true;
  (void)rosidl_runtime_c__String__assign(&result->message,
                                         "motor outputs stopped; normal application reboot scheduled");
}

#endif  // !SIMULATION
