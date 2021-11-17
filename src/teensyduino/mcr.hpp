#pragma once

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>

namespace mcr {

class MicroRosWrapper {
public:
  MicroRosWrapper() : callbacks_count_(0) {
    set_microros_transports();
    allocator_ = rcl_get_default_allocator();
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "micro_ros_arduino_node", "", &support_);
  }

private:
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  size_t callbacks_count_;
};

} // namespace mcr
