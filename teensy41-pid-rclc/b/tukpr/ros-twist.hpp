#ifndef ROS_TWIST_23646034973906
#define ROS_TWIST_23646034973906

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/twist.h>

#include "Robot.hpp"

constexpr size_t PID_DT = 20;
constexpr size_t STOP_DT = 250;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rclc_executor_t exe;
size_t callbacks_count = 0;

geometry_msgs__msg__Twist twist_msg;
rcl_subscription_t twist_sub;

rcl_timer_t pid_timer;

rcl_timer_t stop_timer;
size_t on_twist_msg = 0;

Robot_t robot;

void pid_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    robot.updateRegulation(PID_DT);
  }
}

void stop_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (on_twist_msg) {
      on_twist_msg = 0;
    } else {
      robot.updateTargetWheelsSpeed({{0, 0, 0}, {0, 0, 0}});
    }
  }
}

void twist_sub_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  robot.updateTargetWheelsSpeed(
      {{msg->linear.x, 0, 0}, {0, 0, msg->angular.z}});
  on_twist_msg = 1;
}

void rclSetup() {

  robot.start();

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  // twist_sub
  rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");
  callbacks_count++;

  // pid_timer
  rclc_timer_init_default(&pid_timer, &support, RCL_MS_TO_NS(PID_DT),
                          pid_timer_cb);
  callbacks_count++;

  // stop_timer
  rclc_timer_init_default(&stop_timer, &support, RCL_MS_TO_NS(STOP_DT),
                          stop_timer_cb);
  callbacks_count++;

  // exe
  rclc_executor_init(&exe, &support.context, callbacks_count, &allocator);
  // add timer1 cb
  rclc_executor_add_timer(&exe, &pid_timer);
  // add timer2 cb
  rclc_executor_add_timer(&exe, &stop_timer);
  // add sub cb
  rclc_executor_add_subscription(&exe, &twist_sub, &twist_msg, &twist_sub_cb,
                                 ON_NEW_DATA);
}

#endif /* end of include guard: ROS-TWIST_23646034973906 */
