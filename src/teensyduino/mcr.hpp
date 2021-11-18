#pragma once

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>

#define GET_SUPPORT(item) ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, item)

class MicroRosWrapper {
public:
  MicroRosWrapper() : callbacks_count(0) {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);
  }

  template <class T, class nameT> ///////////
  void initPub(rcl_publisher_t *pub, T sup, nameT name) {
    rclc_publisher_init_default(pub, &node, sup, name);
  }
  void initTwistSub() {
    rclc_subscription_init_default(
        &twist_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    callbacks_count++;
  }

  template <class Callable, class T>
  void initTimer(Callable cb, rcl_timer_t *timer, T delta) {
    rclc_timer_init_default(timer, &support, RCL_MS_TO_NS(delta), cb);
    callbacks_count++;
  }

  void initExecutor() {
    rclc_executor_init(&executor, &support.context, callbacks_count,
                       &allocator);
  }
  /* PRIORITY DETERMINED SEQUENCE:*/
  template <class Callable, class msgT> void addSub(Callable cb, msgT msg) {
    rclc_executor_add_subscription(&executor, &twist_sub, &msg, cb,
                                   ON_NEW_DATA);
  }

  void addTimer(rcl_timer_t *timer) {
    rclc_executor_add_timer(&executor, timer);
  }

  void spinExecutor() { rclc_executor_spin(&executor); }
  ////////////////
  template <class msgT> void publish(rcl_publisher_t *pub, msgT *msg) {
    rcl_publish(pub, msg, NULL);
  }

private:
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;

  size_t callbacks_count;

  rcl_subscription_t twist_sub;
};
