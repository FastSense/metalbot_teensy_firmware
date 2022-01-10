#pragma once

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>

class MicroRosWrapper {
public:
  MicroRosWrapper(size_t id) {
    callbacks_count = 0;
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    node_ops = rcl_node_get_default_options();
    node_ops.domain_id = id;
    rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support,
                                &node_ops);
  }

  template <class msgT> /////////
  void publish(rcl_publisher_t *pub, msgT *msg) {
    rcl_publish(pub, msg, NULL);
  }

  /* MICRO_ROS OBJ-S INIT: */
  template <class supportT, class nameT> /////////
  void initPub(rcl_publisher_t *pub, supportT sup, nameT name) {
    rclc_publisher_init_default(pub, &node, sup, name);
  }
  template <class supportT, class nameT> /////////
  void initSub(rcl_subscription_t *sub, supportT sup, nameT name) {
    rclc_subscription_init_default(sub, &node, sup, name);
    callbacks_count++;
  }
  template <class Callable, class deltaT> /////////
  void initTimer(Callable cb, rcl_timer_t *timer, deltaT delta) {
    rclc_timer_init_default(timer, &support, RCL_MS_TO_NS(delta), cb);
    callbacks_count++;
  }
  void initExecutor() {
    rclc_executor_init(&executor, &support.context, callbacks_count,
                       &allocator);
  }

  /* PRIORITY DETERMINED SEQUENCE OF ADDITIONS: */
  template <class Callable, class msgT> /////////
  void addSub(rcl_subscription_t *sub, Callable cb, msgT msg) {
    rclc_executor_add_subscription(&executor, sub, &msg, cb, ON_NEW_DATA);
  }
  void addTimer(rcl_timer_t *timer) {
    rclc_executor_add_timer(&executor, timer);
  }

  /* ENDLESS LOOP */
  void spinExecutor() { rclc_executor_spin(&executor); }

private:
  size_t callbacks_count;

  rcl_allocator_t allocator;
  rcl_node_options_t node_ops;
  rcl_node_t node;

  rclc_executor_t executor;
  rclc_support_t support;
};
