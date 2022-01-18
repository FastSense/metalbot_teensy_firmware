#pragma once
#include "Common.hpp"

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
  MicroRosWrapper(size_t id_) {
    id = id_;
    callbacks_count = 0;
  }

  void init() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    node_ops = rcl_node_get_default_options();
    node_ops.domain_id = id;
    rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support,
                                &node_ops);
  }
  void fini() {
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    callbacks_count = 0;
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
  void finiPub(rcl_publisher_t *pub) { rcl_publisher_fini(pub, &node); }

  template <class supportT, class nameT> /////////
  void initSub(rcl_subscription_t *sub, supportT sup, nameT name) {
    rclc_subscription_init_default(sub, &node, sup, name);
    callbacks_count++;
  }
  void finiSub(rcl_subscription_t *sub) { rcl_subscription_fini(sub, &node); }

  template <class Callable, class deltaT> /////////
  void initTimer(Callable cb, rcl_timer_t *timer, deltaT delta) {
    rclc_timer_init_default(timer, &support, RCL_MS_TO_NS(delta), cb);
    callbacks_count++;
  }
  void finiTimer(rcl_timer_t *timer) { rcl_timer_fini(timer); }

  void initExecutor() {
    rclc_executor_init(&executor, &support.context, callbacks_count,
                       &allocator);
  }
  void finiExecutor() { rclc_executor_fini(&executor); }

  /* PRIORITY DETERMINED SEQUENCE OF ADDITIONS: */
  template <class Callable, class msgT> /////////
  void addSub(rcl_subscription_t *sub, Callable cb, msgT msg) {
    rclc_executor_add_subscription(&executor, sub, msg, cb, ON_NEW_DATA);
  }
  void addTimer(rcl_timer_t *timer) {
    rclc_executor_add_timer(&executor, timer);
  }

  /* CUSTOM SPIN FUNC*/
  void safeSpinLoop() {
    if (linked) {
      DBG.print("Запущен цикл экзекутора. КБ в очереди: ");
      DBG.println(callbacks_count);
      while (linked)
        rclc_executor_spin_some(&executor, (&executor)->timeout_ns * 0.1);
      // default timeout is 1000ms
    }
  }
  // void toggleLinked() {
  //   linked = !linked;
  //   DBG.println("переключен флаг соединения");
  // }
  void linkOn() {
    linked = true;
    DBG.println("активирован флаг соединения");
  }
  void linkOff() {
    linked = false;
    DBG.println("погашен флаг соединения");
  }

  void waitForConnection() {
    DBG.println("Ожидание соединения");
    while (!linked) {
      static int counter = 0; // DBG
      DBG.println(counter);
      counter++;
      if (RMW_RET_OK == rmw_uros_ping_agent(/*att period*/ 100, /*att*/ 1))
        linkOn();
    }
  }

  bool checkConnection() {
    DBG.print("Проверка соединения..");
    if (RMW_RET_OK == rmw_uros_ping_agent(/*att period*/ 1, /*att*/ 2)) {
      DBG.println("есть контакт");
      return true;
    } else {
      DBG.println("нет связи");
      return false;
    }
  }

private:
  size_t callbacks_count;

  rclc_support_t support;
  rcl_node_options_t node_ops;
  rcl_node_t node;
  rclc_executor_t executor;
  rcl_allocator_t allocator;

  bool linked = false;
  size_t id;
};
