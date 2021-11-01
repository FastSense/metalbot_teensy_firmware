#ifndef ROS_23646034973906
#define ROS_23646034973906

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/twist.h>

#include "Common.hpp"
#include "Robot.hpp"

//////////
#define my_timer_init(timer, period, cb)                                       \
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period), cb);         \
  callbacks_count++;
//////////
#define my_exe_add_timer(timer) rclc_executor_add_timer(&exe, &timer);
//////////

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t exe;

geometry_msgs__msg__Twist twist_msg_pub;
geometry_msgs__msg__Twist twist_msg;
rcl_subscription_t twist_sub;
rcl_publisher_t twist_pub;
rcl_timer_t pid_timer;
rcl_timer_t pub_timer;
rcl_timer_t stop_timer;
size_t on_twist_msg = 0;
size_t callbacks_count = 0;
Robot_t robot;

void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    twist_msg_pub.linear.x = robot.getV();
    twist_msg_pub.angular.z = robot.getW();
    rcl_publish(&twist_pub, &twist_msg_pub, NULL);
  }
}

void pid_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    robot.updateSpeedRegulation();
  }
}

void stop_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (on_twist_msg)
      on_twist_msg = FALSE;
    else {
      // soft stop
      robot.updateTargetWheelsSpeed(0, 0);
      // Ping the agent
      if (RMW_RET_OK != rmw_uros_ping_agent(/*att period*/ 50, /*att*/ 2))
        robot.hardStopLoop(); // micro-ROS Agent is not available -> stop motors
    }
  }
}

void twist_sub_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  robot.updateTargetWheelsSpeed(msg->linear.x, msg->angular.z);
  on_twist_msg = TRUE; // flag for canceling soft stop
}

void rclSetup() {

  robot.start();
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  // vel_pub
  rclc_publisher_init_default(
      &twist_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "velocity");

  // twist_sub (-
  rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");
  callbacks_count++;
  //           -)
  my_timer_init(stop_timer, STOP_DT, stop_timer_cb);
  my_timer_init(pid_timer, PID_DT, pid_timer_cb);
  my_timer_init(pub_timer, PUB_DT, pub_timer_cb);

  // exe (-
  rclc_executor_init(&exe, &support.context, callbacks_count, &allocator);
  /* PRIORITY DETERMINED SEQUENCE:
   GET CONTROL MSG, CHECK CONNECTION / LOST_MSG, REGULATE, PUBLISH ODOM ETC */
  rclc_executor_add_subscription(&exe, &twist_sub, &twist_msg, &twist_sub_cb,
                                 ON_NEW_DATA);
  my_exe_add_timer(stop_timer);
  my_exe_add_timer(pid_timer);
  my_exe_add_timer(pub_timer);
  //     -)
}

#endif /* end of include guard: ROS_23646034973906 */
