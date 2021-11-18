#pragma once
#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>
typedef geometry_msgs__msg__Twist Twist_t;
typedef geometry_msgs__msg__Pose Pose_t;

template <class msg_T> class Publisher {
public:
  Publisher(msg_T msg) : msg_(msg) {}
  void setMsg(msg_T msg) { msg_ = msg; }
  void publish() { rcl_publish(&pub_, &msg_, NULL); }

private:
  msg_T msg_;
  rcl_publisher_t pub_;
};
