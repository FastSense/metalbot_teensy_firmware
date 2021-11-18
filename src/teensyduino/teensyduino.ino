#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include "Robot.hpp"
#include "mcr.hpp"
// #include "publisher.hpp"
// #include "Ros.hpp"
// #include <Array.h>

// Twist_t twist_msg;
// Pose_t pose_msg;
//
// Publisher<Twist_t> velocity_publisher(twist_msg);
// Publisher<Pose_t> pose_publisher(pose_msg);

Robot robot;

MicroRosWrapper micro_ros_wpapper;

rcl_publisher_t twist_pub;
rcl_publisher_t pose_pub;

rcl_timer_t stop_timer;
rcl_timer_t pid_timer;
rcl_timer_t pub_timer;

geometry_msgs__msg__Twist cmd_vel_msg;

geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Pose pose_msg;

bool on_twist_msg = false;

void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    velocity_msg.linear.x = robot.getSpeed();
    velocity_msg.angular.z = robot.getAngularSpeed();

    pose_msg.position.x = robot.getPositionX();
    pose_msg.position.y = robot.getPositionY();
    pose_msg.orientation.z = robot.getQuaternionZ();
    pose_msg.orientation.w = robot.getQuaternionW();

    micro_ros_wpapper.publish(&twist_pub, &velocity_msg);
    micro_ros_wpapper.publish(&pose_pub, &pose_msg);
  }
}

void pid_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    robot.updateSpeedRegulation();
    robot.updateOdometry();
  }
}

void stop_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (on_twist_msg)
      on_twist_msg = false;
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
  on_twist_msg = true; // flag for canceling soft stop
}

void setup() {
  delay(config::setup_delay);

  micro_ros_wpapper.initPub(&twist_pub, GET_SUPPORT(Twist), "velocity");
  micro_ros_wpapper.initPub(&pose_pub, GET_SUPPORT(Pose), "pose");
  micro_ros_wpapper.initTwistSub();
  micro_ros_wpapper.initTimer(stop_timer_cb, &stop_timer, config::stop_dt);
  micro_ros_wpapper.initTimer(pid_timer_cb, &pid_timer, config::pid_dt);
  micro_ros_wpapper.initTimer(pub_timer_cb, &pub_timer, config::pub_dt);
  micro_ros_wpapper.initExecutor();
  micro_ros_wpapper.addSub(twist_sub_cb, cmd_vel_msg);
  micro_ros_wpapper.addTimer(&stop_timer);
  micro_ros_wpapper.addTimer(&pid_timer);
  micro_ros_wpapper.addTimer(&pub_timer);
}

void loop() { micro_ros_wpapper.spinExecutor(); }
