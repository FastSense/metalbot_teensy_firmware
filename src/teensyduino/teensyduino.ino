#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include "Robot.hpp"
#include "mcr.hpp"

Robot<config::motors_count> robot(config::base_width);

MicroRosWrapper micro_ros_wpapper(config::ros_domain_id);

rcl_subscription_t cmd_vel_sub;

rcl_publisher_t velocity_pub;
rcl_publisher_t pose_pub;

rcl_timer_t stop_timer;
rcl_timer_t pid_timer;
rcl_timer_t pub_timer;

geometry_msgs__msg__Twist cmd_vel_msg;

geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Pose pose_msg;

bool on_cmd_vel = false;

void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    velocity_msg.linear.x = robot.getSpeed();
    velocity_msg.angular.z = robot.getAngularSpeed();

    pose_msg.position.x = robot.getPositionX();
    pose_msg.position.y = robot.getPositionY();
    pose_msg.orientation.z = robot.getQuaternionZ();
    pose_msg.orientation.w = robot.getQuaternionW();

    micro_ros_wpapper.publish(&velocity_pub, &velocity_msg);
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
    if (on_cmd_vel)
      on_cmd_vel = false;
    else {
      // soft stop
      robot.updateTargetWheelsSpeed(0, 0);
      // Ping the agent
      if (RMW_RET_OK != rmw_uros_ping_agent(/*att period*/ 50, /*att*/ 3))
        robot.hardStopLoop(); // micro-ROS Agent is not available -> stop motors
    }
  }
}

void cmd_vel_sub_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  robot.updateTargetWheelsSpeed(msg->linear.x, msg->angular.z);
  on_cmd_vel = true; // flag for canceling soft stop
}

void setup() {
  delay(config::setup_delay);

  robot.start();

  /* MICRO_ROS OBJ-S INIT: */
  micro_ros_wpapper.initPub(&velocity_pub, GET_SUPPORT(Twist), "velocity");
  micro_ros_wpapper.initPub(&pose_pub, GET_SUPPORT(Pose), "pose");

  micro_ros_wpapper.initSub(&cmd_vel_sub, GET_SUPPORT(Twist), "cmd_vel");

  micro_ros_wpapper.initTimer(stop_timer_cb, &stop_timer, config::stop_dt);
  micro_ros_wpapper.initTimer(pid_timer_cb, &pid_timer, config::pid_dt);
  micro_ros_wpapper.initTimer(pub_timer_cb, &pub_timer, config::pub_dt);

  micro_ros_wpapper.initExecutor();

  /* PRIORITY DETERMINED SEQUENCE OF ADDITIONS: */
  micro_ros_wpapper.addSub(&cmd_vel_sub, cmd_vel_sub_cb, cmd_vel_msg);

  micro_ros_wpapper.addTimer(&stop_timer);
  micro_ros_wpapper.addTimer(&pid_timer);
  micro_ros_wpapper.addTimer(&pub_timer);
}

void loop() { micro_ros_wpapper.spinExecutor(); }
