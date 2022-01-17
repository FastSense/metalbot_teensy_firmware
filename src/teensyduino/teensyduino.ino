#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include "Robot.hpp"
#include "mcr.hpp"

Robot<config::motors_count> robot(config::base_width);
MicroRosWrapper MRW(config::ros_domain_id);

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
sensor_msgs__msg__Temperature temperature_msg;
geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Pose pose_msg;

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t battery_state_pub;
rcl_publisher_t velocity_pub;
rcl_publisher_t pose_pub;
rcl_publisher_t temperature_pub;

rcl_timer_t stop_timer;
rcl_timer_t pid_timer;
rcl_timer_t pub_timer;

bool on_cmd_vel = false;

void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //заполнение сообщений обратной связи
    velocity_msg.linear.x = robot.getSpeed();
    velocity_msg.angular.z = robot.getAngularSpeed();
    pose_msg.position.x = robot.getPositionX();
    pose_msg.position.y = robot.getPositionY();
    pose_msg.orientation.z = robot.getQuaternionZ();
    pose_msg.orientation.w = robot.getQuaternionW();
    battery_state_msg.voltage = robot.getBatteryVoltage();
    battery_state_msg.current = robot.getBatteryCurrent();
    battery_state_msg.percentage = robot.getBatteryPercentage();
    temperature_msg.temperature = robot.getTemperature();
    //публикация сообщений
    MRW.publish(&velocity_pub, &velocity_msg);
    MRW.publish(&pose_pub, &pose_msg);
    MRW.publish(&battery_state_pub, &battery_state_msg);
    MRW.publish(&temperature_pub, &temperature_msg);
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
      // если пропущено сообщение управления скоростью, регуляторам
      // устанавливается целевое значение {0}
      robot.updateTargetWheelsSpeed(0, 0);
      // тест связи с агентом на хосте
      if (RMW_RET_OK != rmw_uros_ping_agent(/*att period*/ 50, /*att*/ 3))
        robot.hardStopLoop(); // выключение моторов после отрицательного теста
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
  // TODO: ожидание запуска агента на хосте

  robot.start();
  // TODO: иниц-ия[+освобожение] объектов для функции перезапуска без tycmd

  /* MICRO_ROS OBJ-S INIT: */
  MRW.initPub(&velocity_pub,
              ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
              "velocity");
  MRW.initPub(&pose_pub, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
              "pose");

  MRW.initPub(&battery_state_pub,
              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
              "battery_state");

  MRW.initPub(&temperature_pub,
              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
              "temperature");

  MRW.initSub(&cmd_vel_sub,
              ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
              "cmd_vel");

  MRW.initTimer(stop_timer_cb, &stop_timer, config::stop_dt);
  MRW.initTimer(pid_timer_cb, &pid_timer, config::pid_dt);
  MRW.initTimer(pub_timer_cb, &pub_timer, config::pub_dt);

  MRW.initExecutor();
  /* PRIORITY DETERMINED SEQUENCE OF ADDITIONS: */
  MRW.addSub(&cmd_vel_sub, cmd_vel_sub_cb, cmd_vel_msg);
  MRW.addTimer(&stop_timer);
  MRW.addTimer(&pid_timer);
  MRW.addTimer(&pub_timer);
}

void loop() { MRW.spinExecutor(); }
