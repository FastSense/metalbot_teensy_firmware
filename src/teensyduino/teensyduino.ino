#include "common.hpp"
#include "filter.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "uros.hpp"

Robot<config::motors_count> robot(config::base_width);
MicroRosWrapper MRW(config::ros_domain_id);

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
sensor_msgs__msg__Temperature temperature_msg;
geometry_msgs__msg__Twist velocity_msg;
geometry_msgs__msg__Pose pose_msg;
std_msgs__msg__Bool led_msg;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t led_sub;

rcl_publisher_t battery_state_pub;
rcl_publisher_t velocity_pub;
rcl_publisher_t pose_pub;
rcl_publisher_t temperature_pub;

rcl_timer_t stop_timer;
rcl_timer_t pid_timer;
rcl_timer_t pub_timer;

bool micro_ros_init_successful = false;
bool on_cmd_vel = false;
bool on_start = false;

void create_entities();
void destroy_entities();

void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // DBG.println("КБ публикаций");
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
    DBG.print(": ");
    if (on_cmd_vel)
      on_cmd_vel = false;
    else {
      if (on_start) {
        DBG.println("Стоп");
        if (robot.softStop()) { // выключение моторов после отрицательного теста
          on_start = false;
          robot.stop();
        }
      } else
          // тест связи с агентом на хосте
          if (!MRW.checkConnection()) {
        MRW.linkOff();
        destroy_entities();
        // delay(config::reconnection_delay);
      }
    }
    DBG.println(" ");
  }
}

void cmd_vel_sub_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;
  DBG.println("КБ подписчика");
  if (!on_start) {
    robot.activate();
    DBG.println("Старт");
    on_start = true; //проверка получения первого сообщения (начало управления)
  }
  robot.updateTargetWheelsSpeed(msg->linear.x, msg->angular.z);
  on_cmd_vel = true; // flag for canceling soft stop
}

void led_sub_cb(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    digitalWrite(pins::led_1, HIGH);
    digitalWrite(pins::dir_5, HIGH);
  } else {
    digitalWrite(pins::led_1, LOW);
    digitalWrite(pins::dir_5, LOW);
  }
}

void setup() {
  pinMode(pins::led_1, OUTPUT);
  pinMode(pins::led_2, OUTPUT);
  pinMode(pins::dir_5, OUTPUT);
  digitalWrite(pins::led_1, LOW);
  digitalWrite(pins::led_2, HIGH);
  digitalWrite(pins::dir_5, LOW);

  set_microros_transports();
  DBG.begin(115200);
  delay(config::setup_delay);
  DBG.println("~+~");
  DBG.println("Инициализация робота");
  robot.init();
  MRW.waitForConnection();
}

void loop() {
  DBG.println("Запуск основной цикла");
  if (MRW.checkConnection()) {
    MRW.linkOn();
    if (!micro_ros_init_successful) {
      create_entities();
    } else
      MRW.safeSpinLoop();
  } else if (micro_ros_init_successful) {
    destroy_entities();
  }
  MRW.linkOff();
  DBG.println("Ожидание восстановления связи");
  // delay(config::reconnection_delay);
}

void create_entities() {
  digitalWrite(pins::led_2, HIGH);
  robot.stop();
  DBG.println("Инициализация враппера");
  MRW.init();

  DBG.println("Инициализация паблишеров");
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

  DBG.println("Инициализация подписчика");
  MRW.initSub(&cmd_vel_sub,
              ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
              "cmd_vel");
  MRW.initSub(&led_sub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
              "led");

  DBG.println("Инициализация таймеров");
  MRW.initTimer(stop_timer_cb, &stop_timer, config::stop_dt);
  MRW.initTimer(pid_timer_cb, &pid_timer, config::pid_dt);
  MRW.initTimer(pub_timer_cb, &pub_timer, config::pub_dt);

  DBG.println("Инициализация экзекутора и очереди");
  MRW.initExecutor();
  MRW.addSub(&cmd_vel_sub, &cmd_vel_sub_cb, &cmd_vel_msg);
  DBG.println("активирован подписчик");
  MRW.addSub(&led_sub, &led_sub_cb, &led_msg);
  DBG.println("активирован подписчик");
  MRW.addTimer(&pid_timer);
  MRW.addTimer(&pub_timer);
  MRW.addTimer(&stop_timer);
  micro_ros_init_successful = true;
  DBG.println("Сущности созданы");
  digitalWrite(pins::led_2, LOW);
}

void destroy_entities() {
  digitalWrite(pins::led_2, HIGH);
  robot.stop();
  DBG.println("Удаление всех сущностей..");
  MRW.finiPub(&temperature_pub);
  DBG.println("удален паблишер");
  MRW.finiPub(&battery_state_pub);
  DBG.println("удален паблишер");
  MRW.finiPub(&pose_pub);
  DBG.println("удален паблишер");
  MRW.finiPub(&velocity_pub);
  DBG.println("удален паблишер");
  MRW.finiSub(&cmd_vel_sub);
  DBG.println("удален подписчик");
  MRW.finiSub(&led_sub);
  DBG.println("удален подписчик");
  MRW.finiTimer(&pub_timer);
  DBG.println("удален таймер");
  MRW.finiTimer(&pid_timer);
  DBG.println("удален таймер");
  MRW.finiTimer(&stop_timer);
  DBG.println("удален таймер");
  MRW.finiExecutor();
  DBG.println("удален экзекутор");
  MRW.fini();
  DBG.println("удален враппер");
  micro_ros_init_successful = false;
  on_cmd_vel = false;
  on_start = false;
  DBG.println("..завершено");
  digitalWrite(pins::led_2, LOW);
}
