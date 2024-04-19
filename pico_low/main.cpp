#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "imu.h"
#include "kinematics.h"
#include "motors.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.c"
// только так (расширене не .h ,а .c)
// работает передача данных по юарт.
#include "locker.h"
#include "rpm.h"
#include "servo.h"

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#define LED_PIN 25

bool flag = true;

bool locker_state_up = false;
bool locker_state_down = false;

rpm current_rpm;
imp_num current_imp_num;
struct repeating_timer rpm_timer;

rcl_publisher_t rpm_publisher;
nav_msgs__msg__Odometry odom_data;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_data;

rcl_publisher_t locker_publisher;
std_msgs__msg__UInt8MultiArray locker_status;

rcl_publisher_t servo_publisher;
std_msgs__msg__UInt8MultiArray servo_status;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_subscription_t servo_subscriber;
std_msgs__msg__UInt8MultiArray servo_move;

Kinematics kinematics(Kinematics::LINO_BASE, MOTOR_MAX_RPM, MAX_RPM_RATIO,
#ifdef BRUSH_MOTORS
                      MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
#endif
#ifdef BRUSHLESS_MOTORS
                      MAX_RPM,
#endif
                      WHEEL_DIAMETER, LR_WHEELS_DISTANCE);

IMU::data imu_dat;
Kinematics::velocities cmd_vel;

void blink() {
  gpio_put(LED_PIN, flag);
  flag = !flag;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  rcl_ret_t ret;

  // wtf with imu data ????
  imu_dat = IMU::get_data();
  imu_data.angular_velocity.z =
      imu_dat.ang_z; // Отправляется не угловая скорость, а угол!!!!!!
  imu_data.linear_acceleration.x = imu_dat.lin_accel_x;
  imu_data.linear_acceleration.y = imu_dat.lin_accel_y;
  ret = rcl_publish(&imu_publisher, &imu_data, nullptr);

  // publish all data
  blink();
}

void motor_subscriber_callback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Twist *)msgin;

  cmd_vel.linear_x = msg->linear.x;
  cmd_vel.linear_y = msg->linear.y;
  cmd_vel.angular_z = msg->angular.z;

  Kinematics::rpm req_rpm =
      kinematics.getRPM(msg->linear.x, msg->linear.y, msg->angular.z);

  motor1_controller((int)req_rpm.motor1); // обращаемся к элементу motor1
  motor2_controller((int)req_rpm.motor2);
  motor3_controller((int)req_rpm.motor3);
  motor4_controller((int)req_rpm.motor4);

  blink();
}

void servo_subscriber_callback(const void *msgin) {
  const auto *msg = (const geometry_msgs__msg__Vector3 *)msgin;
  servo_set_angle((uint8_t)msg->x, (int16_t)msg->y);

  blink();
}

int main() {
  //  rmw_uros_set_custom_transport(
  //      true, nullptr, pico_serial_transport_open,
  //      pico_serial_transport_close, pico_serial_transport_write,
  //      pico_serial_transport_read);

  motor_init_all();
  servo_init_all();
  lockers_init();
  IMU::imu_init();
  impulse_counter_init();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  //  rcl_timer_t timer;
  //  rcl_node_t node;
  //  rcl_allocator_t allocator;
  //  rclc_support_t support;
  //  rclc_executor_t executor;
  //
  //  allocator = rcl_get_default_allocator();
  //
  //  // Wait for agent successful ping for 2 minutes.
  //  const int timeout_ms = 1000;
  //  const uint8_t attempts = 120;
  //
  //  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
  //
  //  if (ret != RCL_RET_OK) {
  //    // Unreachable agent, exiting program.
  //    return ret;
  //  }
  //
  //  rclc_support_init(&support, 0, nullptr, &allocator);
  //
  //  rclc_node_init_default(&node, "pico_low", "", &support);
  //
  //  rclc_subscription_init_default(
  //      &twist_subscriber, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  //
  //  rclc_publisher_init_default(
  //      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,
  //      Imu), "imu_data");
  //
  //  rclc_publisher_init_default(
  //      &rpm_publisher, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  //
  //  rclc_subscription_init_default(
  //      &servo_subscriber, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
  //      "servo_move");
  //
  //  rclc_publisher_init_default(
  //      &locker_publisher, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
  //      "locker_status");
  //
  //  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(200),
  //  timer_callback);
  //
  //  rclc_executor_init(&executor, &support.context, 2, &allocator);
  //  rclc_executor_add_timer(&executor, &timer);
  //  rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_move,
  //                                 &servo_subscriber_callback, ON_NEW_DATA);
  //  rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg,
  //                                 &motor_subscriber_callback, ON_NEW_DATA);

  gpio_put(LED_PIN, true);

  int16_t servo_angle = 0;
  double speed = 5;
  Kinematics::rpm req_rpm = kinematics.getRPM(speed, 0, 0);

  while (true) {
    if (locker_state_up || locker_state_down) {
      if (servo_angle)
        servo_angle = 0;
      else
        servo_angle = 90;

      speed *= -1;

      req_rpm = kinematics.getRPM(speed, 0, 0);

      blink();
    }

    for (int i = 0; i < SERVO_COUNT; i++)
      servo_set_angle(i, servo_angle);

    motor1_controller((int)req_rpm.motor1);
    motor2_controller((int)req_rpm.motor2);
    motor3_controller((int)req_rpm.motor3);
    motor4_controller((int)req_rpm.motor4);

    //    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  return 0;
}
