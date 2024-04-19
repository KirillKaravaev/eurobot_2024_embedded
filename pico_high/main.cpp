#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8_multi_array.h>

#include "locker.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.c" //только так (расширене не .h ,а .c) работает передача данных по юарт.
#include "stepper.h"

#define LED_PIN 25

bool locker_state_up = false;
bool locker_state_down = false;

bool flag = true;

rcl_subscription_t stepper_subscriber;
std_msgs__msg__UInt8MultiArray stepper_msg;

rcl_subscription_t screen_subscriber;
std_msgs__msg__String screen_data;

rcl_subscription_t locker_subscriber;
std_msgs__msg__UInt8MultiArray locker_status;

// Простейшая мигалка
void blink() {
  gpio_put(LED_PIN, flag);
  flag = !flag;
}

void stepper_subscriber_callback(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt8MultiArray *)msgin;
  //  stepper move
  blink();
}

void locker_subscriber_callback(const void *msgin) {
  const auto *msg = (const std_msgs__msg__UInt8MultiArray *)msgin;
  // set statuses and stop/start steps
  blink();
}

void screen_subscriber_callback(const void *msgin) {
  const auto *msg = (const std_msgs__msg__String *)msgin;
  // screen info here
  blink();
}

int main() {
  //  rmw_uros_set_custom_transport(
  //      true, nullptr, pico_serial_transport_open,
  //      pico_serial_transport_close, pico_serial_transport_write,
  //      pico_serial_transport_read);

  stepper_init();
  lockers_init();
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
  //  rclc_node_init_default(&node, "pico_high", "", &support);
  //
  //  rclc_subscription_init_default(
  //      &stepper_subscriber, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
  //      "stepper_topic");
  //
  //  rclc_subscription_init_default(
  //      &screen_subscriber, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "screen_topic");
  //
  //  rclc_subscription_init_default(
  //      &locker_subscriber, &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
  //      "locker_status");
  //
  //  rclc_executor_init(&executor, &support.context, 3, &allocator);
  //  rclc_executor_add_timer(&executor, &timer);
  //
  //  rclc_executor_add_subscription(&executor, &stepper_subscriber,
  //  &stepper_msg,
  //                                 &stepper_subscriber_callback, ON_NEW_DATA);
  //  rclc_executor_add_subscription(&executor, &locker_subscriber,
  //  &locker_status,
  //                                 &locker_subscriber_callback, ON_NEW_DATA);
  //  rclc_executor_add_subscription(&executor, &screen_subscriber,
  //  &screen_data,
  //                                 &screen_subscriber_callback, ON_NEW_DATA);

  gpio_put(LED_PIN, true);

  int32_t stepper_speed = 50;
  int8_t direction = 1;

  while (true) {

    if (locker_state_up || locker_state_down)
      direction *= -1;

    stepper(true, direction, stepper_speed);
    stepper(false, direction, stepper_speed);

    //    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  return 0;
}
