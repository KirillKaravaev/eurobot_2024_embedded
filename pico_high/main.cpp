#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.c" //только так (расширене не .h ,а .c) работает передача данных по юарт.
#include "stepper.h"

#include <geometry_msgs/msg/vector3.h>

const uint LED_PIN = 25;
int cnt = 1;

rcl_subscription_t stepper_subscriber;
geometry_msgs__msg__Vector3 stepper_msg;

// Простейшая мигалка
void blink() {
  if (cnt == 1) {
    gpio_put(LED_PIN, 1);
    cnt = 0;
  } else if (cnt == 0) {
    gpio_put(LED_PIN, 0);
    cnt = 1;
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

  //	pub_twist_msg.linear.x = cmd_vel.linear_x;
  //	pub_twist_msg.linear.y = cmd_vel.linear_y;
  //	pub_twist_msg.angular.z = cmd_vel.angular_z;

  //	rcl_ret_t ret = rcl_publish(&publisher, &pub_twist_msg, NULL);

  //    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  //    msg.data++;
}

void stepper_subscriber_callback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg =
      (const geometry_msgs__msg__Vector3 *)msgin;
  stepper(msg->x, msg->y, msg->z);
  blink();
}

int main() {
  //  rmw_uros_set_custom_transport(
  //      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
  //      pico_serial_transport_write, pico_serial_transport_read);

  stepper_init();
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  while (true) {

    stepper(1, 1, 100);
    stepper(0, 1, 100);
  }

  rcl_timer_t timer;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    return ret;
  }

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "pico_node", "", &support);

  rclc_subscription_init_default(
      &stepper_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "stepper_topic");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  rclc_executor_add_subscription(&executor, &stepper_subscriber, &stepper_msg,
                                 &stepper_subscriber_callback, ON_NEW_DATA);
  gpio_put(LED_PIN, 1);

  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  return 0;
}