#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.c"  //только так (расширене не .h ,а .c) работает передача данных по юарт. 
#include "motors.h"
#include "kinematics.h"
#include "rpm.h"


#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>

rpm current_rpm;
imp_num current_imp_num;
struct repeating_timer timer;

const uint LED_PIN = 25;
int cnt = 1;

//rcl_publisher_t publisher;
//geometry_msgs__msg__Twist pub_twist_msg;

rcl_publisher_t rpm_publisher;
geometry_msgs__msg__Quaternion rpm_msg;

//rcl_subscription_t subscriber;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

//rcl_publisher_t imu_publisher;
//sensor_msgs__msg__Imu imu_msg;



Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    #ifdef  BRUSH_MOTORS 
	MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE,
	#endif 
	#ifdef	BRUSHLESS_MOTORS
	MAX_RPM,
	#endif
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);



Kinematics::velocities cmd_vel;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	
	
//	pub_twist_msg.linear.x = cmd_vel.linear_x;
//	pub_twist_msg.linear.y = cmd_vel.linear_y;
//	pub_twist_msg.angular.z = cmd_vel.angular_z;
	
	
//	rcl_ret_t ret = rcl_publish(&publisher, &pub_twist_msg, NULL);
    
    
    rpm_msg.w = current_rpm.rpm1;
    rpm_msg.x = current_rpm.rpm2;
    rpm_msg.y = current_rpm.rpm3;
    rpm_msg.z = current_rpm.rpm4;
  
    rcl_ret_t ret = rcl_publish(&rpm_publisher, &rpm_msg, NULL);
}


void subscriber_callback(const void * msgin)
{

const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

//Kinematics::velocities cmd_vel;
cmd_vel.linear_x = msg->linear.x;
cmd_vel.linear_y = msg->linear.y;
cmd_vel.angular_z = msg->angular.z;

Kinematics::rpm req_rpm = kinematics.getRPM(
        msg->linear.x, 
        msg->linear.y, 
        msg->angular.z
    );


//Kinematics::rpm rpm; // Объявляем название структуры rpm внутри класса Kinematics. Операцией объявления области видимости :: мы как бы смотрим изнутри класса
//int motor1_rpm = rpm.motor1; //и тем самым видим уже его элементы, включая искомую структуру rpm. И уже по месту даем ей название rpm, а дальше по этому названию
motor1_controller((int)req_rpm.motor1);//обращаемся к элементу motor1
//int motor2_rpm = rpm.motor2;
motor2_controller((int)req_rpm.motor2);
//int motor3_rpm = rpm.motor3;
motor3_controller((int)req_rpm.motor3);
//int motor4_rpm = rpm.motor4;
motor4_controller((int)req_rpm.motor4);

//Простейшая мигалка
if(cnt == 1){  
	gpio_put(LED_PIN, 1);  
	cnt = 0;
}
else if(cnt == 0){
	gpio_put(LED_PIN, 0);  
	cnt = 1;
}

}


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    motors_init();
    impulse_counter_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");


    
    //rclc_publisher_init_default(
    //    &publisher,
    //    &node,
    //    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //    "pico_publisher");

    rclc_publisher_init_default(
        &rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
        "rpm_topic");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

 //    std_msgs__msg__String__init(&sub_msg);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
//    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscriber_callback, ON_NEW_DATA);

    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &subscriber_callback, ON_NEW_DATA);
 //   gpio_put(LED_PIN, 1);

//    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
