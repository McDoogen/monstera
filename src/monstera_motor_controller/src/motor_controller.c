#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transport.h"


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

void stall_motor(uint pin1, uint pin2) {
    gpio_put(pin1, 0);
    gpio_put(pin2, 0);
}

void spin_motor(uint pin1, uint pin2, bool direction) {
    if(direction) {
        gpio_put(pin1, 0);
        gpio_put(pin2, 1);
    } else {
        gpio_put(pin1, 1);
        gpio_put(pin2, 0);
    }

}

void brake_motor(uint pin1, uint pin2) {
    gpio_put(pin1, 1);
    gpio_put(pin2, 1);
}

int main() {

    // Micro ROS Setup
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);


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
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // LED Status Pin Setup
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // H-Bridge Pin Setup
    const uint AIN1_PIN = 8;
    gpio_init(AIN1_PIN);
    gpio_set_dir(AIN1_PIN, GPIO_OUT);

    const uint AIN2_PIN = 7;
    gpio_init(AIN2_PIN);
    gpio_set_dir(AIN2_PIN, GPIO_OUT);

    const uint BIN1_PIN = 9;
    gpio_init(BIN1_PIN);
    gpio_set_dir(BIN1_PIN, GPIO_OUT);

    const uint BIN2_PIN = 10;
    gpio_init(BIN2_PIN);
    gpio_set_dir(BIN2_PIN, GPIO_OUT);

    const uint PWMA_PIN = 6;
    gpio_set_function(PWMA_PIN, GPIO_FUNC_PWM);
    uint sliceA = pwm_gpio_to_slice_num(PWMA_PIN);
    uint chanA = pwm_gpio_to_channel(PWMA_PIN);
    pwm_set_wrap(sliceA, 100);    //MAX: 65535
    pwm_set_chan_level(sliceA, chanA, 0);
    pwm_set_enabled(sliceA, true);

    const uint PWMB_PIN = 11;
    gpio_set_function(PWMB_PIN, GPIO_FUNC_PWM);
    uint sliceB = pwm_gpio_to_slice_num(PWMB_PIN);
    uint chanB = pwm_gpio_to_channel(PWMB_PIN);
    pwm_set_wrap(sliceB, 100);    //MAX: 65535
    pwm_set_chan_level(sliceB, chanB, 0);
    pwm_set_enabled(sliceB, true);

    msg.data = 0;
    while (true) {
    /*
        Better Motor Test:
        - Stall 3 seconds
        - Spin CW accelerating up to max
        - Slow down to brake
        - Brake 3 seconds
        - Same but CCW


        Is there any difference between stall and brake? And what if I set the PWM to zero?

        //TODO:DS: How do I read the hall effect sensor?
        - Interrupt on the rising edges?
        - 90 degree phase difference?
        - Go ahead and make this a ROS thing now
        - Depending on the order of the signals tells you the direction it is spinning
        - Each count of theedge pulse is a revolution
        - Measure the time since the previous revolution, RPM
        - Gear ratio is either 1:20 or... 244904:12000.... ?
        - Look into C Structs to organize this better. And lets move some of this to headers to reuse on other projects! :D


    */
        // Stall 3 seconds
        gpio_put(LED_PIN, 0);
        stall_motor(AIN1_PIN, AIN2_PIN);
        sleep_ms(3000);

        // Spin 3 seconds
        gpio_put(LED_PIN, 1);
        spin_motor(AIN1_PIN, AIN2_PIN, 0);
        sleep_ms(3000);

        // Brake 3 seconds
        gpio_put(LED_PIN, 0);
        brake_motor(AIN1_PIN, AIN2_PIN);
        sleep_ms(3000);

        // Spin 3 seconds, in the opposite direction
        gpio_put(LED_PIN, 1);
        spin_motor(AIN1_PIN, AIN2_PIN, 1);
        sleep_ms(3000);

        //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); TODO:DS: ?
    }
}