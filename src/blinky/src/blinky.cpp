/*
    I want... a ROS node that subscribes to teleop and blinks an LED using the libgpiod library

    This will be your typical Hello World for GPIO control on the PI!

    Also, lets make it fade too! PWM!

*/


#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <gpiod.hpp>
//TODO:DS: Do I need to include the '-lgpiodcxx' flag for CMakeLists.txt somewhere? for the g++ compiler?


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        ::gpiod::chip chip("gpiochip0");
        RCLCPP_INFO(this->get_logger(), "Name: '%s' %c", chip.name().c_str(), isOn);

        this->publisher_->publish(message);

        auto line = chip.get_line(21);
        line.request({"example", gpiod::line_request::DIRECTION_OUTPUT, 0},1);

        if(isOn) {
          line.set_value(0);
        } else {
          line.set_value(1);
        }
        isOn = !isOn;
        line.release();
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  bool isOn = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
