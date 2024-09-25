/*
    I want... a ROS node that subscribes to teleop and blinks an LED using the libgpiod library

    This will be your typical Hello World for GPIO control on the PI!

    Also, lets make it fade too! PWM!

*/


#include <chrono>
#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MonsteraBlink : public rclcpp::Node
{
  public:
  MonsteraBlink() : Node("monstera"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("comm_led", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::Int32();
        message.data = count_++ % 2;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
  bool isOn = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonsteraBlink>());
  rclcpp::shutdown();
  return 0;
}
