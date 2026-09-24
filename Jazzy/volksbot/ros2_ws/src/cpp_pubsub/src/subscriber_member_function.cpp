#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  // constructor inheriting from Node, naming node "minimal_subscriber"
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    // creating subscription of type String, subscribing to topic "topic", queue size 10
    // binding the "topic_callback" function to the node
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  // receives the message published on "topic" and prints it to the console
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  // defining the fileds
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  // init ros2
  rclcpp::init(argc, argv);
  // start processing data from the node
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}