#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

//creating node class by inheriting from rclcpp::Node
class MinimalPublisher : public rclcpp::Node
{
  public:
    //Constructor erbt von Node und benennt node "minimal_publisher", inits count_ to 0
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
    //creating publisher with message type String and topic "topic", queue size 10, saved in var publisher_
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      //creating a timer that triggers the timer_callback functions every 500ms
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        //creating the message variable of type String
      auto message = std_msgs::msg::String();
      //writing the message
      message.data = "Hello, world! " + std::to_string(count_++);
      //Getting the logger and writing the message into the console as a string
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //Publishing the message
      publisher_->publish(message);
    }
    //declaring the fields timer_, publisher_ and count_
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    //initializing ros 2
  rclcpp::init(argc, argv);
  //start processing data from the node
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
