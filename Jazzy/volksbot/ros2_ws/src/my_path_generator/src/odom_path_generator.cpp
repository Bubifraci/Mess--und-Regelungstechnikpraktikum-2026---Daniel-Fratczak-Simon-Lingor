#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class OdomPathGenerator : public rclcpp::Node
{
  public:
    OdomPathGenerator()
    : Node("odom_path_generator"), log_file_("pathDataOdom.txt")
    {
      if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN FILE! Check write permissions in current directory.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully created pathDataOdom.txt");
      }
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomPathGenerator::topic_callback, this, _1));
    }

  ~OdomPathGenerator()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

  private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      
      // Extract linear and angular velocity
      double vx = msg->twist.twist.linear.x;
      double vth = msg->twist.twist.angular.z;

      RCLCPP_INFO(this->get_logger(), 
        "Position -> x: %.2f, y: %.2f | Velocity -> vx: %.2f, vth: %.2f", 
        x, y, vx, vth);

        if (log_file_.is_open()) {
          log_file_ << x << " " << y << "\n";
          log_file_.flush();
        }
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::ofstream log_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPathGenerator>());
  rclcpp::shutdown();
  return 0;
}