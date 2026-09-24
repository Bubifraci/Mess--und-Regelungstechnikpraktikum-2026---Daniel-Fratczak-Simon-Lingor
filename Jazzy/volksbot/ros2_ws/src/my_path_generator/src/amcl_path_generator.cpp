#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
using std::placeholders::_1;

class AMCLPathGenerator : public rclcpp::Node
{
  public:
    AMCLPathGenerator()
    : Node("amcl_path_generator"), log_file_("pathDataAMCL.txt")
    {
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN FILE! Check write permissions in current directory.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully created pathDataAMCL.txt");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&AMCLPathGenerator::pose_callback, this, std::placeholders::_1));
    }

  ~AMCLPathGenerator()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

  private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      
      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      double timestamp = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);

      RCLCPP_INFO(this->get_logger(), "AMCL Position -> x: %.2f, y: %.2f, yaw: %.2f", x, y, yaw);

        if (log_file_.is_open()) {
          log_file_ << x << " " << y << " " << yaw << " " << timestamp << "\n";
          log_file_.flush();
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::ofstream log_file_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AMCLPathGenerator>());
  rclcpp::shutdown();
  return 0;
}