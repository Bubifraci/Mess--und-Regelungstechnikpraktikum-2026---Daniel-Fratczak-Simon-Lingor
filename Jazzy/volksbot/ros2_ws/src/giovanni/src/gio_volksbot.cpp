#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "giovanni/gio_path.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class GiovanniVolksbot : public rclcpp::Node {
public:
    GiovanniVolksbot() : Node("gio_volksbot") {

        // ROS2 Param
        this->declare_parameter<std::string>("path", "quadrat.dat");
        std::string path_file = this->get_parameter("path").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading path file from: %s", path_file.c_str());

        gio_control = std::make_unique<CGioController>();
        
        if (!gio_control->getPathFromFile(path_file.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open GioPath File: %s", path_file.c_str());
        }

        gio_control->setCurrentVelocity(0.2);
        gio_control->setAxisLength(0.44);

        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/Vel_twist", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&GiovanniVolksbot::controlLoop, this)
        );

        subscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&GiovanniVolksbot::pose_callback, this, std::placeholders::_1)
        );
    }
  
    ~GiovanniVolksbot() {
        stopRobot();
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
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
        current_theta = yaw;
        current_x = x;
        current_y = y;
        pose_received = true;
    }

    void controlLoop() {
        if(!pose_received) {
            RCLCPP_INFO(this->get_logger(), "Warte auf AMCL Pose...");
            return;
        }
        double u = 0.0, omega = 0.0, vleft = 0.0, vright = 0.0;

        gio_control->setPose(current_x, current_y, current_theta);
        int status = gio_control->getNextState(u, omega, vleft, vright, 0);

        if (status == 0) {
            RCLCPP_INFO(this->get_logger(), "Target reached, stopping robot.");
            stopRobot();
            timer->cancel();
            return;
        }

        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = u;
        msg.angular.z = omega;

        velocity_publisher->publish(msg);
    }

    void stopRobot() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        velocity_publisher->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<CGioController> gio_control;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription;
    bool pose_received = false;

    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;

    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_theta = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GiovanniVolksbot>());
    rclcpp::shutdown();
    return 0;
}