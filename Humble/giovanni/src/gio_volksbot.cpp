#include <string>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#define VB_NO_GEOMETRY

#include "volksface/volksbot.h"
#include "giovanni/gio_path.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class GiovanniVolksbot : public rclcpp::Node {
public:
    GiovanniVolksbot() : Node("giovanni_volksbot") {
        this->declare_parameter<std::string>("path", "path.dat");
        std::string path_file = this->get_parameter("path").as_string();

        RCLCPP_INFO(this->get_logger(), "Loading path file from: %s", path_file.c_str());

        this->declare_parameter<double>("speed", 0.2);
        double speed = this->get_parameter("speed").as_double();

        gio_control = std::make_unique<CGioController>();
        
        if (!gio_control->getPathFromFile(path_file.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open GioPath File: %s", path_file.c_str());
        }

        gio_control->setCurrentVelocity(speed);
        gio_control->setAxisLength(0.44);

        vel_publisher = this->create_publisher<VB::msg::VelGP>(VB::TOPIC_NAME_VEL_GP, 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(5),
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
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;
      
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

        if (!pose_received) {
            pose_received = true;
            gio_control->transformPath(current_x, current_y, current_theta);
            gio_control->setPose(current_x, current_y, current_theta);
            RCLCPP_INFO(this->get_logger(), "Pfad erfolgreich an AMCL-Startpose ausgerichtet!");
        }
    }

    void controlLoop() {
        if (!pose_received) {
            RCLCPP_INFO(this->get_logger(), "Warte auf AMCL Pose...");
            return;
        }

        double u = 0.0, omega = 0.0, vleft = 0.0, vright = 0.0;

        gio_control->setPose(current_x, current_y, current_theta);
        int status = gio_control->getNextState(u, omega, vleft, vright, 0);

        if (status == 0) {
            RCLCPP_INFO(this->get_logger(), "Ziel erreicht / Pfad beendet. Stoppe Roboter.");
            stopRobot();
            timer->cancel();
            return;
        }

        auto msg = VB::msg::VelGP();
    
        // Radgeschwindigkeiten von m/s nach cm/s umrechnen
        msg.left  = vleft * 100.0;
        msg.right = vright * 100.0;

        vel_publisher->publish(msg);
    }

    void stopRobot() {
        auto msg = VB::msg::VelGP();
        msg.left  = 0.0;
        msg.right = 0.0;
        vel_publisher->publish(msg);
    }

    rclcpp::Publisher<VB::msg::VelGP>::SharedPtr vel_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<CGioController> gio_control;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription;
    bool pose_received = false;

    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GiovanniVolksbot>());
    rclcpp::shutdown();
    return 0;
}