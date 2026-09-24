#include <memory>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
using std::placeholders::_1;

class PathLoggerNode : public rclcpp::Node
{
public:
    // creating a Node called "path_logger_node"
    PathLoggerNode() : Node("path_logger_node")
    {
        outFile_.open("odomXYData.txt");
        if (!outFile_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening file for writing!");
        }

        // creating subscription of type Odometry, subscribing to topic "odom", queue size 10
        // binding the "topic_callback" function to the node
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&PathLoggerNode::odom_callback, this, _1)
        );
    }

    // Close the file in the destructor to ensure clean resource management
    ~PathLoggerNode()
    {
        if (outFile_.is_open()) {
            outFile_.close();
        }
    }

private:
    // receives the message published on "odom" and prints it to the console
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Position: x=%.3f y=%.3f z=%.3f",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );

        outFile_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";
    }

    // definig the necessairy fields
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::ofstream outFile_;
};

int main(int argc, char *argv[])
{
    // init ros2
    rclcpp::init(argc, argv);
    // start processing data from PathLoggerNode
    rclcpp::spin(std::make_shared<PathLoggerNode>());
    rclcpp::shutdown();

    return 0;
}