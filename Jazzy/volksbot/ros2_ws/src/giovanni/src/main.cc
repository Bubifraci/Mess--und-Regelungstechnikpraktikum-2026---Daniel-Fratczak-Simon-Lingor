#include <string>
#include <memory>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <volksface/msg/vel_gp.hpp>

#include <giovanni/gio_path.h>

// Zeiteinheiten wie Sekunden und Millisekunden
using namespace std::chrono_literals;

class GioNode : public rclcpp::Node {
public:
  GioNode() : Node("gio_node"){
    // Startposition auf (0,0,0) setzten
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_theta_ = 0.0;

    // ROS2 Variable wo das Pfadfile gespeichert ist
    this->declare_parameter<std::string>("path", "circle.dat");
    std::string path_file = this->get_parameter("path").as_string();

    this->declare_parameter<double>("speed", 1);
    double usr_speed = this->get_parameter("speed").as_double();

    // Pfad in Console ausgeben
    RCLCPP_INFO(this->get_logger(), "Loading path file from: %s", path_file.c_str());

    // Vmax und Axenlänge festlegen
    controller_.setCurrentVelocity(usr_speed);
    controller_.setAxisLength(0.44);

    // Controller-Pfad laden
    if (!controller_.getPathFromFile(path_file.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Pfad konnte nicht geladen werden!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Pfad erfolgreich geladen.");
    }

    // ROS2 Publisher & Subscriber
    // Kommandos auf topic publishen, damit sich Roboter bewegt
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/Vel_twist", 10);
    // Roboterdaten einlesen
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GioNode::odomCallback, this, std::placeholders::_1)
    );
    // Gamepad publisher für Räder
    pub_gp_ = this->create_publisher<volksface::msg::VelGP>("/Vel_gp", 10);

    // Regler alle 5ms ausführen (200Hz)
    timer_ = this->create_wall_timer(
      5ms, std::bind(&GioNode::timerCallback, this)
    );
  }

  ~GioNode(){
    stopRobot();
  }

private:
  //Speichert die relevanten odom Werte, sobald diese ankommen
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Pose aus odom rauslesen
    double temp_x = msg->pose.pose.position.x;
    double temp_y = msg->pose.pose.position.y;
    
    // Quaternion auslesen
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );

    // Umwandeln des Quaternions q in Winkel theta
    double roll, pitch, temp_theta;
    tf2::Matrix3x3(q).getRPY(roll, pitch, temp_theta);

    if(!got_first_odom_){
      start_x_ = temp_x;
      start_y_ = temp_y;
      start_theta_ = temp_theta;

      RCLCPP_INFO(
        this->get_logger(),
        "Startpose gesetzt auf x=%.2f, y=%.2f, theta=%.2f",
        start_x_, start_y_, start_theta_
      );

      got_first_odom_ = 1;
    }

    // Differenz der current pos zur start pos
    double dx = temp_x - start_x_;
    double dy = temp_y - start_y_;

    // Rotation des Translationsvektors um -start_theta_ -> Blick entlang der x-Achse
    current_x_ = dx * cos(-start_theta_) - dy * sin(-start_theta_);
    current_y_ = dx * sin(-start_theta_) + dy * cos(-start_theta_);

    current_theta_ = temp_theta - start_theta_;

    // Normalisieren des Winkels
    while (current_theta_ > M_PI) current_theta_ -= 2.0 * M_PI;
    while (current_theta_ < -M_PI) current_theta_ += 2.0 * M_PI;
  }

  // Führt den Regler aus
  void timerCallback() {

    if(!got_first_odom_){
      return;
    }

    // (Transformierte) Position an Controller übergeben
    controller_.setPose(current_x_, current_y_, current_theta_);

    // Nächste Geschwindigkeiten berechnen
    double u = 0.0, omega = 0.0, vl = 0.0, vr = 0.0;
    int status = controller_.getNextState(u, omega, vl, vr, 0);

    if (status == 0) {
      RCLCPP_INFO(this->get_logger(), "Target reached, stopping robot.");
      stopRobot();
      timer_->cancel();
      return;
    }

    // Befehl an Roboter senden
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = u;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = omega;
    
    /*
    pub_->publish(cmd);
    */

    auto vel_msg = volksface::msg::VelGP();
    vel_msg.left = static_cast<float>(vr * 100);
    vel_msg.right = static_cast<float>(vl * 100);
    pub_gp_->publish(vel_msg);

    // Debug-Ausgabe im Terminal
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "Sende: u=%.2f, omega=%.2f | Pose: x=%.2f, y=%.2f, th=%.2f | Räder: L=%.2f, R=%.2f", 
      cmd.linear.x, cmd.angular.z, current_x_, current_y_, current_theta_, vel_msg.left, vel_msg.right);
  }

  // Stoppen sobald ende
  void stopRobot() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    pub_->publish(msg);

    // Räder stoppen
    auto vel_msg = volksface::msg::VelGP();
    vel_msg.left = 0.0f;
    vel_msg.right = 0.0f;
    pub_gp_->publish(vel_msg);
  }

  CGioController controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<volksface::msg::VelGP>::SharedPtr pub_gp_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Speichern der Startpositionen
  double start_x_;
  double start_y_;
  double start_theta_;

  // Speichern der aktuell relevanten odom Daten des Roboters
  double current_x_;
  double current_y_;
  double current_theta_;
  int got_first_odom_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GioNode>());
  rclcpp::shutdown();
  return 0;
}
