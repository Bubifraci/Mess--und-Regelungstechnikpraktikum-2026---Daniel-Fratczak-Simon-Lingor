#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include <string>

using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::max;
using std::min;

#include "giovanni/gio_path.h"
#include "rclcpp/rclcpp.hpp"

ofstream giofile;

double gio_u = 0.0;
double gio_omega = 0.0;
double gio_vleft = 0.0;
double gio_vright = 0.0;
int    drive_a_path = 0;


/*----------------------------------------------------------------*/
/*   Geschwindigkeiten und Positionsaenderungen berechnen         */
/*----------------------------------------------------------------*/
int PredictRobotBehaviour(double leftspeed, double rightspeed, 
                          double current_theta, double elapsed_time, 
                          double *dbeta, double *backDX, double *backDY)
{
  double rad_abstand = 0.440; // Achsabstand

  //Mittlere Bahngeschwindigkeit und Winkelgeschwindigkeiten
  double v = 0.5 * (leftspeed + rightspeed);
  double omega_sim = (rightspeed - leftspeed) / rad_abstand;

  *dbeta = omega_sim * elapsed_time;

  //Integration über den mittleren Winkel im Zeitschritt
  double mid_theta = current_theta + (*dbeta) * 0.5;

  *backDX = v * cos(mid_theta) * elapsed_time;
  *backDY = v * sin(mid_theta) * elapsed_time;

  return 0;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("gio_sim_path_node");
  node->declare_parameter<string>("path", "quadrat.dat");
  string path_file;
  node->get_parameter("path", path_file);
  RCLCPP_INFO(node->get_logger(), "Loading path file from: %s", path_file.c_str());
  /* #####################
   *  Prediction of robot
   * #################### */
  double leftspeed, rightspeed, v_diff;
  double dx = 0.0, dy = 0.0, dbeta;
  double scale_factor;
  double elapsed_time = 0.01; // 10 ms
  double epsilon = 1e-4;
  double x = 0, y = 0, theta = 0.0;

  double u, omega;
  double u_max = 1.0;

  
  CGioController *gio_control = new CGioController(); // object and also init function
  if (!gio_control->getPathFromFile(path_file.c_str()))
    RCLCPP_ERROR(node->get_logger(), "Can not open GioPath File: %s", path_file.c_str());
  else
    drive_a_path = 1;
  
  gio_control->setCurrentVelocity(u_max);
  gio_control->setAxisLength(0.440);

  // debugging
  giofile.open("pos.dat");

  //loop
  while  (drive_a_path) {
    //    gio_control->setPose(x * 0.001, y_from_encoder*0.001,theta_from_encoder);
    gio_control->setPose(x, y, theta);
    // get trajectory
    if (gio_control->getNextState(gio_u, gio_omega, leftspeed, rightspeed, 1)==0) {
	     cout<<"finish";
	     drive_a_path = 0;
    }
    
    // v_diff = gio_omega * gio_u / M_PI;
    leftspeed = (float) (gio_u - gio_omega * gio_control->getAxisLength() * 0.5); // - fabs(v_diff) - v_diff); 
    rightspeed = (float) (gio_u + gio_omega * gio_control->getAxisLength() * 0.5); // fabs(v_diff) + v_diff); 

    scale_factor = 1.0;
    if (fabs(leftspeed) > u_max) scale_factor = fabs(u_max / leftspeed);
    if (fabs(rightspeed) > u_max) scale_factor = fabs(u_max / rightspeed);
    leftspeed *= scale_factor;
    rightspeed *= scale_factor;


    
    // SET SPEED HERE =====================================
    /*
    set_wheel_speed2(v_l_soll, v_r_soll,
				 v_l_ist, v_r_ist,
				 omega, Get_mtime_diff(9), AntiWindup);
    */
    
    /* ######################################
	*  Implementation of the robot simulator
	* ###################################### */
    PredictRobotBehaviour(leftspeed, rightspeed, theta, elapsed_time, &dbeta, &dx, &dy);  
    x += dx;
    y += dy;
    theta += dbeta;

    //Normalisiere theta innerhalb von 180°
    while (theta > M_PI)  theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;
    //gio_control->getRoboterPose(leftspeed, rightspeed, x, y, theta);
  
    giofile << x << " " << y << " "
		  << theta << " " << gio_u << " " << gio_omega << " "
		  << leftspeed << " " << rightspeed << endl;
    RCLCPP_INFO(node->get_logger(), 
        "Pose -> x: %.2f, y: %.2f, theta: %.2f | Velocity -> u: %.2f, omega: %.2f, leftspeed: %.2f, rightspeed: %.2f\n", 
        x, y, theta, gio_u, gio_omega, leftspeed, rightspeed);
    cout.flush();
    giofile.flush();

  }
  giofile.close();
  giofile.clear();
  delete gio_control;
  rclcpp::shutdown();
  return 0;

}
