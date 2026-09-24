//
// STANDALONE TEST g++ -o odometrie odometrie.cc -lm -Wall
//
 
/***************************************************************************
 * INCLUDED HEADERS
 ***************************************************************************/
#include <iostream>
#include <fstream>
using std::ofstream;
using std::cout;
using std::endl;
#include <cmath>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"


/***************************************************************************
 * CONSTANTS & MACROS
 ***************************************************************************/  
#define FALSE                  0
#define TRUE                   1
#define INVPIMUL180            57.29578
#define SQR(a)                 ((a)*(a))          


const double rad_abstand = 0.485;          // in Meter
const double epsilon = 1e-4;
const double elapsed_time = 0.01;        // 10 ms


/***************************************************************************
 * FUNCTION CalcNextVelocities input robot pos x, y, and orientation
 * phi goal position and orientation is allways zero (0,0,0) so
 * transform your coordinate system according to the goal the output
 * is u (linear velocity) and omega angular velocity u could be to
 * large, so it is limited to u_max maximum speed
 ***************************************************************************/  
int CalcNextVelocities(double u_max, double x, double y,
				   double phi,
				   double *u, double *omega)
{
  double c, theta;
  double alpha;
  double e;
  // h > 1; 2 < beta < h + 1 according to lyaponv criteria
  double gamma = 1.0, beta = 2.9, h = 2.0; // has to be choosen from eq 17
  double epsilon = 1e-4;
  
  e = sqrt(SQR(x) + SQR(y)); // distance to goal
  
  if (e > epsilon) {
    theta = atan2(-y, -x);    // orientation to goal
    if (theta >  M_PI ) theta -= 2.0*M_PI ;
    if (theta < -M_PI)  theta += 2.0*M_PI;
    
    alpha = theta - phi;     // diff steering angle and orientation
    if (alpha >  M_PI ) alpha -= 2.0 * M_PI ;
    if (alpha < -M_PI ) alpha += 2.0 * M_PI ;
    
    *u = gamma * e;                                      // compute the velocity
    if (*u >= u_max) {
	 *u = u_max;
    }

    if (fabs(alpha) > epsilon) { 
      c = (sin(alpha) + (h * theta * sin(alpha) / alpha + beta * alpha)) / e; 
    }
    else {
      c = (alpha + alpha * beta + h * theta) / e;
    }
    *omega = c * *u ;                                  // compute the turning velocity
  }
  else {                                               // stop, iff we are there
    *u = 0.0;
    *omega = 0.0;
  }
  return 0;
}


/*----------------------------------------------------------------*/
/*   Geschwindigkeiten und Positionsaenderungen berechnen         */
/*----------------------------------------------------------------*/
int PredictRobotBehaviour(const double leftspeed, const double rightspeed, 
                          const double theta,
					 const double elapsed_time, 
                          double &dtheta, double &dx, double &dy)
{
  double v = 0.0;

  if (fabs(rightspeed) < epsilon) {

    if (fabs(leftspeed) < epsilon) {
	 dtheta = 0.0;
	 dx = 0.0;
	 dy = 0.0;
	 return(2);
    }
  }

  v = 0.5 * (leftspeed + rightspeed);
  dtheta = elapsed_time * (rightspeed - leftspeed) / rad_abstand;
  
  dx = v * cos(theta + dtheta * 0.5) * elapsed_time;
  dy = v * sin(theta + dtheta * 0.5) * elapsed_time;
  
  return(0);
} 

int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("odometrie_sim");
  double x = -1.0, y = -1.0, theta = -1.5 ;
  double u, omega;
  double u_max = 0.35;

  /* #####################
   *  Prediction of robot
   * #################### */
  double leftspeed, rightspeed;
  double dx = 0.0, dy = 0.0, dtheta;
  double scale_factor;
  double epsilon = 0.01;
  int    i = 0, nr = 10000;

  ofstream out("pos.dat");
  
  do {
    //  ####################################
    //  Implementatierung des Regelgesetzes
    //  #################################### 
    CalcNextVelocities(u_max, x, y, theta, &u, &omega);

    //u = 0.1;
    //omega = 0.5;
    
    leftspeed  = (u - omega * rad_abstand * 0.5);
    rightspeed = (u + omega * rad_abstand * 0.5); 

    scale_factor = 1.0;
    if (fabs(leftspeed) > u_max) scale_factor = fabs(u_max / leftspeed);
    if (fabs(rightspeed) > u_max) scale_factor = fabs(u_max / rightspeed);
    leftspeed  *= scale_factor;
    rightspeed *= scale_factor;

    /*
    if (leftspeed  < 0) leftspeed  = 0.0;
    if (rightspeed < 0) rightspeed = 0.0;
    */
    
    /* ######################################
	*  Implementation of the robot simulator
	* ###################################### */
    PredictRobotBehaviour(leftspeed, rightspeed,
					 theta,
					 elapsed_time,
					 dtheta, dx, dy);  

    x += dx;
    y += dy;
    theta += dtheta;
    
    RCLCPP_INFO(logger, 
        "Robot Sim Iteration -> i: %d | Pose -> x: %.2f, y: %.2f, theta: %.2f | Velocity -> u: %.2f, omega: %.2f, leftspeed: %.2f, rightspeed: %.2f\n", 
        i, x, y, theta, u, omega, leftspeed, rightspeed);

    out << i << " "
	   << x << " "
	   << y << " "
	   << theta << " "
	   << u << " "
	   << omega << " "
	   << leftspeed << " "
	   << rightspeed
	   << endl;
    
  } while (((fabs(x) > epsilon) || (fabs(y) > epsilon)) && (i++ < nr) );
  rclcpp::shutdown();
  return 0;
}






