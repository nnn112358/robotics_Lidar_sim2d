#include "viewgui.h"
#include <math.h>
#include <iostream>
#include <limits>
using namespace std;
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>

//Robot制御のパラメータ
extern struct URG urg_data;
extern double input_x,input_y;

void mySigintHandler(int sig){
  printf("shutdown catch signal %d \n", sig);
  ros::shutdown();
}


int main(int argc, char* argv[]) {
	Graphics GL;

	ros::init(argc, argv, "n_GUI_joystick");
	ros::NodeHandle nh_;

	ros::Publisher cmd_vel_pub_;
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_cmd;

	ros::Rate r(20.0);

	double x_gain=0.3;
	double y_gain=0.3;
	if (!nh_.getParam("x_gain", x_gain))	x_gain=0.3;
	if (!nh_.getParam("y_gain", y_gain))	y_gain=0.3;
	signal(SIGINT, mySigintHandler);

	while(nh_.ok()){
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

		base_cmd.linear.x=input_y*x_gain;
		base_cmd.angular.z = -input_x*y_gain;

		cmd_vel_pub_.publish(base_cmd);
		cout<<"x:"<<base_cmd.linear.x<<"\t y:"<<base_cmd.angular.z <<"\n";

		r.sleep();

	}
	return 0;

}
