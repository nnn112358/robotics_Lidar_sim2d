
#include "gl_view.h"
#include <math.h>
#include <iostream>
#include <limits>
using namespace std;
#include <ctime>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
extern struct ROBOT myrobot;
extern struct LRF lrf_data;
extern struct MOVE_OBSTACLE move_obst;


extern string gl_map_fname;

void GetRPY(const geometry_msgs::Quaternion &quat, double &theta){
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	theta = yaw;
}
void mySigintHandler(int sig){
	printf("shutdown catch signal %d \n", sig);
	ros::shutdown();
}

void move_obst_Callback(const geometry_msgs::PoseArray &msg){
	for (int i = 0; i < move_obst.n; i++){
		double my_x=move_obst.x[i];
		double my_y=move_obst.y[i];
		double my_theta=move_obst.theta[i] ;

		my_x += msg.poses[i].position.x;
		my_y += msg.poses[i].position.y;
		double roll, pitch, yaw;
		GetRPY(msg.poses[i].orientation, yaw);
		my_theta += yaw;

		double dis=sqrt((my_x-myrobot.x)*(my_x-myrobot.x)+(my_y-myrobot.y)*(my_y-myrobot.y));
		if(dis<(move_obst.obst_size+myrobot.size+1.50)){
			//衝突しそうな場合は回避する
			double aa=(my_y-myrobot.y)/(my_x-myrobot.x+0.000001);
			double direc=atan(1.0/aa);
			my_x+=0.25*sin(-direc);//-msg.poses[i].position.x;
			my_y+=0.25*cos(-direc);//-msg.poses[i].position.y;
			my_theta-=0.1*direc-yaw;

			move_obst.x[i]=my_x;
			move_obst.y[i]=my_y;
			move_obst.theta[i] =my_theta;
			ROS_WARN("alert");
		}
		else{
			move_obst.x[i]=my_x;
			move_obst.y[i]=my_y;
			move_obst.theta[i] =my_theta;
		}
	}
}

void vel_Callback(const geometry_msgs::Twist &vel_msg){
	myrobot.v = vel_msg.linear.x;
	myrobot.w = -vel_msg.angular.z;
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, "n_robot_sim");
	ros::NodeHandle private_nh("~");

	double LRF_data_num,LRF_deg,LRF_len_max,LRF_len_min,LRF_leng_stdev;
	private_nh.param("LRF_data_num", LRF_data_num, 768.0);
	private_nh.param("LRF_deg", LRF_deg, 240.0);

	private_nh.param("LRF_len_max", LRF_len_max, 10.0);
	private_nh.param("LRF_len_min", LRF_len_min, 0.2);
	private_nh.param("LRF_leng_stdev", LRF_leng_stdev, 0.01);
	lrf_data.data_num = LRF_data_num;
	lrf_data.start_rad = -LRF_deg / 2.0 / 360.0 * 2 * M_PI;
	lrf_data.reso = LRF_deg / LRF_data_num / 360.0 * 2 * M_PI;
	lrf_data.leng_max = LRF_len_max;
	lrf_data.leng_min = LRF_len_min;
	lrf_data.leng_stdev = LRF_leng_stdev;


	double odom_noise_liner,odom_noise_liner_stdev,odom_noise_angle,odom_noise_angle_stdev;
	private_nh.param("odom_noise_liner", odom_noise_liner, 0.05);
	private_nh.param("odom_noise_liner_stdev", odom_noise_liner_stdev, 1.0);
	private_nh.param("odom_noise_angle", odom_noise_angle, 0.05);
	private_nh.param("odom_noise_angle_stdev", odom_noise_angle_stdev, 1.0);
	myrobot.odom_noise_liner=odom_noise_liner;
	myrobot.odom_noise_liner_stdev=odom_noise_liner_stdev;
	myrobot.odom_noise_angle=odom_noise_angle;
	myrobot.odom_noise_angle_stdev=odom_noise_angle_stdev;

	string map_fname;
	private_nh.param("map_fname", map_fname, std::string("./obstacle.csv"));
	gl_map_fname=map_fname;

	double sim_fps;
	private_nh.param("sim_fps", sim_fps, 40.0);

	int move_obst_num = 0;
	private_nh.param("move_obst_num", move_obst_num, 0);
	move_obst.n = move_obst_num;

	double move_obst_size = 0.0;
	private_nh.param("move_obst_size", move_obst_size, 0.40);
	move_obst.obst_size = move_obst_size;


	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Publisher odom_pub_r = n.advertise<nav_msgs::Odometry>("odom_real", 10);

	tf::TransformBroadcaster odom_broadcaster;

	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);
	const std::string vel_topic_ = "cmd_vel";
	ros::Subscriber cmd_vel = n.subscribe(vel_topic_, 10, vel_Callback);
	ros::Subscriber move_obst_cb = n.subscribe("move_obst", 10, move_obst_Callback);

	//scan
	unsigned int num_readings = lrf_data.data_num;
	double laser_frequency = sim_fps;
	double ranges[num_readings];
	double intensities[num_readings];

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	Graphics GL;
	usleep(100*1000);
	signal(SIGINT, mySigintHandler);
	ros::Rate r(sim_fps);

	while (n.ok())	{
		ros::spinOnce();

		current_time = ros::Time::now();

		//double theta;
		//GetRPY(odom_quat, theta);

		//tf odom->base_link
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = myrobot.y_dummy;
		odom_trans.transform.translation.y = -myrobot.x_dummy;
		odom_trans.transform.translation.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(myrobot.theta_dummy);
		odom_trans.transform.rotation = odom_quat;
		odom_broadcaster.sendTransform(odom_trans);

		//tf map->odom_true
		geometry_msgs::TransformStamped odom_trans2;
		odom_trans2.header.stamp = current_time;
		odom_trans2.header.frame_id = "map";
		odom_trans2.child_frame_id = "odom_true";

		geometry_msgs::Quaternion odom_quat_r = tf::createQuaternionMsgFromYaw(myrobot.theta);
		odom_trans2.transform.translation.x = myrobot.y;
		odom_trans2.transform.translation.y = -myrobot.x;
		odom_trans2.transform.translation.z = 0.0;
		odom_trans2.transform.rotation = odom_quat_r;
		odom_broadcaster.sendTransform(odom_trans2);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "map";
		odom.child_frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = myrobot.y_dummy;
		odom.pose.pose.position.y = -myrobot.x_dummy;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		//set the velocity
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = myrobot.v;
		odom.twist.twist.angular.z = myrobot.w;

		//publish the message
		odom_pub.publish(odom);


		nav_msgs::Odometry odom_r;
		odom_r.header.stamp = current_time;
		odom_r.header.frame_id = "map";
		odom_r.child_frame_id = "odom_real";

		//set the position
		odom_r.pose.pose.position.x = myrobot.y;
		odom_r.pose.pose.position.y = -myrobot.x;
		odom_r.pose.pose.position.z = 0.0;
		odom_r.pose.pose.orientation = odom_quat_r;
		//set the velocity
		odom_r.twist.twist.linear.x = 0.0;
		odom_r.twist.twist.linear.y = myrobot.v;
		odom_r.twist.twist.angular.z = myrobot.w;

		odom_pub_r.publish(odom_r);

		//populate the LaserScan message
		sensor_msgs::LaserScan scan;
		scan.header.stamp = current_time;
		scan.header.frame_id = "base_link";
		scan.angle_min = lrf_data.start_rad;
		scan.angle_max = lrf_data.start_rad + lrf_data.reso * lrf_data.data_num;
		scan.angle_increment = lrf_data.reso;
		scan.time_increment = (1 / laser_frequency) / (lrf_data.data_num);
		scan.range_min = lrf_data.leng_min;
		scan.range_max = lrf_data.leng_max;

		scan.ranges.resize(num_readings);
		scan.intensities.resize(num_readings);
		for (unsigned int i = 0; i < num_readings; ++i){
			if (lrf_data.leng[i] < 0.01){
				scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
			}
			else{
				scan.ranges[i] = lrf_data.leng[i];
			}
			scan.intensities[i] = 0.0;
		}

		scan_pub.publish(scan);

		last_time = current_time;
		r.sleep();
	}

	return 0;
}
