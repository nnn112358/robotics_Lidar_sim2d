#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
using namespace std;


double myrobot_x;
double myrobot_y;
double myrobot_theta;
	
	
void mySigintHandler(int sig){
  printf("shutdown catch signal %d \n", sig);
  ros::shutdown();
}

static double get_dtime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long double n_time = tv.tv_sec + tv.tv_usec * 1e-6;
    static long double o_time = n_time;
    double dt_msec = (n_time - o_time);
    //cout<<dt_msec<<endl;
    o_time = n_time;
    return dt_msec;
}

void GetRPY(const geometry_msgs::Quaternion &quat, double &theta){
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	theta = yaw;
}

void odom_Callback(const nav_msgs::Odometry &odom){

	myrobot_x= odom.pose.pose.position.x;
	myrobot_y = odom.pose.pose.position.y;
	
	double roll, pitch, yaw;
        GetRPY(odom.pose.pose.orientation, yaw);
        myrobot_theta = yaw;
        
}


int main(int argc, char **argv){
  ros::init(argc, argv, "robot_sim_moveobst_publisher");

  ros::NodeHandle nh;
  double speed_gain;
  double turn_gain;
  int move_obst_num;


  double x_area_init;
  double y_area_init;


  ros::NodeHandle private_nh("~");
  private_nh.param("move_obst_num", move_obst_num,50);
  private_nh.param("speed_gain", speed_gain, 5.00);
  private_nh.param("turn_gain", turn_gain, 1.000);
  private_nh.param("x_area_init", x_area_init, 0.0);
  private_nh.param("y_area_init", y_area_init, 0.0);

  const std::string odom_topic = "odom_real";
 // ros::Subscriber odom_pos = nh.subscribe(odom_topic, 10, odom_Callback);  
  ros::Publisher move_obst_pub = nh.advertise<geometry_msgs::PoseArray>("/move_obst", 1);

  ros::Rate r(10.0);
  signal(SIGINT, mySigintHandler);

  cout << "moving....." << endl;

  double robot_theta[100];
  double robot_x[100];
  double robot_y[100];

  for (int i = 0; i < 100; i++){
    robot_theta[i] = ((double)rand() / ((double)RAND_MAX + 1)-0.5)* M_PI;
    robot_x[i] =((double)rand() / ((double)RAND_MAX + 1)-0.5)*x_area_init;
    robot_y[i] =((double)rand() / ((double)RAND_MAX + 1)-0.5)*y_area_init;
  }
  
  
  while (nh.ok()){
    double dt=get_dtime();
    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "/odom";
    //ROS_INFO_STREAM("poseArray.header: frame=" << poseArray.header.frame_id);

    geometry_msgs::PoseStamped pose;

    for (int i = 0; i < move_obst_num; i++){
      if(i%2==0)      robot_theta[i] += (((double)rand() / ((double)RAND_MAX + 1))-0.1) *turn_gain*dt;
      else      robot_theta[i] -= (((double)rand() / ((double)RAND_MAX + 1))-0.1) *turn_gain*dt;
     // robot_theta[i]%=2*M_PI;
     
  //   	double dis=(robot_x[i]-myrobot_x)*(robot_x[i]-myrobot_x)+(robot_y[i]-myrobot_y)*(robot_y[i]-myrobot_y);
//	if(dis<(0.5+0.5)){
		//
//		pose.pose.position.x = 0.0;
//		pose.pose.position.y =  0.0;
//		pose.pose.position.z = 0.0;
//		pose.pose.orientation = tf::createQuaternionMsgFromYaw( 0.0);
//		poseArray.poses.push_back(pose.pose);
	      
//	}
//	else{
	      double dis = ((double)rand() / ((double)RAND_MAX + 1));

	      robot_x[i] = dis * sin(-robot_theta[i]) * speed_gain*dt;
	      robot_y[i] = dis * cos(-robot_theta[i]) * speed_gain*dt;

	      pose.pose.position.x = -robot_y[i];
	      pose.pose.position.y = robot_x[i];
	      pose.pose.position.z = 0.0;
	      pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_theta[i]);
	      poseArray.poses.push_back(pose.pose);
	//}
      //  cout << "_" << pose.pose.position.x << "_" << pose.pose.position.y <<"_"<<robot_theta[i] << endl;
    }

    move_obst_pub.publish(poseArray);
    //ROS_INFO("poseArray size: %i", poseArray.poses.size()); //this outputs 92161

    ros::spinOnce();
    r.sleep();
  }

  //  ros::spin();

  return 0;
}
