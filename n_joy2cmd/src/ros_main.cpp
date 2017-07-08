#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <vector>

using namespace std;

#include <signal.h>

int linear_ch = 0, angular_ch = 1;
double l_scale = 0.0, a_scale = 0.0;
//int button_ch;
int button_ch1 = 0;
int button_ch2 = 1;
int button_ch3 = 2;
int button_ch4 = 3;
int button_ch5 = 4;
int button_ch6 = 5;
ros::Publisher twist_pub_;

class JoyStickOp{
public:
  JoyStickOp();
  void twist_send();
  ros::NodeHandle nh_;
  static void mySigintHandler(int);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void vel_Callback(const geometry_msgs::Twist::ConstPtr &vel_msg);
  void vel2_Callback(const geometry_msgs::Twist::ConstPtr &vel_msg);

  ros::Publisher button_pub1_;
  ros::Publisher button_pub2_;
  ros::Publisher button_pub3_;
  ros::Publisher button_pub4_;
  ros::Publisher button_pub5_;
  ros::Publisher button_pub6_;
  //ros::Publisher Debug_pub1;

  ros::Subscriber joy_sub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber vel_sub_2;

  double axis_y, axis_x;
  int button[8];
  double twist_v, twist_w;
  double twist_v2, twist_w2;

  double move_x, move_y;
};

JoyStickOp::JoyStickOp()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("axis_linear_ch", linear_ch, 1);
  private_nh.param("axis_angular_ch", angular_ch, 0);
  private_nh.param("scale_angular", a_scale, 1.0);
  private_nh.param("scale_linear", l_scale, 1.0);
  twist_v = 0.0, twist_w = 0.0;

  for (int i = 0; i < 8; i++)  {
    button[i] = 0;
  }
  move_x = 0;
  move_y = 0;

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  button_pub1_ = nh_.advertise<std_msgs::Int16>("button1", 1);
  button_pub2_ = nh_.advertise<std_msgs::Int16>("button2", 1);
  button_pub3_ = nh_.advertise<std_msgs::Int16>("button3", 1);
  button_pub4_ = nh_.advertise<std_msgs::Int16>("button4", 1);
  button_pub5_ = nh_.advertise<std_msgs::Int16>("button5", 1);
  button_pub6_ = nh_.advertise<std_msgs::Int16>("button6", 1);

  //Debug_pub1 = nh_.advertise<geometry_msgs::PoseArray>("move_obst", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStickOp::joyCallback, this);
  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel2", 10, &JoyStickOp::vel_Callback, this);
  vel_sub_2 = nh_.subscribe<geometry_msgs::Twist>("cmd_vel4", 10, &JoyStickOp::vel2_Callback, this);

  signal(SIGINT, mySigintHandler);
}
void JoyStickOp::twist_send()
{

  geometry_msgs::Twist twist;

  if (button[0] == 1)
  {
    twist.angular.z = axis_x;
    twist.linear.x = axis_y;
  }
  else if (button[1] == 1)
  {
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;
  }
  else if (button[2] == 1)
  {
    twist.angular.z = twist_w2;
    twist.linear.x = twist_v2;
  }
  else
  {
    twist.angular.z = twist_w;
    twist.linear.x = twist_v;
  }

  twist_pub_.publish(twist);
  //  geometry_msgs::PoseArray move_obst_msg;
  //
  //  move_obst_msg.header.frame_id = "odom";
  //  move_obst_msg.poses.resize(100);
  //
  //  for (int i = 0; i < 100; i++)
  //  {
  //    move_obst_msg.poses[i].position.x = move_x;
  //    move_obst_msg.poses[i].position.y = move_y;
  //  }
  //
  //  Debug_pub1.publish(move_obst_msg);
}

void JoyStickOp::vel_Callback(const geometry_msgs::Twist::ConstPtr &vel_msg)
{
  twist_v = vel_msg->linear.x;
  twist_w = vel_msg->angular.z;

  twist_send();
}

void JoyStickOp::vel2_Callback(const geometry_msgs::Twist::ConstPtr &vel_msg)
{
  twist_v2 = vel_msg->linear.x;
  twist_w2 = vel_msg->angular.z;

  twist_send();
}

void JoyStickOp::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

  // ROS_INFO("I heard l: [%lf]", l_scale);
  // ROS_INFO("I heard a: [%lf]", a_scale);

  axis_x = a_scale * joy->axes[angular_ch];
  axis_y = l_scale * joy->axes[linear_ch];

  for (int i = 0; i < 8; i++)
  {
    button[i] = joy->buttons[i];
  }

  std_msgs::Int16 button_out;

  button_out.data = button[button_ch1];
  button_pub1_.publish(button_out);

  button_out.data = button[button_ch2];
  button_pub2_.publish(button_out);

  button_out.data = button[button_ch3];
  button_pub3_.publish(button_out);

  button_out.data = button[button_ch4];
  button_pub4_.publish(button_out);

  button_out.data = button[button_ch5];
  button_pub5_.publish(button_out);

  button_out.data = button[button_ch6];
  button_pub6_.publish(button_out);

  // std_msgs::Float32 joy_axis;
  move_x = joy->axes[2];
  move_y = joy->axes[3];

  twist_send();
}

void JoyStickOp::mySigintHandler(int sig){

  //終了時に速度を0に実行する。
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;

  twist_pub_.publish(twist);

  printf("shutdown catch signal %d \n", sig);
  ros::shutdown();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "n_joy2cmd");
  JoyStickOp teleop_turtle;

  ros::Rate r(10.0);

  while (teleop_turtle.nh_.ok())  {
    ros::spinOnce();
    r.sleep();

    teleop_turtle.twist_send();
  }
  return 0;
}
