/////////////////////////////////////////////////
////Create By IPF nemoto
/////////////////////////////////////////////////

#include <math.h>
#include <iostream>
#include <limits>
#include <fstream>
#include <cstdio>
#include <stdio.h>
using namespace std;

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
tf::TransformListener *tflistener;

void GetRPY(const geometry_msgs::Quaternion &quat, double &theta){
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
  //theta=yaw+3.1415;
  theta = -yaw;
}


double sub_rad(double rad1,double rad2){
  double sub_rad;
  sub_rad=rad1-rad2;
  if(fabs(rad1-rad2)>1.0*M_PI){
    if(fabs(rad1-rad2+4.0*M_PI)<fabs(rad1-rad2)){
      sub_rad=rad1-rad2+2.0*M_PI;
    }
    else if(fabs(rad1-rad2-4.0*M_PI)<fabs(rad1-rad2)){
      sub_rad=rad1-rad2-2.0*M_PI;
    }
    else if(fabs(rad1-rad2+2.0*M_PI)<fabs(rad1-rad2)){
      sub_rad=rad1-rad2+2.0*M_PI;
    }
    else if(fabs(rad1-rad2-2.0*M_PI)<fabs(rad1-rad2)){
      sub_rad=rad1-rad2-2.0*M_PI;
    }
  }
  return sub_rad;

}

void mySigintHandler(int sig){
  printf("shutdown catch signal %d \n", sig);
  ros::shutdown();
}


int main(int argc, char *argv[]){

  ros::init(argc, argv, "tf_check");

  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10));


  ros::NodeHandle private_nh("~");
  string tf_name1;
  string tf_name2;
  string tf_name3;
  string tf_name4;
  private_nh.param("tf_name1",tf_name1,std::string("/base_link"));
  private_nh.param("tf_name2",tf_name2,std::string("/map"));
  private_nh.param("tf_name3",tf_name3,std::string("/odom_true"));
  private_nh.param("tf_name4",tf_name4,std::string("/map"));

  int end_time=0;
  private_nh.param("end_time",end_time,200);


  string outfname;
  private_nh.param("fname",outfname,std::string("/home/iqit/"));

  string outfname_total;
  private_nh.param("fname_total",outfname_total,std::string("/home/iqit/"));


  ros::Rate r(10.0);
  double dis_o=0.0;
  double dis_m=0.0;
  double dis_e=0.0;

  double rot_o=0.0;
  double rot_m=0.0;
  double rot_e=0.0;
  static double x_po=0.0, y_po=0.0, yaw_po=0.0;
  static double x_pm=0.0, y_pm=0.0, yaw_pm=0.0;
  static double x_pe=0.0, y_pe=0.0, yaw_pe=0.0;

  double total_dis_e=0.0;
  double total_rot_e=0.0;

  //現在時刻取得
  char fname[200];
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  sprintf(fname, "%s_%04d%02d%02d%02d%02d%02d.csv",outfname.c_str(), pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
  pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
  std::ofstream fout(fname) ;


  fout<< "error" << ",";
  fout<< "dis_error" << ",";
  fout<< "x_error" << ",";
  fout<< "y_error" << ",";
  fout<< "yaw_error" << ",";

  fout<< "slam" << ",";
  fout<< "x_slam" << ",";
  fout<< "y_slam" << ",";
  fout<< "yaw_slam"<< ",";


  fout<< "odom" << ",";
  fout<< "x_odom" << ",";
  fout<< "y_odom" << ",";
  fout<< "yaw_odom" << ",";
  fout<< endl;
  signal(SIGINT, mySigintHandler);
  int cnt=0;
  while (n.ok())  {
    cnt++;
    if(cnt>end_time) break;

    double x_o=0.0, y_o=0.0, yaw_o=0.0;
    double x_m=0.0, y_m=0.0, yaw_m=0.0;
    double x_e=0.0, y_e=0.0, yaw_e=0.0;

    tf::StampedTransform trans_slam;
    tf::StampedTransform trans_true_pos;
    tf::StampedTransform trans_error;
    try {
      listener.lookupTransform(tf_name2, tf_name1,ros::Time(0), trans_slam);
      x_m = trans_slam.getOrigin().x();
      y_m = trans_slam.getOrigin().y();
      yaw_m = tf::getYaw(trans_slam.getRotation());
      if(x_pm!=0)   dis_m+=sqrt((x_pm-x_m)*(x_pm-x_m)+(y_pm-y_m)*(y_pm-y_m));
      if(yaw_pm!=0)   rot_m+=fabs(sub_rad(yaw_m,yaw_pm));

      listener.lookupTransform(tf_name4, tf_name3,ros::Time(0), trans_true_pos);
      x_o = trans_true_pos.getOrigin().x();
      y_o = trans_true_pos.getOrigin().y();
      yaw_o = tf::getYaw(trans_true_pos.getRotation());
      if(x_po!=0)   dis_o+=sqrt((x_po-x_o)*(x_po-x_o)+(y_po-y_o)*(y_po-y_o));
      if(yaw_po!=0)   rot_o+=fabs(sub_rad(yaw_o,yaw_po));

      //          listener.lookupTransform(tf_name3, tf_name1,ros::Time(0), trans_error);
      //            x_e = trans_error.getOrigin().x();
      //            y_e = trans_error.getOrigin().y();
      //            yaw_e = tf::getYaw(trans_error.getRotation());
      //            if(x_pe!=0)   dis_e+=sqrt((x_pe-x_e)*(x_pe-x_e)+(y_pe-y_e)*(y_pe-y_e));
      //            if(yaw_pe!=0)   rot_e+=fabs(sub_rad(yaw_o,yaw_po));
      x_e=(x_m-x_o);
      y_e=(y_m-y_o);
      yaw_e = yaw_m-yaw_o;
      dis_e=sqrt(x_e*x_e+y_e*y_e);

      total_dis_e+=dis_e;
      total_rot_e+=yaw_e;

      x_pm=x_m;
      x_po=x_o;
      x_pe=x_e;

      y_pm=y_m;
      y_po=y_o;
      y_pe=y_e;

      yaw_pm=yaw_m;
      yaw_po=yaw_o;
      yaw_pe=yaw_e;

    }

    catch (tf::TransformException &ex)  {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    cout<< "error" << ",";
    cout<< dis_e << ",";
    cout<< x_e << ",";
    cout<< y_e << ",";
    cout<< yaw_e/M_PI*180.0 <<endl;

    cout<< "mcl" << ",";
    cout<< x_m << ",";
    cout<< y_m << ",";
    cout<< yaw_m/M_PI*180.0<< endl;

    cout<< "odom" << ",";
    cout<< x_o << ",";
    cout<< y_o << ",";
    cout<< yaw_o/M_PI*180.0 << endl;

    cout<< endl;
    cout<< endl;

    fout<< "error" << ",";
    fout<< dis_e << ",";
    fout<< x_e << ",";
    fout<< y_e << ",";
    fout<< yaw_e/M_PI*180.0 << ",";

    fout<< "mcl" << ",";
    fout<< x_m << ",";
    fout<< y_m << ",";
    fout<< yaw_m/M_PI*180.0<< ",";

    fout<< "odom" << ",";
    fout<< x_o << ",";
    fout<< y_o << ",";
    fout<< yaw_o/M_PI*180.0 << ",";
    fout<< endl;
    ros::spinOnce();
    r.sleep();
  }
  int ret;


  char fname_total[200];
  sprintf(fname_total, "%s_total.csv",outfname_total.c_str());
  std::ofstream fout_total(fname_total,ios_base::app) ;
 
  fout_total<< fname<< ",";
  fout_total<< "total_dis_e" << ","<< total_dis_e << ",";
  fout_total<< "total_rot_e" << ","<< total_rot_e /M_PI*180.0<< endl;


  ret = system("rosrun n_map_saver n_map_saver_node -f map_");

  usleep(1*1000*1000);
  ret = system("rosnode kill /cartographer ");
  usleep(1*1000*1000);

  ret = system("rosnode kill -a");

  //  usleep(1*1000*1000);


  //exit();


  return 0;
}
