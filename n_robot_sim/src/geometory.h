#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cfloat>

#include <sstream>
#include <fstream>
using namespace std;
#define LRF_DATA_MAX 1024
#define OBSTACLE_MAX 1024
#define MOVE_OBST_MAX_MAIN 1000
#define PATH_POINTS_MAX 1024

double rand_uniform( void );
double rand_normal( double mu, double sigma );

struct ROBOT{
	double x;
	double y;
	double theta;
	double v;
	double w;

	double size;

	long double x_dummy;
	long double y_dummy;
	long double theta_dummy;
	double real_v;
	double real_w;

	double ac_trans_limit;
	double ac_rot_limit;

	double odom_noise_liner;
	double odom_noise_liner_stdev;
	double odom_noise_angle;
	double odom_noise_angle_stdev;

	ROBOT(){
		x = 0.0001;
		y = 0.0;
		v = 0.0;
		w = 0.0;
		theta = 0.0;
		x_dummy = x;
		y_dummy = 0.0;
		theta_dummy = 0.0;

		size=0.5;

		real_v=0.0;
		real_w=0.0;
		ac_trans_limit=5.0;
		ac_rot_limit=5.0;
		odom_noise_liner=0.0;
		odom_noise_liner_stdev=1.0;
		odom_noise_angle=0.0;
		odom_noise_angle_stdev=1.0;
	}
};

struct LRF{
	double leng[LRF_DATA_MAX];
	double x[LRF_DATA_MAX];
	double y[LRF_DATA_MAX];
	int data_num;
	double start_rad;
	double reso;
	double leng_max;
	double leng_min;
	double leng_stdev;

	LRF(){
		data_num=768.0;
		start_rad= -240.0 / 2.0 / 360.0 * 2 * M_PI;
		reso=240.0 / 768.0 / 360.0 * 2 * M_PI;
		leng_max=4.0;
		leng_min=0.1;
		leng_stdev=0;

		for(int i=0;i<data_num;i++){
			x[i] = 0.0;
			y[i] = 0.0;
			leng[i] = 0.0;
		}
	}


};

struct OBSTACLE{
	double x1[OBSTACLE_MAX];
	double x2[OBSTACLE_MAX];
	double y1[OBSTACLE_MAX];
	double y2[OBSTACLE_MAX];
	unsigned int n;
	unsigned int n_max;

	OBSTACLE(){
		n_max=OBSTACLE_MAX;
		for(int i=0;i<n_max;i++){
			x1[i] = 0.0;
			x2[i] = 0.0;
			y1[i] = 0.0;
			y2[i] = 0.0;
		}

	}
};

struct MOVE_OBSTACLE{
	double obst_x[100][5];
	double obst_y[100][5];
	double theta[100];

//	double phase[100];
//	double freq[100];
//	double amp[100];

	double x[100];
	double y[100];
//	double now;
	double n;
	double m;
	double obst_size;
	double init_area;

	MOVE_OBSTACLE(){
		obst_size=0.5;
		n=10;
		m=5;
		for(int i=0;i<n;i++){
			x[i]=10.0;
			y[i]=10.0;
//			phase[i]=0.0;
//			amp[i]=0;
//			freq[i]=0;
		}
		for(int i=0;i<n;i++){
			for(int j=0;j<n;j++){
				obst_x[i][j]=0.0;
				obst_y[i][j]=0.0;
			}
		}
		init_area=30.0;
		for(int i=0;i<50;i++){
			x[i]=((double)rand() / ((double)RAND_MAX + 1)-0.5)*init_area;
			y[i]=((double)rand() / ((double)RAND_MAX + 1)-0.5)*init_area;
			theta[i] = ((double)rand() / ((double)RAND_MAX + 1)-0.5)* M_PI;
		}
	}

};


bool cross_xy(double a1_x,double a1_y,double a2_x,double a2_y,double b1_x,double b1_y,double b2_x,double b2_y,double &cro_x,double &cro_y);
bool cross_check(double a1_x,double a1_y,double a2_x,double a2_y,double b1_x,double b1_y,double b2_x,double b2_y);
void lrf_noise( LRF &lrf_data);
void lrf_calcurate(ROBOT &robot, LRF lrf_area, OBSTACLE obst, MOVE_OBSTACLE move_obst, LRF &lrf_data_out);
void read_obstacle(OBSTACLE &obst,string fname="obstacle.dat");
void write_obstacle(OBSTACLE &obst,string fname="obstacle.dat");
double get_dtime(void);
double get_time(void);

double str_double(string str);
void arc_tan2(double &target_x,double &target_y,double &theta);
void To_goal_velocity(double goal_x,double goal_y,double robot_x,double robot_y,double robot_theta,double &speed,double &turn,double goal_reach_dis=0.2,double turn_limit=3.14);
void robot_move(ROBOT &myrobot,MOVE_OBSTACLE &move_obst,double dt);
