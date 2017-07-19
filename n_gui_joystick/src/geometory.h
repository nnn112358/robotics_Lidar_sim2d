
#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;
const double PI=3.14159265359;

#define URG_DATA_MAX 1024
const int urg_data_num=680;
const double urg_range_deg=240.0;

struct TWIST{		//Twist���`
	double v;		//�ڕW���x(m/s)
	double w;		//�ڕW�p���x(rad/s)
	TWIST(){
		v = 0.0;
		w = 0.0;
	}
};


struct ROBOT{		//���{�b�g�̒��`
	double x;		//X���W
	double y;		//Y���W
	double theta;	//�p�x
	double v;		//���x
	double w;		//�p���x

	ROBOT(){
		x = 0.0;
		y = 0.0;
		v = 0.0;
		w = 0.0;
		theta = 0.0;
	}

	void odometory(double dt){
		x+=v*sin(-theta)*dt;
		y+=v*cos(-theta)*dt;
		theta-=w*dt;

		//if(theta<=(-2.0*PI)) theta+=2*PI;
		//if(theta>(2.0*PI)) theta-=2*PI;

	}
};

struct URG{			//�����Z���T�̒��`
	double leng[URG_DATA_MAX];
	double x[URG_DATA_MAX];
	double y[URG_DATA_MAX];
	int data_num;
	double start_rad;
	double reso;

	URG(){
		data_num=URG_DATA_MAX*urg_range_deg/360.0;
		start_rad=-data_num/2.0/URG_DATA_MAX*2*PI;
		reso=2*PI/URG_DATA_MAX;

		for(int i=0;i<URG_DATA_MAX;i++){
		x[i] = 0.0;
		y[i] = 0.0;
		leng[i] = 0.0;
		}
	}
};

struct OBSTACLE{		//���Q���Z���T�̒��`
	double x1[1024];
	double x2[1024];
	double y1[1024];
	double y2[1024];
	int n;
	OBSTACLE(){
		for(int i=0;i<100;i++){
		x1[i] = 0.0;
		x2[i] = 0.0;
		y1[i] = 0.0;
		y2[i] = 0.0;
		}

	}
};


//�����̌��������֐�
bool cross_xy(double a1_x,double a1_y,double a2_x,double a2_y,double b1_x,double b1_y,double b2_x,double b2_y,double &cro_x,double &cro_y);

//���_�̍��W�����߂�
bool cross_check(double a1_x,double a1_y,double a2_x,double a2_y,double b1_x,double b1_y,double b2_x,double b2_y);

//URG�ŏ��Q�������m����
void urg_calcurate( ROBOT &robot, URG urg_area, OBSTACLE obst, URG &urg_data);

//���Q�����ǂݍ���
void read_obstacle(string ,OBSTACLE &);

//���Ԃ��v������
double get_dtime(void);

//str��double�ɕϊ�����
double str_double(string str);


//xy���W�̐����p�xtheta�����߂��B
void arc_tan2(double &target_x,double &target_y,double &theta);

//�ړI�l�Ɍ��������x�w�ߒl���Z�o����
void follow_velocity(double goal_x,double goal_y,double robot_theta,double &speed,double &turn);
