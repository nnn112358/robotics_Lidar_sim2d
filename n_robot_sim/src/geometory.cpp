/////////////////////////////////////////////////

/////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#define _USE_MATH_DEFINES
#include <cmath>

#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.0)
const double D_MIN=1.0/1000000.0;
#include "geometory.h"

double rand_uniform(void){
	return ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
}

double rand_normal(double mu, double sigma){
	double z = sqrt(-2.0 * log(rand_uniform())) * sin(2.0 * M_PI * rand_uniform());
	return mu + sigma * z;
}

double str_double(string str){
	istringstream is;
	is.str(str);
	double x;
	is >> x;
	return x;
}

double get_dtime(){
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long double n_time = tv.tv_sec + tv.tv_usec * 1e-6;
	static long double o_time = n_time;
	double dt_msec = (n_time - o_time);
	o_time = n_time;
	return dt_msec;
}

double get_time(){
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long double n_time = tv.tv_sec + tv.tv_usec * 1e-6;
	return n_time;
}

bool cross_check(double a1_x, double a1_y, double a2_x, double a2_y, double b1_x, double b1_y, double b2_x, double b2_y){
	double v1 = (a2_x - a1_x) * (b1_y - a1_y) - (a2_y - a1_y) * (b1_x - a1_x);
	double v2 = (a2_x - a1_x) * (b2_y - a1_y) - (a2_y - a1_y) * (b2_x - a1_x);
	double m1 = (b2_x - b1_x) * (a1_y - b1_y) - (b2_y - b1_y) * (a1_x - b1_x);
	double m2 = (b2_x - b1_x) * (a2_y - b1_y) - (b2_y - b1_y) * (a2_x - b1_x);
	if ((v1 * v2 <= 0) && (m1 * m2 <= 0)){
		return true;
	}
	else	{
		return false;
	}
}

bool cross_xy(double a1_x, double a1_y, double a2_x, double a2_y, double b1_x, double b1_y, double b2_x, double b2_y, double &cro_x, double &cro_y){
	double aa_a;
	double bb_a;

	if((a2_x - a1_x)==0.0){
		aa_a = (a2_y - a1_y) / (a2_x - a1_x + D_MIN);
	}
	else{
		aa_a = (a2_y - a1_y) / (a2_x - a1_x );
	}

	if((b2_x - b1_x)==0.0){
		bb_a = (b2_y - b1_y) / (b2_x - b1_x + D_MIN);
	}
	else{
		bb_a = (b2_y - b1_y) / (b2_x - b1_x );
	}

	double aa_b = a1_y - aa_a * a1_x;
	double bb_b = b1_y - bb_a * b1_x;

	if (a2_x != a1_x)	{
		aa_a = (a2_y - a1_y) / (a2_x - a1_x);
		aa_b = a1_y - aa_a * a1_x;
	}

	if (b2_x != b1_x)	{
		bb_a = (b2_y - b1_y) / (b2_x - b1_x);
		bb_b = b1_y - bb_a * b1_x;
	}
	if (aa_a != bb_a)	{
		cro_x = -(bb_b - aa_b) / (bb_a - aa_a);
		cro_y = cro_x * aa_a + aa_b;
		return true;
	}
	return false;
}


void lrf_noise( LRF &lrf_data){
		for(int i=0;i<lrf_data.data_num;i++){
		lrf_data.leng[i] = lrf_data.leng[i] +lrf_data.leng[i]*rand_normal(0,1.0)*lrf_data.leng_stdev;
		if(lrf_data.leng[i]==lrf_data.leng_max){
				lrf_data.leng[i]=NAN;
		}

	}
}


//仮想距離センサ
void lrf_calcurate(ROBOT &robot, LRF lrf_area, OBSTACLE obst, MOVE_OBSTACLE move_obst, LRF &lrf_data_out){

	LRF lrf_data;
	lrf_data.data_num = lrf_data_out.data_num ;
	lrf_data.start_rad =	lrf_data_out.start_rad ;
	lrf_data.reso =	lrf_data_out.reso;
	lrf_data.leng_max =lrf_data_out.leng_max;
	lrf_data.leng_min = lrf_data_out.leng_min ;
	lrf_data.leng_stdev = lrf_data_out.leng_stdev;

	//URGセンサの検知:URGの上の表示
	for (int i = 0; i < lrf_area.data_num; i++)
			lrf_area.leng[i] = lrf_data.leng_max;


	//URG_Dataの初期化
	for(int i=0;i<lrf_data.data_num;i++){
		lrf_data.x[i]=robot.x;
		lrf_data.y[i]=robot.y;
		lrf_data.leng[i]=0.0;
	}

	//URG_レーザ
	for(int i=0;i<lrf_area.data_num;i++){
		lrf_area.x[i]=(-1)*lrf_area.leng[i]*sin(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
		lrf_area.y[i]=lrf_area.leng[i]*cos(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;
	}

	for(int j=0;j<obst.n;j++){
		for(int i=0;i<lrf_area.data_num;i++){
			double a1_x=obst.x1[j];
			double a1_y=obst.y1[j];
			double a2_x=obst.x2[j];
			double a2_y=obst.y2[j];
			double b2_x=lrf_area.x[i];
			double b2_y=lrf_area.y[i];
			double b1_x=robot.x;
			double b1_y=robot.y;
			//線分の交差判定
			if(cross_check(a1_x,a1_y,a2_x,a2_y,b1_x,b1_y,b2_x,b2_y)==true){
				double ata_x=0.0;
				double ata_y=0.0;
				//線分の交点の算出
				if(cross_xy(a1_x,a1_y,a2_x,a2_y,b1_x,b1_y,b2_x,b2_y,ata_x,ata_y)==true){//交点のある場合

					if(lrf_data.leng[i]==0){	//既にデータが入っていないなら
						lrf_data.leng[i]=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
					}

					else{											//既にデータが入っていたら、最短点を格納
						double oldleng=lrf_data.leng[i];
						double temp_leng=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
						if(temp_leng<oldleng){//元より小さければ格納
							lrf_data.leng[i]=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
						}
					}

				}
				//線分の交点算出はここまで
			}
			//線分の交差判定はここまで
		}
	}

	for(int j=0;j<move_obst.n;j++){
		for(int k=0;k<move_obst.m-1;k++){
			for(int i=0;i<lrf_area.data_num;i++){
				double a1_x=move_obst.obst_x[j][k];
				double a1_y=move_obst.obst_y[j][k];
				double a2_x=move_obst.obst_x[j][k+1];
				double a2_y=move_obst.obst_y[j][k+1];
				double b2_x=lrf_area.x[i];
				double b2_y=lrf_area.y[i];
				double b1_x=robot.x;
				double b1_y=robot.y;
				//線分の交差判定
				if(cross_check(a1_x,a1_y,a2_x,a2_y,b1_x,b1_y,b2_x,b2_y)==true){
					double ata_x=0.0;
					double ata_y=0.0;
					//線分の交点の算出
					if(cross_xy(a1_x,a1_y,a2_x,a2_y,b1_x,b1_y,b2_x,b2_y,ata_x,ata_y)==true){//交点のある場合
						if(lrf_data.leng[i]==0){		//既にデータが入っていないなら
							lrf_data.leng[i]=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
						}
						else{												//既にデータが入っていたら、最短点を格納
							double oldleng=lrf_data.leng[i];
							double temp_leng=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
							if(temp_leng<oldleng){//元より小さければ格納
								lrf_data.leng[i]=sqrt((ata_y-robot.y)*(ata_y-robot.y)+(ata_x-robot.x)*(ata_x-robot.x));
							}
						}
					}
				}//線分の交点算出はここまで
			}//線分の交差判定はここまで
		}
	}

	lrf_noise(lrf_data);

	for(int i=0;i<lrf_data.data_num;i++){
		lrf_data_out.x[i]=lrf_data.x[i];
		lrf_data_out.y[i]=lrf_data.y[i];
		lrf_data_out.leng[i]=lrf_data.leng[i];
	}
}


//障害物情報の読み込み
void read_obstacle(OBSTACLE &obst,string fname){
	cout << "Obstacle File reading" << endl;

	string filename(fname.c_str());
	string str_line;
	ifstream ifs( filename.c_str() );

	if( !ifs ) {
		cout << "Error:Input data file not found" << endl;
		return;
	}

	string obst_in[4096][4]={};

	int line=0;
	int width=0;
	getline( ifs, str_line );
	while( getline( ifs, str_line ) ){
		string token;
		istringstream stream( str_line );

		while( getline( stream, token, ',' ) ) {
			obst_in[line][width]=token;
			//		cout << obst_in[line][width] << ",";
			width++;
		}
		line++;
		width=0;

		//	cout << endl;
	}

	obst.n=line;
	//cout<<"obst:" <<obst.n<< endl;

	for(int i=0;i<obst.n;i++){
		obst.x1[i]=str_double(obst_in[i][0]);
		obst.y1[i]=str_double(obst_in[i][1]);
		obst.x2[i]=str_double(obst_in[i][2]);
		obst.y2[i]=str_double(obst_in[i][3]);
	//	obst.z[i]=obstacle_z;
	}
//	for(int i=0;i<obst.n;i++){
//		cout << obst.x1[i] << ",";
//		cout << obst.x2[i] << ",";
//		cout << obst.y1[i] << ",";
//		cout << obst.y2[i] << ",";
//		cout << endl;
//	}

	return;
}

void write_obstacle(OBSTACLE &obst, string fname){
	cout << "Obstacle File Write" << endl;
	for (int i = 0; i < obst.n; i++){
		cout << obst.x1[i] << ",";
		cout << obst.x2[i] << ",";
		cout << obst.y1[i] << ",";
		cout << obst.y2[i] << ",";
		cout << endl;
	}

	std::ofstream writing_file;
	writing_file.open(fname.c_str(), std::ios::out);
	std::cout << "writing " << fname << "..." << std::endl;
	for (int i = 0; i < obst.n; i++){
		writing_file << obst.x1[i] * 1.0 << "\t";
		writing_file << obst.y1[i] * 1.0 << "\t";
		writing_file << obst.x2[i] * 1.0 << "\t";
		writing_file << obst.y2[i] * 1.0 << "";
		writing_file << endl;
	}
	return;
}


void robot_move(ROBOT &myrobot,MOVE_OBSTACLE &move_obst,double dt){
	if(dt==0.0)	dt+=D_MIN;

	for (int i = 0; i < move_obst.n; i++){
		double dis=(move_obst.x[i]-myrobot.x)*(move_obst.x[i]-myrobot.x)+(move_obst.y[i]-myrobot.y)*(move_obst.y[i]-myrobot.y);
		if(dis<(move_obst.obst_size+myrobot.size)){
//			return;
		}
	}

	double ac_v=fabs(myrobot.v-myrobot.real_v)/dt;
	double ac_w=fabs(myrobot.w-myrobot.real_w)/dt;

	if(ac_v>=myrobot.ac_trans_limit)	ac_v=myrobot.ac_trans_limit;
	if(ac_w>=myrobot.ac_rot_limit)		ac_w=myrobot.ac_rot_limit;

	if((myrobot.v-myrobot.real_v)>=0)		myrobot.real_v+=ac_v*dt;
	else if((myrobot.v-myrobot.real_v)<0)	myrobot.real_v-=ac_v*dt;

	if((myrobot.w-myrobot.real_w)>=0)		myrobot.real_w+=ac_w*dt;
	else if((myrobot.w-myrobot.real_w)<0)	myrobot.real_w-=ac_w*dt;

	myrobot.x+=myrobot.real_v*sin(-myrobot.theta)*dt;
	myrobot.y+=myrobot.real_v*cos(-myrobot.theta)*dt;
	myrobot.theta-=myrobot.real_w*dt;

 double odometory_noise_liner=myrobot.odom_noise_liner;
 double odometory_noise_liner_sigma=myrobot.odom_noise_liner_stdev;
 double odometory_noise_angle=myrobot.odom_noise_angle;
 double odometory_noise_angle_sigma=myrobot.odom_noise_angle_stdev;

	myrobot.x_dummy+=myrobot.real_v*sin(-myrobot.theta_dummy)*dt*(1+fabs(rand_normal(odometory_noise_liner,odometory_noise_liner_sigma))*1.0);
	myrobot.y_dummy+=myrobot.real_v*cos(-myrobot.theta_dummy)*dt*(1+fabs(rand_normal(odometory_noise_liner,odometory_noise_liner_sigma))*1.0);
	myrobot.theta_dummy-=myrobot.real_w*dt*(1+fabs(rand_normal(odometory_noise_angle,odometory_noise_angle_sigma))*1.0);





}
