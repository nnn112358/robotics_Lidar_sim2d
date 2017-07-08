/////////////////////////////////////////////////
/////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>
using namespace std;

#include "gl_view.h"
#include "geometory.h"

//各種OpenGL変数宣言
int xBegin, yBegin;												//mouseのXY座標
int mButton;															//mouseのClick
float distance_gl, twist, elevation, azimuth;//カメラの回転行列
float xOrig , yOrig, zOrig;					//カメラの位置

//Robot制御のパラメータ
struct ROBOT myrobot;						//ロボットの位置
struct LRF lrf_area;						//LRF走査範囲
struct LRF lrf_data;						//LRFデータ
struct OBSTACLE obst; 					//障害物情報
struct MOVE_OBSTACLE move_obst;	//移動障害物
int gl_mode=0;//モードを切り替える。
double goal_x=0.0,goal_y=0.0;
string gl_map_fname;


int Graphics::wait_sleep(int time){
	usleep(time*1000);
	return 0;
}


Graphics::Graphics(){
	xOrig = 0.0, yOrig = 0.0, zOrig=0.0; //始点中心
	gui_start();
}

Graphics::~Graphics(){
	gui_end();
}
int Graphics::main_proc(){
    double old_time=get_time();

		for(;;){
		double now_stock = get_time(); //msec
    double now =now_stock-old_time;
    old_time=now_stock;
		//オドメトリーを計算/速度・角速度を積分して、ロボットを移動
		robot_move(myrobot, move_obst,now);
		//LRFの検出
		move_obstacle_cal(now);
		//lrfセンサの検知
		lrf_calcurate(myrobot,lrf_area,obst,move_obst,lrf_data);
		wait_sleep(10);
	}
	return 0;
}

void Graphics::main_draw() {

	double now = get_dtime(); 		//msec
	move_obstacle_view();					//移動障害物
	robot_view(myrobot);					//Robotの表示
	lrf_view(myrobot,lrf_data);					//lrfの表示
	obstacle_view();


	//Gridの表示
	glColor3f(0.9,0.9,0.9);
	glLineWidth(1.0);

	double grid_size=1.0;
	int width=50;
	int data_num=width/grid_size;

	for(int i=0;i<data_num*data_num;i++){
		double start_x=data_num/2.0;
		double start_y=data_num/2.0;
		int ix=i%data_num;
		int iy=i/data_num;
		double xx=ix*grid_size-start_x;
		double yy=iy*grid_size-start_y;
		glBegin(GL_LINE_LOOP);
		glVertex3f(xx,yy,-0.1);
		glVertex3f(xx+grid_size,yy,-0.1);
		glVertex3f(xx+grid_size,yy+grid_size,-0.1);
		glVertex3f(xx,yy+grid_size,-0.1);
		glEnd();
	}

	return ;
}

void Graphics::display(){
	glClearColor(1 , 1 , 1 , 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glPushMatrix ();
	polarview();

	//メイン描画の関数
	main_draw();

	glPopMatrix();
	glutSwapBuffers();
	glutPostRedisplay();
	wait_sleep(30);
}

void Graphics::idle(void){
	glutPostRedisplay();
	wait_sleep(30);
}

void Graphics::myInit(string progname){
	float aspect = (float) WIDTH / (float) HEIGHT;

	glutInitWindowPosition(0, 0);
	glutInitWindowSize( WIDTH, HEIGHT);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA| GLUT_DEPTH);
	glutCreateWindow(progname.c_str());
	glClearColor (0, 0, 0, 1.0);

	glutKeyboardFunc( myKbd );
	glutKeyboardUpFunc( myKbdup );
	glutSpecialFunc( mySkey );

	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);
	resetview();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, aspect, 0.010, 100.0);//画角等視野の設定
	glMatrixMode(GL_MODELVIEW);

	lrf_area.data_num = lrf_data.data_num ;
	lrf_area.start_rad =	lrf_data.start_rad ;
	lrf_area.reso =	lrf_data.reso;
	lrf_area.leng_max =lrf_data.leng_max;
	lrf_area.leng_min = lrf_data.leng_min ;
	lrf_area.leng_stdev = lrf_data.leng_stdev;
}

void Graphics::reshape(int ,int){
	glutReshapeWindow( WIDTH, HEIGHT );
}

 int Graphics::GUImain(){
	read_obstacle(obst,gl_map_fname);

	int argc= 3;
	char *argv[]    ={"program", "-display", ":0.0"};
	glutInit ( &argc,argv );

	string win_name="simulation";
	myInit(win_name);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutIdleFunc(idle);
	glutMainLoop();
	return 0;
}



void Graphics::gui_start(){
	th2 = SDL_CreateThread((int (*)(void *))main_proc, this);
	th1 = SDL_CreateThread((int (*)(void *))GUImain,this);
}


void Graphics::gui_end(){
	SDL_KillThread(th1);
  SDL_KillThread(th2);

}

void Graphics::myMouse( int button, int state, int x, int y ){

	double obj_x=0,obj_y=0,obj_z=0;

	if (state == GLUT_DOWN) {
		switch(button) {
			case GLUT_LEFT_BUTTON:
			mButton = button;
			click_pickup(x,y,obj_x,obj_y,obj_z);
			goal_y=obj_y;
			goal_x=obj_x;

			if(gl_mode==2){
				if((obst.x1[obst.n]==0)&&(obst.y1[obst.n]==0)){
					obst.x1[obst.n]=obj_x;
					obst.y1[obst.n]=obj_y;
				}
				else{
					obst.x2[obst.n]=obj_x;
					obst.y2[obst.n]=obj_y;
					obst.n++;
					write_obstacle(obst);
				}
			}
			break;
			case GLUT_MIDDLE_BUTTON:

			break;
			case GLUT_RIGHT_BUTTON:

			if(gl_mode==2){//obstacle_delete
				if(obst.n>0){
					obst.n--;
					obst.x1[obst.n]=0.0;
					obst.y1[obst.n]=0.0;
					obst.x2[obst.n]=0.0;
					obst.y2[obst.n]=0.0;
				}
			}
			mButton = button;
			break;
		}
		xBegin = x;
		yBegin = y;
	}
	if (state == GLUT_UP) {
		switch(button) {
			case GLUT_RIGHT_BUTTON:

			break;
		}
	}

	return ;
}


void Graphics::myMotion( int x, int y ){
	int xDisp, yDisp;
	xDisp = x - xBegin;
	yDisp = y - yBegin;

	switch (mButton) {
		case GLUT_LEFT_BUTTON:
		xOrig += (float) xDisp/50.0;
		zOrig -= (float) yDisp/50.0;
		break;
		case GLUT_RIGHT_BUTTON:
		distance_gl -= (float) xDisp/10.0;
		if(distance_gl<0.1)distance_gl=0.1;
		break;
	}
	xBegin = x;
	yBegin = y;


	glutPostRedisplay();
}


void Graphics::myKbdup( unsigned char key, int x, int y ){
	switch( key ) {
	}

}
void Graphics::myKbd( unsigned char key, int x, int y )
{
	switch( key ) {
		case 0x1B:	//終了
		exit(0);
		break;

//		case 'z':	//モード切り替え
//		gl_mode++;
//		myrobot.v=0;
//		myrobot.w=0;
		break;
	}
}



void Graphics::mySkey( int key, int x, int y )
{
	switch( key ) {
		case GLUT_KEY_LEFT:
		xOrig -= 0.2;
		break;
		case GLUT_KEY_RIGHT:
		xOrig += 0.2;
		break;
		case GLUT_KEY_UP:
		zOrig += 0.2;
		break;
		case GLUT_KEY_DOWN:
		zOrig -= 0.2;
		break;
		case GLUT_KEY_PAGE_UP:
		yOrig += 0.2;
		break;
		case GLUT_KEY_PAGE_DOWN:
		yOrig -= 0.2;
		break;

	}
	glutPostRedisplay();
}


void Graphics::resetview( void ){
	distance_gl = 50.0;
	twist = 0.0;
	elevation = 0.0;
	azimuth = 0.0;
}
void Graphics::polarview( void ){
	//マウスで移動
	glTranslatef( 0.0, 0.0, -distance_gl);
	glRotatef( -twist, 0.0, 0.0, 1.0);
	glRotatef( -elevation, 1.0, 0.0, 0.0);
	glRotatef( -azimuth, 0.0, 1.0, 0.0);
	//キーボードで移動
	glTranslatef(xOrig,zOrig ,yOrig);
}

float Graphics::click_Depth(int x, int y){//マウスのX/Y座標からDepthを算出
	float z;
	GLint viewport[4];                      //ビューポート
	//デバイス座標系とウィンドウ座標系の変換
	glGetIntegerv(GL_VIEWPORT,viewport);    //現在のビューポートを代入
	glReadPixels(x,viewport[3]-y,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&z);
	// デプスバッファの値を返す．
	return z;
}

void Graphics::click_pickup(int x,int y,double &ax,double &ay,double &az){//マウスのX/Y座標からX/Y/Z座標を算出
	GLdouble modelview[16];
	GLdouble projection[16];
	GLint viewport[4];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLdouble winX, winY, winZ;
	GLdouble objX=0.0,objY=0.0,objZ=-distance_gl+yOrig;

	//原点のGLUT座標系を算出
	gluProject(objX,objY,objZ,modelview,projection,viewport,&winX,&winY,&winZ);
	GLdouble objX1,objY1,objZ1;
	//gluUnProject((double)x,(double)y,0.0,modelview,projection,viewport,&objX1,&objY1,&objZ1);
	//cout<<"near_window:"<<objX1<<"\t"<<objY1<<"\t"<<objZ1<<"\t"<<endl;
	//gluUnProject((double)x,(double)y,1.0,modelview,projection,viewport,&objX1,&objY1,&objZ1);
	//cout<<"far_window:"<<objX1<<"\t"<<objY1<<"\t"<<objZ1<<"\t"<<endl;
	gluUnProject((double)x,(double)y,winZ,modelview,projection,viewport,&objX1,&objY1,&objZ1);
	ax=objX1-xOrig;
	ay=-(objY1+zOrig);
	az=objZ1-yOrig;
	//cout<<"mouse_click:"<<"\t X:"<<x<<"\t Y:"<<y<<"\t x:"<<ax<<"\t y:"<<ay<<"\t z:"<<az<<""<<endl;
	return;
}
void Graphics::draw_string(string str, int w, int h, int x0, int y0){
	glColor3d(0.0, 0.60, 0.0);
	// glDisable(GL_LIGHTING);
	// 平行投影にする
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// 画面上にテキスト描画
	glRasterPos2f(x0, y0);
	int size = (int)str.size();
	for(int i = 0; i < size; ++i){
		char ic = str[i];
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


void Graphics::move_obstacle_cal(double t_now){
    static long double now = 0;
    now += (t_now) ;

		double phase = 0;
    double offset_y = 0.0;


    for (int i = 0; i < move_obst.n; i++){
				//double dis=(move_obst.x[i]-myrobot.x)*(move_obst.x[i]-myrobot.x)+(move_obst.y[i]-myrobot.y)*(move_obst.y[i]-myrobot.y);
				double theta=move_obst.theta[i];

				//if(dis>(move_obst.obst_size+myrobot.size)){

        move_obst.obst_x[i][0] = move_obst.x[i] - move_obst.obst_size;
        move_obst.obst_x[i][1] = move_obst.x[i] - move_obst.obst_size;
        move_obst.obst_x[i][2] = move_obst.x[i] + move_obst.obst_size;
        move_obst.obst_x[i][3] = move_obst.x[i] + move_obst.obst_size;
        move_obst.obst_x[i][4] = move_obst.x[i] - move_obst.obst_size;

        move_obst.obst_y[i][0] = move_obst.y[i] - move_obst.obst_size + offset_y;
        move_obst.obst_y[i][1] = move_obst.y[i] + move_obst.obst_size + offset_y;
        move_obst.obst_y[i][2] = move_obst.y[i] + move_obst.obst_size + offset_y;
        move_obst.obst_y[i][3] = move_obst.y[i] - move_obst.obst_size + offset_y;
        move_obst.obst_y[i][4] = move_obst.y[i] - move_obst.obst_size + offset_y;
			//}
		}


}



void Graphics::robot_view(struct ROBOT &robot){
	glLineWidth(2.0);

	double xx, yy, theta;
	xx=robot.x;
	yy=robot.y;
	theta=robot.theta;
	static double radius=0.5;
	glColor3f(0, 0, 0.0);
	int grid=24.0;

	glBegin(GL_LINES);
	glVertex3f(xx, yy, 0.010);
	glVertex3f(cos(theta+M_PI/2.0) * radius+xx, sin(theta+M_PI/2.0) * radius+yy, 0.010);
	glEnd();

	for(int i = 0; i < grid; i++){
		double rad_1= 2.0*M_PI*i/grid+theta;
		double rad_2= 2.0*M_PI*(i+1)/grid+theta;
		glBegin(GL_LINES);
		glVertex3f(cos(rad_1) * radius+xx, sin(rad_1) * radius+yy, 0.01);
		glVertex3f(cos(rad_2) * radius+xx, sin(rad_2) * radius+yy, 0.01);
		glEnd();
	}
	glLineWidth(1.0);

}

void Graphics::move_obstacle_view(){
	glColor3f(1.0, 0.20, 0.0); //Red
	glLineWidth(2.0);

	for (int i = 0; i < move_obst.n; i++){
			glBegin(GL_LINE_LOOP);
			glVertex3f(move_obst.obst_x[i][0], move_obst.obst_y[i][0], 0);
			glVertex3f(move_obst.obst_x[i][1], move_obst.obst_y[i][1], 0);
			glVertex3f(move_obst.obst_x[i][2], move_obst.obst_y[i][2], 0);
			glVertex3f(move_obst.obst_x[i][3], move_obst.obst_y[i][3], 0);
			glVertex3f(move_obst.obst_x[i][4], move_obst.obst_y[i][4], 0);
			glEnd();
	}
	glLineWidth(1.0);
}


void Graphics::lrf_view( ROBOT &robot,LRF &lrf_data){
	//lrfのレーザーの表示
	glColor3f(0, 0.7, 1.0);
	glLineWidth(2.0);
	const int flg=1;
	if(flg==0){	//レーザ探査領域の表示
		const double length=lrf_data.leng_max;

		for(int i=0;i<lrf_data.data_num;i++){
			double xx1=(-1)*length*sin(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
			double yy1=length*cos(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;
			double xx2=(-1)*length*sin((i+1)*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
			double yy2=length*cos((i+1)*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;
			glBegin(GL_LINES);
			glVertex3f(xx1,yy1,0 );
			glVertex3f(xx2,yy2,0 );
			glEnd();
		}
		double xx1=(-1)*length*sin(lrf_data.start_rad+robot.theta)+robot.x;
		double yy1=length*cos(lrf_data.start_rad+robot.theta)+robot.y;
		double xx2=(-1)*length*sin((lrf_data.data_num)*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
		double yy2=length*cos((lrf_data.data_num)*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;
		glBegin(GL_LINES);
		glVertex3f(xx1,yy1,0 );
		glVertex3f(robot.x, robot.y,-0 );
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(xx2,yy2,0 );
		glVertex3f(robot.x, robot.y,-0 );
		glEnd();
	}
	if(flg==1){
		for(int i=0;i<lrf_data.data_num;i++){
			double length=lrf_data.leng[i];
			if(length==0.0)	length=lrf_data.leng_max;

			double xx=(-1)*length*sin(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
			double yy=length*cos(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;


			glBegin(GL_LINES);
			glVertex3f(xx,yy,0 );
			glVertex3f(robot.x, robot.y,-0 );
			glEnd();
		}
	}
	glLineWidth(1.0);


	//レーザとOBJECTの交点の座標
	glPointSize(2.0);
	glColor3f(0, 0.0, 1);

	for(int i=0;i<lrf_data.data_num-1;i++){
		lrf_data.x[i]=(-1)*lrf_data.leng[i]*sin(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.x;
		lrf_data.y[i]=lrf_data.leng[i]*cos(i*lrf_data.reso+lrf_data.start_rad+robot.theta)+robot.y;
		glBegin(GL_POINTS);
		glVertex3f(lrf_data.x[i], lrf_data.y[i],0.02 );
		glEnd();
	}
	glPointSize(1.0);
	glLineWidth(2.0);
	return;
}


void Graphics::obstacle_view(){
	glLineWidth(2.0);
	glColor3f(0, 0.70, 0.30);

	for(int i=0;i<obst.n;i++){
		glBegin(GL_LINES);
		glVertex3f(obst.x1[i], obst.y1[i],0 );
		glVertex3f(obst.x2[i], obst.y2[i],0 );
		glEnd();
	}
	glLineWidth(1.0);
}
