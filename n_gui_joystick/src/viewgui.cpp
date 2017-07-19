/////////////////////////////////////////////////
////Create By IPF nemoto
/////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>
using namespace std;

#include "viewgui.h"
#include "geometory.h"

//各種OpenGL変数宣言
const int WIDTH = 300, HEIGHT = WIDTH;//ウィンドウのサイズ
int xBegin, yBegin;			//mouseのXY座標
int mButton;				//mouseのClick
float distance_gl, twist, elevation, azimuth;//カメラの回転行列
float xOrig , yOrig, zOrig;					//カメラの位置

//Robot制御のパラメータ
struct URG urg_data;
//struct OBSTACLE obst;

int gl_mode=0;//モードを切り替える。

double input_x=0.0,input_y=0.0;


int wait_sleep(int time){
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

void Graphics::main_draw() {

	//URGのレーザーの表示
	glColor3f(0.0, 0.5, 0.5);
	glLineWidth(2.0);

	glBegin(GL_LINES);
	glVertex3f(-2,0,0 );
	glVertex3f(2,0,0 );
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(0,-2,0 );
	glVertex3f(0,2,0 );
	glEnd();

	for(int i=0;i<100;i++){
		double length=1.0;
		if(length==0)length=5;

		double xx1=(-1)*length*sin(i/100.0*2*3.1415);
		double yy1=length*cos(i/100.0*2*3.1415);
		double xx2=(-1)*length*sin((i+1)/100.0*2*3.1415);
		double yy2=length*cos((i+1)/100.0*2*3.1415);

		glBegin(GL_LINES);
		glVertex3f(xx1,yy1,0 );
		glVertex3f(xx2,yy2,0 );

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

	//cout<<"x:"<<input_x<<"\t y:"<<input_y<<"\n";

	usleep(10*1000);

}

void Graphics::idle(void)
{
	glutPostRedisplay();
	#ifdef _WIN32
	Sleep(10);
	#else
	usleep(10*1000);
	#endif
}

void Graphics::myInit(char *progname)
{
	float aspect = (float) WIDTH / (float) HEIGHT;

	glutInitWindowPosition(0, 0);
	glutInitWindowSize( WIDTH, HEIGHT);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA| GLUT_DEPTH);
	glutCreateWindow(progname);
	glClearColor (0, 0, 0, 1.0);


	glutKeyboardFunc( myKbd );
	glutKeyboardUpFunc( myKbdup );
	glutSpecialFunc( mySkey );

	glutMouseFunc(myMouse);
	glutMotionFunc(myMotion);
	resetview();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, aspect, 0.000001, 100.0);//画角等視野の設定
	glMatrixMode(GL_MODELVIEW);
}

void Graphics::reshape(int ,int)
{
	glutReshapeWindow( WIDTH, HEIGHT );
}

int Graphics::GUImain(){
	//	read_obstacle("obstacle.csv",obst);
	int argc= 3;
	char *argv[]    ={"program", "-display", ":0.0"};
	glutInit ( &argc, argv );

	char *win_name="GUI_joystick";
	myInit(win_name);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutIdleFunc(idle);
	glutMainLoop();


	return 0;
}


void Graphics::gui_start(){
	th1 = SDL_CreateThread((int (*)(void *))GUImain,this);
}


void Graphics::gui_end(){
	SDL_KillThread(th1);
}


void Graphics::myMouse( int button, int state, int x, int y ){

	double obj_x=0,obj_y=0,obj_z=0;

	if (state == GLUT_DOWN) {
		switch(button) {
			case GLUT_LEFT_BUTTON:
			mButton = button;
			click_pickup(x,y,obj_x,obj_y,obj_z);
			input_y=obj_y;
			input_x=obj_x;

			break;
			case GLUT_MIDDLE_BUTTON:

			break;
			case GLUT_RIGHT_BUTTON:
			mButton = button;
			break;
		}
		xBegin = x;
		yBegin = y;
	}
	if (state == GLUT_UP) {
		switch(button) {
			case GLUT_LEFT_BUTTON:
			input_y=0;
			input_x=0;
			break;
		}
	}

	return ;
}


void Graphics::myMotion( int x, int y )
{
	double obj_x=0,obj_y=0,obj_z=0;
	int xDisp, yDisp;

	xDisp = x - xBegin;
	yDisp = y - yBegin;

	switch (mButton) {
		case GLUT_LEFT_BUTTON:

		click_pickup(x,y,obj_x,obj_y,obj_z);
		input_y=obj_y;
		input_x=obj_x;


		//azimuth -= (float) xDisp/2.0;
		//elevation -= (float) yDisp/2.0;
		//	xOrig += (float) xDisp/50.0;
		//	zOrig -= (float) yDisp/50.0;

		break;
		case GLUT_RIGHT_BUTTON:
		distance_gl -= (float) xDisp/50.0;
		if(distance_gl<0.000001)distance_gl=0.000001;

		break;
	}
	xBegin = x;
	yBegin = y;


	glutPostRedisplay();
}


void Graphics::myKbdup( unsigned char key, int x, int y )
{

	switch( key ) {

		case 'w':	//ROBOTの移動指令
		//	if(gl_mode==0)		myrobot.v=0.0;
		break;
		case 's':	//ROBOTの移動指令
		//	if(gl_mode==0)		myrobot.v=-0.0;
		break;
		case 'a':	//ROBOTの移動指令
		//	if(gl_mode==0)		myrobot.w=-0.0;
		break;
		case 'd':	//ROBOTの移動指令
		//	if(gl_mode==0)		myrobot.w=+0.0;
		break;



	}

}
void Graphics::myKbd( unsigned char key, int x, int y )
{
	switch( key ) {
		case 0x1B:	//終了
		//	exit(0);
		break;

		case 'q':	//視点をリセット
		//	exit(0);;
		break;

		case 'z':	//視点をリセット


		break;
		case 'x':	//視点をリセット


		break;

		case 'w':	//ROBOTの移動指令
		//	if(gl_mode==0)		myrobot.v=0.30;

		break;
		case 's':	//ROBOTの移動指令
		//	if(gl_mode==0)	myrobot.v=-0.30;
		break;
		case 'a':	//ROBOTの移動指令
		//	if(gl_mode==0)	myrobot.w=-0.10;
		break;
		case 'd':	//ROBOTの移動指令
		//	if(gl_mode==0)	myrobot.w=+0.10;
		break;

	}
}



void Graphics::mySkey( int key, int x, int y )
{
	switch( key ) {
		/*
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
		*/
	}
	glutPostRedisplay();
}


void Graphics::resetview( void )
{
	distance_gl = 5.0;
	twist = 0.0;
	elevation = 0.0;
	azimuth = 0.0;
}



void Graphics::polarview( void )
{
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
