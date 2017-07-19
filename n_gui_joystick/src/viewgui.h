#include <sys/time.h>
#include <unistd.h>


#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>


#include "geometory.h"


class Graphics{

private:
	static void display();
	static void idle(void);
	static void myInit(char *progname);
	static void myMouse( int button, int state, int x, int y );
	static void myMotion( int x, int y );
	static void mySkey( int key, int x, int y );
	static void myKbd( unsigned char key, int x, int y );
	static void myKbdup( unsigned char key, int x, int y );
	static void resetview( void );
	static void reshape(int ,int);
	static void main_draw();
	static void polarview( void );
	static void* start_gui(void *);
	static int GUImain();
	void gui_start();
	void gui_end();
	double timer_ms();

	SDL_Thread *th1;
//	static int POINTS;
	 float click_Depth(int x, int y);//マウスのX/Y座標からDepthを算出
	static void click_pickup(int x,int y,double &ax,double &ay,double &az);//マウスのX/Y座標からX/Y/Z座標を算出


public:
    Graphics();
    virtual ~Graphics();
};




