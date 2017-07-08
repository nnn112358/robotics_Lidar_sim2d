/////////////////////////////////////////////////
/////////////////////////////////////////////////


#include <sys/time.h>
#include <unistd.h>
#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>
#include "geometory.h"

const int WIDTH = 600, HEIGHT = WIDTH;

class Graphics{
private:
	static void display();
	static void idle(void);
	static void myInit(string progname);
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
	static int main_proc();
	static void draw_string(string str, int w=WIDTH, int h=HEIGHT, int x0=10, int y0=10);
	static void move_obstacle_view();
	static void robot_view(struct ROBOT &robot);
	static void lrf_view( ROBOT &robot,LRF &lrf_data);
	static void obstacle_view();
	static void move_obstacle_cal(double t_now);
	static int wait_sleep(int time);

	void gui_start();
	void gui_end();
	double timer_ms();

	SDL_Thread *th1;
	SDL_Thread *th2;

	float click_Depth(int x, int y);
	static void click_pickup(int x,int y,double &ax,double &ay,double &az);

public:
	Graphics();
	virtual ~Graphics();
};
