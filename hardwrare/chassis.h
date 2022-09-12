#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "main.h"
#define chassis_mode1
#define speed1 2000
typedef struct chssisOut
{
     int m3508A;
     int m3508B;
     int m3508C;
     int m3508D;
     int m3508a;
     int m3508b;
     int m3508c;
     int m3508d;
			int m3508e;
			int m3508E;
 }chssisOut;
typedef struct angle
{
  int anl1;
	int anl2;
	int anl3;
	int anl4;
	int anl5;
    
}angle;

void xy_change(float x,float y,float z,int flat);
void xy_out(void);
extern void chassis_task(void *pvParameters);
extern void XY_CHANGE_task(void *pvParameters);
void angleChange( float X,float Y, angle *an);
void arm(int a1,int a2,int a3,int a4,int a5,int flat);
void getObject();
 void throw_ojb();
void getObject_low();
void re_XY(int flat);
void action_hub(int i);
void down_action(int i);
void down_half(int i);
void start();
int AUTO_rode1(int c);
int AUTO_rode2(int c);
void rude_quit(int flat);
void get_color(int flat);
void puton_TOW(int loc,int flat);
void put_down(int loc,int flat);
void get_high_test(int i,int j ,int k);
void down_high_test(int i,int j, int k);
void get_down_high_test();
void down_low_test(int i,int j,int k);
void vdelay(int i,int flat);
void re_Z(int flat);
void move2();
void move3();
void test();
#endif 
