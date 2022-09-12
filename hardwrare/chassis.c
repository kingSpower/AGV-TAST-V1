#include "chassis.h"
#include "math.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
//struct PID testMotor;
#define pi 3.1415926
#define ABS(x)		((x>0)? (x): (-x)) 
#define ABSS(x)		((x<200)? (x): (x-256))
int c1=0,c2=0,c3=0,c4=0,c5=0,c6=0;
extern struct PID chassis_m3508a,
									chassis_m3508b,
									chassis_m3508c,
									chassis_m3508d,
									chassis_m3508A,
									chassis_m3508B,
									chassis_m3508C,
									chassis_m3508D,   
									chassis_m3508e,
									chassis_m3508E;
struct angle steer; 
extern struct motorData m3508a,m3508b,m3508c,m3508d,m3508e;
struct chssisOut motor_out;
extern volatile  RC_Ctl_t RC_Ctl;
float r3,r2,r1,r0,R3,R2;
int vx,vy,vxy,flag=0,C,A=100,x,y,open,yaw,drtX=0,drtY=0,xy;
float left1,left2,right1,right2;
int location1,location2,updown=0;
char res;
int ms_flag,err_x,err_y,speed_flat=0,location_flat,a,auto_flat=0;//标识符
extern int rebuf[8],rrebuf[14];
int a1=190,a2=0,a3=0,a4=0,dta1=0,dta2=0,dta3=0,dta4=0;
int color[6]={0,0,0,0,0,0},code[6]={0,0,0,0,0,0};
int TD=150,TE=150,TF=150,TG=150,TH=150,Td=150,Te=150,Tf=150,Tg=150,Th=150;
extern int tD,tF,tE,tG,tH;
#ifdef chassis_mode1


void get_down(int i,int j,int k) // 拿下层
{
//  re_XY(0);
	motorArm(150,150,150,150,150,0);
	if(i==1)
	{
	motorArm(150,150,150,150,150,1);
	motorArm(120,150,150,150,150,2);
	motorArm(115,180,55,220,150,3); 
	motorArm(133,180,55,220,150,4);
	
	motorArm(133,180,85,190,150,5);
	motorArm(133,235,115,185,150,6);
	motorArm(133,235,115,185,198,7);
	motorArm(133,180,55,220,198,8);
  motorArm(115,180,55,220,198,9);
	motorArm(120,150,150,150,198,10);
	motorArm(150,150,150,150,198,11);
		
		motorArm(150,150,150,150,198,12);
		motorArm(150,150,150,150,198,13);
		motorArm(165,165,150,150,198,14);
		motorArm(165,165,230,140,198,15);
		motorArm(165,165,230,140,150,16);
		motorArm(165,150,150,150,150,17);
	}
	if(i==2)
	{
		motorArm(150,150,150,150,150,1);
	motorArm(120,150,150,150,150,2);
	motorArm(115,180,55,220,150,3); 
	motorArm(150,180,55,220,150,4);
	
	motorArm(150,180,85,190,150,5);
	motorArm(150,225,105,195,150,6);
	motorArm(150,225,105,195,198,7);
	motorArm(150,180,55,220,198,8);
  motorArm(115,180,55,220,198,9);
	motorArm(120,150,150,150,198,10);
	motorArm(150,150,150,150,198,11);
		
		motorArm(150,150,150,150,198,12);
		motorArm(150,150,150,150,198,13);
		motorArm(165,165,150,150,198,14);
		motorArm(165,165,230,140,198,15);
		motorArm(165,165,230,140,150,16);
		motorArm(165,150,150,150,150,17);
	}
	
	if(i==3)
	{
		motorArm(150,150,150,150,150,1);
	motorArm(120,150,150,150,150,2);
	motorArm(115,180,55,220,150,3); 
	motorArm(165,180,55,220,150,4);
	
	motorArm(165,180,85,190,150,5);
	motorArm(165,235,115,185,150,6);
	motorArm(165,235,115,185,198,7);
	motorArm(165,180,55,220,198,8);
  motorArm(185,180,55,220,198,9);
	motorArm(180,150,150,150,198,10);
	motorArm(150,150,150,150,198,11);
		
		motorArm(150,150,150,150,198,12);
		motorArm(150,150,150,150,198,13);
		motorArm(165,165,150,150,198,14);
		motorArm(165,165,230,140,198,15);
		motorArm(165,165,230,140,150,16);
		motorArm(165,150,150,150,150,17);
	}
		if(j==1)
	{
	motorArm(150,150,150,150,150,18);
	motorArm(120,150,150,150,150,19);
	motorArm(115,180,55,220,150,20); 
	motorArm(133,180,55,220,150,21);
	
	motorArm(133,180,85,190,150,22);
	motorArm(133,235,115,185,150,23);
	motorArm(133,235,115,185,198,24);
	motorArm(133,180,55,220,198,25);
  motorArm(115,180,55,220,198,26);
	motorArm(120,150,150,150,198,27);
	motorArm(150,150,150,150,198,28);
		
		motorArm(150,150,150,150,195,29);
		motorArm(150,150,150,150,195,30);
motorArm(148,170,150,150,198,31);
motorArm(148,170,233,140,198,32);
motorArm(148,170,233,140,150,33);
motorArm(148,150,150,150,150,34);
	}
	if(j==2)
	{
		motorArm(150,150,150,150,150,18);
	motorArm(120,150,150,150,150,19);
	motorArm(115,180,55,220,150,20); 
	motorArm(150,180,55,220,150,21);
	
	motorArm(150,180,85,190,150,22);
	motorArm(150,225,105,195,150,23);
	motorArm(150,225,105,195,198,24);
	motorArm(150,180,55,220,198,25);
  motorArm(115,180,55,220,198,26);
	motorArm(120,150,150,150,198,27);
	motorArm(150,150,150,150,198,28);
		
		motorArm(150,150,150,150,198,29);
		motorArm(150,150,150,150,198,30);
motorArm(148,170,150,150,198,31);
motorArm(148,170,233,140,198,32);
motorArm(148,170,233,140,150,33);
motorArm(148,150,150,150,150,34);
	}
	
	if(j==3)
	{
		motorArm(150,150,150,150,150,18);
	motorArm(120,150,150,150,150,19);
	motorArm(115,180,55,220,150,20); 
	motorArm(165,180,55,220,150,21);
	
	motorArm(165,180,85,190,150,22);
	motorArm(165,235,115,185,150,23);
	motorArm(165,235,115,185,198,24);
	motorArm(165,180,55,220,198,25);
  motorArm(185,180,55,220,198,26);
	motorArm(180,150,150,150,198,27);
	motorArm(150,150,150,150,198,28);
		
		motorArm(150,150,150,150,198,29);
		motorArm(150,150,150,150,198,30);
motorArm(148,170,150,150,195,31);
motorArm(148,170,233,140,195,32);
motorArm(148,170,233,140,150,33);
motorArm(148,150,150,150,150,34);
	}
		if(k==1)
	{
	motorArm(150,150,150,150,150,35);
	motorArm(120,150,150,150,150,36);
	motorArm(115,180,55,220,150,37); 
	motorArm(133,180,55,220,150,38);
	
	motorArm(133,180,85,190,150,39);
	motorArm(133,235,115,185,150,40);
	motorArm(133,235,115,185,198,41);
	motorArm(133,180,55,220,198,42);
  motorArm(115,180,55,220,198,43);
	motorArm(120,150,150,150,198,44);
	motorArm(150,150,150,150,198,45);
		
		motorArm(150,150,150,150,198,46);
		motorArm(150,150,150,150,198,47);
motorArm(131,168,150,150,195,48);
motorArm(131,168,232,140,195,49);
motorArm(131,168,232,140,150,50);
motorArm(131,150,150,150,150,51);
	}
	if(k==2)
	{
		motorArm(150,150,150,150,150,35);
	motorArm(120,150,150,150,150,36);
	motorArm(115,180,55,220,150,37); 
	motorArm(150,180,55,220,150,38);
	
	motorArm(150,180,85,190,150,39);
	motorArm(150,225,105,195,150,40);
	motorArm(150,225,105,195,198,41);
	motorArm(150,180,55,220,198,42);
  motorArm(115,180,55,220,198,43);
	motorArm(120,150,150,150,198,44);
	motorArm(150,150,150,150,198,45);
		
		motorArm(150,150,150,150,198,46);
		motorArm(150,150,150,150,198,47);
motorArm(131,168,150,150,195,48);
motorArm(131,168,232,140,195,49);
motorArm(131,168,232,140,150,50);
motorArm(131,150,150,150,150,51);
	}
	
	if(k==3)
	{
		motorArm(150,150,150,150,150,35);
	motorArm(120,150,150,150,150,36);
	motorArm(115,180,55,220,150,37); 
	motorArm(165,180,55,220,150,38);
	
	motorArm(165,180,85,190,150,39);
	motorArm(165,235,115,185,150,40);
	motorArm(165,235,115,185,198,41);
	motorArm(165,180,55,220,198,42);
  motorArm(185,180,55,220,198,43);
	motorArm(180,150,150,150,198,44);
	motorArm(150,150,150,150,198,45);
		
		motorArm(150,150,150,150,198,46);
		motorArm(150,150,150,150,198,47);
motorArm(131,168,150,150,198,48);
motorArm(131,168,232,140,198,49);
motorArm(131,168,232,140,150,50);
motorArm(131,150,150,150,150,51);
	}
//	re_XY(52);
	xy_change(1.5,5.5,-1,52);
	xy_change(1.5,5.5,0,53);
		re_XY(54);
	rude_quit(55);
 }
void vdelay(int i,int flat)
{
if(flag==flat)
	{
		vTaskDelay(i);
		flag=flat+1;
	}
}
void get_high_test(int i,int j ,int k)//拿上层
{
//	motorArm(150,150,150,150,150,0);
		re_XY(0);
if(i==1)
{
motorArm(150,150,150,150,150,1);
motorArm(125,130,78,150,150,2);
motorArm(125,130,78,150,195,3);
motorArm(125,150,150,150,195,4);
motorArm(165,165,150,150,195,5);
motorArm(165,165,230,140,195,6);
motorArm(165,165,230,140,150,7);
motorArm(165,150,150,150,150,8);
}
if(i==2)
{
motorArm(150,150,150,150,150,1);
motorArm(150,80,60,120,150,2);
motorArm(150,80,60,120,195,3);
motorArm(150,150,150,150,195,4);
motorArm(165,165,150,150,195,5);
motorArm(165,165,230,140,195,6);
motorArm(165,165,230,140,150,7);
	motorArm(165,150,150,150,150,8);
}
if(i==3)
{
motorArm(150,150,150,150,150,1);
motorArm(180,130,78,150,150,2);
motorArm(180,130,78,150,195,3);
motorArm(180,150,150,150,195,4);
motorArm(165,165,150,150,195,5);
motorArm(165,165,230,140,195,6);
motorArm(165,165,230,140,150,7);
	motorArm(165,150,150,150,150,8);
}
if(j==1)
{
motorArm(150,150,150,150,150,9);
motorArm(125,130,78,150,150,10);
motorArm(125,130,78,150,195,11);
motorArm(125,150,150,150,195,12);
motorArm(148,170,150,150,195,13);
motorArm(148,170,233,140,195,14);
motorArm(148,170,233,140,150,15);
motorArm(148,150,150,150,150,16);
}
if(j==2)
{
motorArm(150,150,150,150,150,9);
motorArm(150,80,60,120,150,10);
motorArm(150,80,60,120,195,11);
motorArm(150,150,150,150,195,12);
//motorArm(148,165,150,150,195,13);
//motorArm(148,165,228,140,195,14);
//motorArm(148,165,228,140,150,15);
motorArm(148,170,150,150,195,13);
motorArm(148,170,233,140,195,14);
motorArm(148,170,233,140,150,15);
motorArm(148,150,150,150,150,16);
}
if(j==3)
{
motorArm(150,150,150,150,150,9);
motorArm(180,130,78,150,150,10);
motorArm(180,130,78,150,195,11);
motorArm(180,150,150,150,195,12);
motorArm(148,170,150,150,195,13);
motorArm(148,170,233,140,195,14);
motorArm(148,170,233,140,150,15);
motorArm(148,150,150,150,150,16);
}if(k==1)
{
motorArm(150,150,150,150,150,17);
motorArm(125,130,78,150,150,18);
motorArm(125,130,78,150,195,19);
motorArm(125,150,150,150,195,20);
motorArm(131,168,150,150,195,21);
motorArm(131,168,232,140,195,22);
motorArm(131,168,232,140,150,23);
motorArm(131,150,150,150,150,24);
}
if(k==2)
{
motorArm(150,150,150,150,150,17);
motorArm(150,80,60,120,150,18);
motorArm(150,80,60,120,195,19);
motorArm(150,150,150,150,195,20);
motorArm(131,168,150,150,195,21);
motorArm(131,168,232,140,195,22);
motorArm(131,168,232,140,150,23);
motorArm(131,150,150,150,150,24);
}
if(k==3)
{
motorArm(150,150,150,150,150,17);
motorArm(180,130,78,150,150,18);
motorArm(180,130,78,150,195,19);
motorArm(180,150,150,150,195,20);
motorArm(131,168,150,150,195,21);
motorArm(131,168,232,140,195,22);
motorArm(131,168,232,140,150,23);
motorArm(131,150,150,150,150,24);
}

		re_XY(25);
xy_change(1.5,5.5,-1,26);
//	  re_XY(8);
xy_change(1.5,5.5,0,27);
	  re_XY(28);

//	  re_XY(16);
		rude_quit(29);
}
void move1()
{
  xy_change(3.5,5.5,0,0);
	re_XY(1);
  xy_change(3.5,6.5,0,2);
	re_XY(3);
	xy_change(3.5,5.95,0,4);

	rude_quit(5);
}
void move2()
{
  xy_change(3.5,3.5,0,0);
	re_XY(1);
  xy_change(5.5,3.5,0,2);
	re_XY(3);
	xy_change(5.5,3.5,1,4);
  re_XY(5);
	xy_change(5.5,3.6,1,6);
	rude_quit(7);
}
void move3()
{

	re_XY(0);
	xy_change(3.5,3.5,0,1);
	re_XY(2);
	xy_change(1.5,5.5,0,3);
	re_XY(4);
	xy_change(1.5,5.5,-1,5);
	re_XY(6);
	xy_change(1.5,6.1,-1,7);
	rude_quit(8);
}


void down_high_test(int i,int j, int k)//放粗加工 i取托1 j取托2 k取托3
{
//		re_XY(0);
//		motorArm(150,150,150,150,150,0);
		if(i==1)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(168,240,140,165,198,6);
		vdelay(1000,7);
    motorArm(168,240,140,165,150,8);
		motorArm(168,150,140,165,150,9);	
		}
		if(i==2)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(151,240,140,165,198,6);
		vdelay(1000,7);
    motorArm(151,215,98,200,150,8);
		motorArm(151,150,98,200,150,9);	
		}
		if(i==3)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(138,240,130,165,198,6);
		vdelay(1000,7);
    motorArm(134,250,170,160,150,8);
		motorArm(134,150,160,160,150,9);	
		}
		if(j==1)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(168,240,140,165,198,16);
		vdelay(1000,17);
    motorArm(168,240,140,165,150,18);
		motorArm(168,150,140,165,150,19);	
		}
		if(j==2)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(151,215,98,200,198,16);
			vdelay(1000,17);
    motorArm(151,215,98,200,150,18);
		motorArm(151,150,98,200,150,19);	
		}
		if(j==3)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(134,240,130,165,198,16);
			vdelay(1000,17);
    motorArm(134,250,170,160,150,18);
		motorArm(134,150,160,160,150,19);	
		}
		if(k==1)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(168,240,140,165,198,26);
			vdelay(1000,27);
    motorArm(168,240,140,165,150,28);
		motorArm(168,150,140,165,150,29);	
		}
		if(k==2)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(151,215,98,200,198,26);
			vdelay(1000,27);
    motorArm(151,215,98,200,150,28);
		motorArm(151,150,98,200,150,29);	
		}
		if(k==3)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(134,240,130,165,198,26);
			vdelay(1000,27);
    motorArm(134,250,170,160,150,28);
		motorArm(134,150,160,160,150,29);	
		}
		if(i==1)
		{
		motorArm(150,150,150,150,150,30);
		motorArm(168,150,150,150,150,31);
		motorArm(168,240,140,165,150,32);
		motorArm(168,240,140,165,198,33);
		motorArm(168,150,140,165,198,34);	
	/*************************************/		
    motorArm(165,165,230,140,198,35);	
		motorArm(165,165,230,140,150,36);		
		}
		if(i==2)
		{
		motorArm(150,150,150,150,150,30);
		motorArm(151,150,150,150,150,31);
		motorArm(151,215,98,200,150,32);
		motorArm(151,215,98,200,198,33);
		motorArm(151,150,98,200,198,34);	
	/*************************************/		
    motorArm(165,165,230,140,198,35);	
		motorArm(165,165,230,140,150,36);		
		}
		if(i==3)
		{
		motorArm(150,150,150,150,150,30);
		motorArm(134,150,150,150,150,31);
		motorArm(134,250,150,170,150,32);
		motorArm(134,250,150,170,198,33);
		motorArm(134,150,160,160,198,34);	
	/*************************************/		
    motorArm(165,165,230,140,198,35);	
		motorArm(165,165,230,140,150,36);		
		}
		
				if(j==1)
		{
		motorArm(150,150,150,150,150,37);
		motorArm(168,150,150,150,150,38);
		motorArm(168,240,140,165,150,39);
		motorArm(168,240,140,165,198,40);
		motorArm(168,150,140,165,198,41);	
	/*************************************/		
    motorArm(148,165,228,140,198,42);	
		motorArm(148,165,228,140,150,43);		
		}
		if(j==2)
		{
		motorArm(150,150,150,150,150,37);
		motorArm(151,150,150,150,150,38);
		motorArm(151,215,98,200,150,39);
		motorArm(151,215,98,200,198,40);
		motorArm(151,150,98,200,198,41);	
	/*************************************/		
    motorArm(148,165,228,140,198,42);	
		motorArm(148,165,228,140,150,43);		
		}
		if(j==3)
		{
		motorArm(150,150,150,150,150,37);
		motorArm(134,150,150,150,150,38);
		motorArm(134,250,150,170,150,39);
		motorArm(134,250,150,170,198,40);
		motorArm(134,150,160,160,198,41);	
	/*************************************/		
    motorArm(148,165,228,140,198,42);	
		motorArm(148,165,228,140,150,43);		
		}
		
		
		if(k==1)
		{
		motorArm(150,150,150,150,150,44);
		motorArm(168,150,150,150,150,45);
		motorArm(168,240,140,165,150,46);
		motorArm(168,240,140,165,198,47);
		motorArm(168,150,140,165,198,48);	
	/*************************************/		
    motorArm(131,168,232,140,198,49);	
		motorArm(131,168,232,140,150,50);		
		}
		if(k==2)
		{
		motorArm(150,150,150,150,150,44);
		motorArm(151,150,150,150,150,45);
		motorArm(151,215,98,200,150,46);
		motorArm(151,215,98,200,198,47);
		motorArm(151,150,98,200,198,48);	
	/*************************************/		
    motorArm(131,168,232,140,198,49);	
		motorArm(131,168,232,140,150,50);	
		}
		if(k==3)
		{
		motorArm(150,150,150,150,150,44);
		motorArm(134,150,150,150,150,45);
		motorArm(134,250,150,170,150,46);
		motorArm(134,250,150,170,198,47);
		motorArm(134,150,160,160,198,48);	
	/*************************************/		
    motorArm(131,168,232,140,198,49);	
		motorArm(131,168,232,140,150,50);	
		}
		rude_quit(51);
}
void down_low_test(int i,int j,int k)//放精加工
{
//		re_XY(0);

		
		if(i==1)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(168,240,140,165,198,6);
		vdelay(1000,7);
    motorArm(168,240,140,165,150,8);
		motorArm(168,150,140,165,150,9);	
		}
		if(i==2)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(151,240,140,165,198,6);
		vdelay(1000,7);
    motorArm(151,215,98,200,150,8);
		motorArm(151,150,98,200,150,9);	
		}
		if(i==3)
		{
		motorArm(150,150,150,150,150,0);
		motorArm(165,170,150,150,150,1);
		motorArm(165,170,240,140,150,2);
		motorArm(165,170,240,140,198,3);
		motorArm(165,170,200,140,198,4);
/*************************************/		
    motorArm(150,150,150,150,198,5);
		motorArm(138,240,130,165,198,6);
		vdelay(1000,7);
    motorArm(134,250,170,160,150,8);
		motorArm(134,150,160,160,150,9);	
		}
		if(j==1)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(168,240,140,165,198,16);
		vdelay(1000,17);
    motorArm(168,240,140,165,150,18);
		motorArm(168,150,140,165,150,19);	
		}
		if(j==2)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(151,215,98,200,198,16);
			vdelay(1000,17);
    motorArm(151,215,98,200,150,18);
		motorArm(151,150,98,200,150,19);	
		}
		if(j==3)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,198,13);
		motorArm(148,160,200,125,198,14);
/*************************************/		
    motorArm(150,150,150,150,198,15);
		motorArm(134,240,130,165,198,16);
			vdelay(1000,17);
    motorArm(134,250,170,160,150,18);
		motorArm(134,150,160,160,150,19);	
		}
		if(k==1)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(168,240,140,165,198,26);
			vdelay(1000,27);
    motorArm(168,240,140,165,150,28);
		motorArm(168,150,140,165,150,29);	
		}
		if(k==2)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(151,215,98,200,198,26);
			vdelay(1000,27);
    motorArm(151,215,98,200,150,28);
		motorArm(151,150,98,200,150,29);	
		}
		if(k==3)
		{
		motorArm(150,150,150,150,150,20);
		motorArm(131,170,150,150,150,21);
		motorArm(131,170,240,140,150,22);
		motorArm(131,170,240,140,198,23);
		motorArm(131,170,200,140,198,24);
/*************************************/		
    motorArm(150,150,150,150,198,25);
		motorArm(134,240,130,165,198,26);
			vdelay(1000,27);
    motorArm(134,250,170,160,150,28);
		motorArm(134,150,160,160,150,29);	
		}
	xy_change(5.5,3.5,1,30);	
	xy_change(5.5,3.5,0,31);
	re_XY(32);
		rude_quit(33);
}
void down_low_test_second(int i,int j,int k)//放精加工上层
{
	re_XY(0);
		if(i==1)
		{
		motorArm(150,150,150,150,150,1);
		motorArm(165,170,150,150,150,2);
		motorArm(165,170,240,140,150,3);
		motorArm(165,170,240,140,195,4);
		motorArm(165,170,200,140,195,5);
/*************************************/		
    motorArm(150,150,150,150,195,6);
		motorArm(168,220,140,150,195,7);
    motorArm(168,220,140,150,150,8);
		motorArm(168,150,140,150,150,9);	
		}
		if(i==2)
		{
		motorArm(150,150,150,150,150,1);
		motorArm(165,170,150,150,150,2);
		motorArm(165,170,240,140,150,3);
		motorArm(165,170,240,140,195,4);
		motorArm(165,170,200,140,195,5);
/*************************************/		
    motorArm(150,150,150,150,195,6);
		motorArm(150,195,112,150,195,7);
    motorArm(150,195,112,150,150,8);
		motorArm(150,150,112,150,150,9);	
		}
		if(i==3)
		{
		motorArm(150,150,150,150,150,1);
		motorArm(165,170,150,150,150,2);
		motorArm(165,170,240,140,150,3);
		motorArm(165,170,240,140,195,4);
		motorArm(165,170,200,140,195,5);
/*************************************/		
    motorArm(150,150,150,150,195,6);
		motorArm(134,220,140,150,195,7);
    motorArm(134,220,140,150,150,8);
		motorArm(134,150,160,150,150,9);	
		}
		if(j==1)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,195,13);
		motorArm(148,160,200,125,195,14);
/*************************************/		
    motorArm(150,150,150,150,195,15);
		motorArm(168,220,140,150,195,16);
    motorArm(168,220,140,150,150,17);
		motorArm(168,150,140,150,150,18);	
		}
		if(j==2)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,195,13);
		motorArm(148,160,200,125,195,14);
/*************************************/		
    motorArm(150,150,150,150,195,15);
		motorArm(150,195,112,150,195,16);
    motorArm(150,195,112,150,150,17);
		motorArm(150,150,112,150,150,18);		
		}
		if(j==3)
		{
		motorArm(150,150,150,150,150,10);
		motorArm(148,160,150,150,150,11);
		motorArm(148,160,240,125,150,12);
		motorArm(148,160,240,125,195,13);
		motorArm(148,160,200,125,195,14);
/*************************************/		
    motorArm(150,150,150,150,195,15);
		motorArm(134,220,140,150,195,16);
    motorArm(134,220,140,150,150,17);
		motorArm(134,150,160,150,150,18);
		}
		if(k==1)
		{
		motorArm(150,150,150,150,150,19);
		motorArm(131,170,150,150,150,20);
		motorArm(131,170,240,140,150,21);
		motorArm(131,170,240,140,195,22);
		motorArm(131,170,200,140,195,23);
/*************************************/		
    motorArm(150,150,150,150,195,24);
		motorArm(168,220,140,150,195,25);
    motorArm(168,220,140,150,150,26);
		motorArm(168,150,140,150,150,27);	
		}
		if(k==2)
		{
		motorArm(150,150,150,150,150,19);
		motorArm(131,170,150,150,150,20);
		motorArm(131,170,240,140,150,21);
		motorArm(131,170,240,140,195,22);
		motorArm(131,170,200,140,195,23);
/*************************************/		
    motorArm(150,150,150,150,195,24);
		motorArm(150,195,112,150,195,25);
    motorArm(150,195,112,150,150,26);
		motorArm(150,150,112,150,150,27);	
		}
		if(k==3)
		{
		motorArm(150,150,150,150,150,19);
		motorArm(131,170,150,150,150,20);
		motorArm(131,170,240,140,150,21);
		motorArm(131,170,240,140,195,22);
		motorArm(131,170,200,140,195,23);
/*************************************/		
    motorArm(150,150,150,150,195,24);
		motorArm(134,220,140,150,195,25);
    motorArm(134,220,140,150,150,26);
		motorArm(134,150,160,150,150,27);	
		}
	xy_change(5.5,3.5,0,28);
	re_XY(29);
		rude_quit(30);
}
void test()
{
	
	re_XY(0);
	xy_change(0,0.5,0,1);
	motorArm(150,150,150,150,150,2);
	motorArm(120,150,150,150,150,3);
	motorArm(115,180,55,220,150,4); 
	motorArm(150,180,55,220,150,5);
	
	motorArm(150,180,85,190,150,6);
	motorArm(150,225,105,195,150,7);
	motorArm(150,225,105,195,195,8);
	motorArm(150,180,55,220,195,9);
  motorArm(115,180,55,220,195,10);
	motorArm(120,150,150,150,195,11);
	motorArm(150,150,150,150,195,12);
//	motorArm(150,180,80,190,150,1);
//	motorArm(135,210,125,180,150,3);
//	motorArm(135,210,125,180,195,4);
	
	rude_quit(13);
}
void start()
{
//    TIM_SetCompare1(TIM5,200);
		xy_change(0.5,1.5,0,0);
		re_XY(1);
	  xy_change(0.5,2,0,2);
	  xy_change(0.1,2,0,3);
	  xy_change(0.3,2,0,4);
	  number(5);
	  xy_change(0.5,2,0,6);
	  xy_change(1.5,5.5,0,7);
	  re_XY(8);
	  get_color(9);
	  xy_change(1.5,5.5,-1,10);
	  re_XY(11);
	  xy_change(1.5,6.5,-1,12);
	  re_XY(13);
	  rude_quit(14);
	  
}
void get_color(int flat)
{
		if(flag==flat)
			{
				vTaskDelay(500);
			color[0]=rrebuf[8];
			color[1]=rrebuf[9];
			color[2]=rrebuf[10];
			color[3]=rrebuf[11];
			color[4]=rrebuf[12];
			color[5]=rrebuf[13];

				c1=AUTO_rode1(0);
				c2=AUTO_rode1(1);
				c3=AUTO_rode1(2);
				c4=AUTO_rode2(3);
				c5=AUTO_rode2(4);
				c6=AUTO_rode2(5);

				if(color[5]!=0)
				flag=flat+1;
			}


}
int AUTO_rode1(int c)
{
	int i,color_location;
	for(i=0;i<=2;i++)
	{
	if(color[i]==code[c])
		color_location=i+1;
	}
	return color_location;

}
int AUTO_rode2(int c)
{
int i,color_location;
	for(i=3;i<=5;i++)
	{
	if(color[i]==code[c])
		color_location=i;
	}
	return color_location;
}//得到物块位置
void number(int flat)
{
	if(flag==flat)
	{
		if(code[5]!=0)
		{
			vTaskDelay(500);
   		oled_clear(Pen_Clear);
//      vTaskDelay(500);
		oled_showchar1(0,10,code[0]);
		oled_showchar1(18,10,code[1]);
		oled_showchar1(36,10,code[2]);
	  oled_showchar1(54,10,4);
		oled_showchar1(72,10,code[3]);
		oled_showchar1(90,10,code[4]);
		oled_showchar1(110,10,code[5]);

		    oled_refresh_gram();

		flag=flat+1;
		}
	}
}
void rude_quit(int flat)//状态重置
{
if(flag==flat)
	{
		flag=0;
		auto_flat+=1;
	}


}
void AUTO()
{

	if(auto_flat==0)
	{

	start();

	}

	if(auto_flat==1)
	{

		get_high_test(c1,c2 ,c3);
		
	}

	if(auto_flat==2)
	{
   move1();
	}

	if(auto_flat==3)
	{
		down_high_test(code[0],code[1],code[2]);
	}
	
	if(auto_flat==4)
	{
		move2();
	}
	if(auto_flat==5)
	{
   down_low_test(code[0],code[1],code[2]);
	}

	if(auto_flat==6)
	{
		move3();
	}
		if(auto_flat==7)
	{
		get_down(c4,c5,c6);
	}
		if(auto_flat==8)
	{
		
		move1();
   }	
	if(auto_flat==9)
	{
		down_high_test(code[3],code[4],code[5]);
	}
	if(auto_flat==10)
	{
		move2();
	}
	if(auto_flat==11)
	{
		down_low_test_second(code[3],code[4],code[5]);
	}
	if(auto_flat==12)
	{
		xy_change(6.5,0.5,0,0);
			 re_XY(1);
		 xy_change(7,0,0,2);   
	}
}

void re_XY(int flat)
{
	
	if(flag==flat)
	{

	location_flat=0;
	speed_flat=1;
	if(a==0)
	{
	a1=m3508a.G_Angle;
	a2=m3508b.G_Angle;
	a3=m3508c.G_Angle;
	a4=m3508d.G_Angle;
		a+=1;
	}

	err_y=ABSS(rebuf[3]);
	err_x=ABSS(rebuf[2]);
		speed_flat=1;
		if((err_y>6))
		R3=200;
		else if((err_y<-6))
		R3=-200;
		else
		R3=0;
		if((err_x>6))
		R2=-200;
		else if((err_x<-6))
		R2=200;
		else
		R2=0;
		xy=ABSS(rebuf[4])*-100;
//		vTaskDelay(200);

	
	if((R2==0)&&(R3==0)&&(xy==0))
	{
		
//	  speed_flat=0;
//		flag=0;
		flag=flat+1;
		a=0;
//		auto_flat+=1;
			dta1=m3508a.G_Angle-a1+dta1;
			dta2=m3508b.G_Angle-a2+dta2;
			dta3=m3508c.G_Angle-a3+dta3;
			dta4=m3508d.G_Angle-a4+dta4;
	}
}
}
void re_Z(int flat)
{
	
	if(flag==flat)
	{

	location_flat=0;
	speed_flat=1;
	if(a==0)
	{
	a1=m3508a.G_Angle;
	a2=m3508b.G_Angle;
	a3=m3508c.G_Angle;
	a4=m3508d.G_Angle;
		a+=1;
	}


		xy=ABSS(rebuf[4])*100;
//		vTaskDelay(200);

	
	if(xy==0)
	{
		
//	  speed_flat=0;
//		flag=0;
		flag=flat+1;
		a=0;
//		auto_flat+=1;
			dta1=m3508a.G_Angle-a1+dta1;
			dta2=m3508b.G_Angle-a2+dta2;
			dta3=m3508c.G_Angle-a3+dta3;
			dta4=m3508d.G_Angle-a4+dta4;
	}
}
}
void chassis_task(void *pvParameters)
{  


	while(1)
	{  

			GPIO_SetBits(GPIOG, GPIO_Pin_2 );
			TIM_SetCompare1(TIM5,TD);//D
  	  TIM_SetCompare1(TIM4,TH);//H
		  TIM_SetCompare2(TIM4,TG);//G
  	  TIM_SetCompare3(TIM4,TF);//F
		  TIM_SetCompare4(TIM4,TE);//E 
//		motor_out.m3508e=pid_palc(&chassis_m3508E,updown,m3508e.G_Angle);
//		motor_out.m3508E=pid_palc(&chassis_m3508e,motor_out.m3508e,m3508e.Speed);
//		Run(motor_out.m3508E,0,0,0,0x1ff);
//		if(RC_Ctl.rc.s1==0x01)    
//		{
    if(speed_flat==1)
		{
		    left1=R3+R2+xy;
        left2=R3-R2+xy;
        right1=R2-R3+xy;
        right2=-R2-R3+xy;
		
		    motor_out.m3508B=pid_palc(&chassis_m3508b,left2,m3508b.Speed);
        motor_out.m3508C=pid_palc(&chassis_m3508c,right1,m3508c.Speed);
        motor_out.m3508D=pid_palc(&chassis_m3508d,right2,m3508d.Speed);
        motor_out.m3508A=pid_palc(&chassis_m3508a,left1,m3508a.Speed);

            Run(    
                    motor_out.m3508A,
                    motor_out.m3508B,
                    motor_out.m3508C,
                    motor_out.m3508D,
                    0x200           
            );
//			Run(motor_out.m3508E,0,0,0,0x1ff);
		
		}
		if(location_flat==1)
		{
		
		    left1=r3+r2+r0;
        left2=r3-r2+r0;
        right1=r2-r3+r0;
        right2=-r2-r3+r0;
			  motor_out.m3508b=pid_palc(&chassis_m3508B,left2+dta2,m3508b.G_Angle);
        motor_out.m3508c=pid_palc(&chassis_m3508C,right1+dta3,m3508c.G_Angle);
        motor_out.m3508d=pid_palc(&chassis_m3508D,right2+dta4,m3508d.G_Angle);
        motor_out.m3508a=pid_palc(&chassis_m3508A,left1+dta1,m3508a.G_Angle);
        motor_out.m3508B=pid_palc(&chassis_m3508b,motor_out.m3508b,m3508b.Speed);
        motor_out.m3508C=pid_palc(&chassis_m3508c,motor_out.m3508c,m3508c.Speed);
        motor_out.m3508D=pid_palc(&chassis_m3508d,motor_out.m3508d,m3508d.Speed);
        motor_out.m3508A=pid_palc(&chassis_m3508a,motor_out.m3508a,m3508a.Speed);          
            Run(    
                    motor_out.m3508A,
                    motor_out.m3508B,
                    motor_out.m3508C,
                    motor_out.m3508D,
                    0x200           
            );

		}


		GPIO_ResetBits(GPIOG, GPIO_Pin_2 );
         vTaskDelay(1);

}
}

void xy_change(float x,float y,float z,int flat)
{

	  
   if(flag==flat)
		{
		speed_flat=0;
    location_flat=1;
			r2=-369329*x;
			r3=354889*y;
			r0=-405003*z;
			location_flat=1;
			vTaskDelay(300);
			
    if(    (motor_out.m3508a==0)&&
				   (motor_out.m3508b==0)&&
			     (motor_out.m3508c==0)&&
			     (motor_out.m3508d==0))
			{

			flag=flat+1;
			}//粗位移
			
			
			
		
}
		}


void XY_CHANGE_task(void *pvParameters)
{
	vTaskDelay(3000);
     while(1)
		 {
			 
			AUTO();

		vTaskDelay(1);
		 }
}

void arm(int a1,int a2,int a3,int a4,int a5,int flat)
{
		 if(flag==flat)
		 {		
			TIM_SetCompare1(TIM5,a1);
  	  TIM_SetCompare1(TIM4,a2);
		  TIM_SetCompare2(TIM4,a3);
  	  TIM_SetCompare3(TIM4,a5);
		  TIM_SetCompare4(TIM4,a4);
			vTaskDelay(1000);
			 flag+=1;
		 }
}
#endif
