#include "motor.h"
#include "math.h"
extern int TD,TE,TF,TG,TH;
int tD=150,tF=150,tE=150,tG=150,tH=150;

int flag_arm_FIRST,flag_arm_SECOND,flag_arm;
extern int flag;
int motorChange(int i,int tt)
{
if(i>tt)
{
		tt+=1;
		vTaskDelay(6);	
}
if(i<tt)
{
		tt-=1;
		vTaskDelay(6);	
}
if(i==tt)
flag_arm=0;
 return tt;
}
void mode_judgement()
{
		if((TG==tG)&&(TF==tF)&&(TE==tE)&&(TD==tD)&&(TH==tH))
			flag_arm_FIRST=1;
//		else if((TE==tE)&&(TD==tD))
//			flag_arm_SECOND=1;
		else
		{
		flag_arm_FIRST=0;
//		flag_arm_SECOND=0;
		}
			
}

void motorArm(int dd,int ee,int ff,int gg,int hh,int flat)
{
			if(flag==flat)
	{
//			if(flag_arm_FIRST==1)
//			{
			tF=ff;
			tG=gg;
//			vTaskDelay(100);
//			}
//			if((flag_arm_SECOND==1)&&(flag_arm_FIRST==1))
//			{
			tD=dd;
			tE=ee;
		  tH=hh;
				vTaskDelay(150);
//			}
//			vTaskDelay(500);
		if(flag_arm_FIRST==1)
		{
		flag=flat+1;
		}
//		if(flag==1)
//			flag=0;
	}

}
void Motor_task(void *pvParameters)
{ 
		while(1)
		{
		
		TD=motorChange(tD,TD);
			mode_judgement();
		TE=motorChange(tE,TE);
			mode_judgement();
		TF=motorChange(tF,TF);
			mode_judgement();
		TG=motorChange(tG,TG);
			mode_judgement();
		TH=motorChange(tH,TH);	
			mode_judgement();
		}
	
}