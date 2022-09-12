#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
void Motor_task(void *pvParameters);
int motorChange(int i,int tt);
void motorArm(int dd,int ee,int ff,int gg,int hh,int flat);
void mode_judgement();
#endif 