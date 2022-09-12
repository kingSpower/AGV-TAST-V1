
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "can1.h"
#include "usart.h"
#include "pid.h"
#include "spi.h"
#include "oled.h"
#include "chassis.h"
#include "motor.h"
#include "usart6.h"
#include "power.h"
#include "TIM5.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//void delay_ms(u32 t);
int LiMit(int input);
void number(int flat);
#endif 
