#ifndef _CAN1_H_
#define _CAN1_H_
#include "main.h"
typedef struct motorData
{
    int Speed;
    int Angle;
    int current;
    int temp;
    int L_Angle;
    int G_Angle; 

}motorData;

void CAN1_Configuration(void);
void Run(int16_t num1,int16_t num2,int16_t num3,int16_t num4, int16_t Std);
void CanReceiveMsgProcess(CanRxMsg *can_receive_message);
int LiMit(int input);
void DataHande(motorData *md);
#endif 

