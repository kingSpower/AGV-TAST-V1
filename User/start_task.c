#include "start_task.h"

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

#define Motor_TASK_PRIO 19
#define Motor_STK_SIZE 512
TaskHandle_t MotorTask_Handler;

#define LED_TASK_PRIO 5
#define LED_STK_SIZE 512
TaskHandle_t LEDTask_Handler;

#define XY_CHANGE_TASK_PRIO 15
#define XY_CHANGE_STK_SIZE 512
TaskHandle_t XY_CHANGETask_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();
    
    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
					
								
		xTaskCreate((TaskFunction_t)Motor_task,
                (const char *)"MotorTask",
                (uint16_t)Motor_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Motor_TASK_PRIO,
                (TaskHandle_t *)&MotorTask_Handler);
   
    xTaskCreate((TaskFunction_t)LED_task,
                (const char *)"LEDTask",
                (uint16_t)LED_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)LED_TASK_PRIO,
                (TaskHandle_t *)&LEDTask_Handler);
//  
    xTaskCreate((TaskFunction_t)XY_CHANGE_task,
                (const char *)"XY_CHANGEask",
                (uint16_t)XY_CHANGE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)XY_CHANGE_TASK_PRIO,
                (TaskHandle_t *)&XY_CHANGETask_Handler);

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t )start_task,          //������
                (const char * )"start_task",          //��������
                (uint16_t )START_STK_SIZE,            //�����ջ��С
                (void * )NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t )START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t * )&StartTask_Handler); //������
}
