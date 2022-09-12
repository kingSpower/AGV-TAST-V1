#include "main.h"
 

int main()
{   
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	 	TIM4_PWM_Init();
	  TIM5_PWM_Init();
    CAN1_Configuration();
    USART6_Configuration();
  	USART3_Configuration();
    USART1_Configuration();
    Power_GPIO_Init();
	
	GPIO_SPI_Init();
	SPI1_Init();	
	oled_init();
	oled_clear(Pen_Clear);
			oled_showchar1(0,10,1);
		oled_showchar1(18,10,1);
		oled_showchar1(36,10,1);
	  oled_showchar1(54,10,4);
		oled_showchar1(72,10,1);
		oled_showchar1(90,10,1);
		oled_showchar1(110,10,1);
	 oled_refresh_gram();
//	OLED_AD_GET();

    All_pid();
//    delay_ms(100);
    startTast();
    vTaskStartScheduler();
    
    while(1)
    {
     
    }
}
//void delay_ms(u32 t)
//{
//	int i, j;
//	for (i = 0; i < 1000; i++)
//		for (j = 0; j < t * 100; j++);
//}

//int LiMit(int input)
//{
//    if(input>50000)
//    {
//    input=input-65535;
//    }
//   return input;
//}
//void key_tx()
//{  
//    
//    if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)==1)
//    {  
//        
//        
//       key+=500;
//        if(key==2000)
//        key=0;
//        
//   }
//   
   

//}
