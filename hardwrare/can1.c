#include "can1.h"
#include "math.h"
#define ABS(x)		((x>0)? (x): (-x)) 


struct motorData m3508a,m3508b,m3508c,m3508d,m3508e,m3508f;

/*******************初始化CAN1函数*********************/
void CAN1_Configuration(void)   
{
		CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//使能CAN1时钟
	
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); //PD0是CAN1_RX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); //PD1是CAN1_TX
		
//		/**********GPIO初始化***********/
//    gpio.GPIO_Pin = GPIO_Pin_0;									
//		gpio.GPIO_Speed = GPIO_Speed_100MHz;				
//    gpio.GPIO_Mode = GPIO_Mode_AF;			//复用
//    GPIO_Init(GPIOD, &gpio);
	
		gpio.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;;
    gpio.GPIO_Mode = GPIO_Mode_AF;			//复用
    GPIO_Init(GPIOD, &gpio);
	
		/**********NVIC中断初始化***********/
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;				//设置中断通道
    nvic.NVIC_IRQChannelPreemptionPriority = 1;	//设置中断的抢占优先级为1
    nvic.NVIC_IRQChannelSubPriority = 0;				//设置中断的相应优先级为1
    nvic.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能    IRQ——中断请求
    NVIC_Init(&nvic);				//根据上面的参数初始化NVIC寄存器
       
//		nvic.NVIC_IRQChannel = CAN1_TX_IRQn;				//设置中断通道
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;	//设置中断的抢占优先级为0
//    nvic.NVIC_IRQChannelSubPriority = 0;				//设置中断的相应优先级为1
//    nvic.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能		IRQ——中断请求
//    NVIC_Init(&nvic);				//根据上面的参数初始化NVIC寄存器
    
    CAN_DeInit(CAN1);						
    CAN_StructInit(&can);
	
		/******CAN1单元设置******/
    can.CAN_TTCM = DISABLE;						//非时间触发通信模式
    can.CAN_ABOM = DISABLE;						//软件自动离线管理
    can.CAN_AWUM = DISABLE;						//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    can.CAN_NART = DISABLE;						//禁止报文自动传送
    can.CAN_RFLM = DISABLE;						//报文不锁定,新的覆盖旧的
    can.CAN_TXFP = ENABLE;						//优先级由报文标识符决定
    can.CAN_Mode = CAN_Mode_Normal;		//模式设置： mode:0,普通模式;1,回环模式；
		
		/******CAN1波特率设置******/
    can.CAN_SJW  = CAN_SJW_1tq;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
    can.CAN_BS1 = CAN_BS1_7tq;				//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
    can.CAN_BS2 = CAN_BS2_7tq;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
    can.CAN_Prescaler = 3;  					//分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &can);							//初始化CAN1
		
		can_filter.CAN_FilterNumber = 10;											//过滤器0
		can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale = CAN_FilterScale_32bit;		//32位标识符屏蔽位模式
		can_filter.CAN_FilterIdHigh = 0x0000;									//32位标识符ID
		can_filter.CAN_FilterIdLow = 0x0000;
		can_filter.CAN_FilterMaskIdHigh = 0x0000;							//32位的MASK屏蔽位
		can_filter.CAN_FilterMaskIdLow = 0x0000;
		can_filter.CAN_FilterFIFOAssignment = 0;							//过滤器0关联到FIFO0
		can_filter.CAN_FilterActivation=ENABLE;								//激活过滤器0
		CAN_FilterInit(&can_filter); 				//过滤器初始化
		
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//开启FMP0中断
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);			//开启TME邮箱中断 TME=0邮箱挂号，TME=1邮箱空置
}

/*****************控制电机函数*****************/
void Run(int16_t num1, int16_t num2, int16_t num3, int16_t num4, int16_t Std)     
{
    CanTxMsg tx_message;
	  tx_message.DLC=0x08;
	  tx_message.RTR=CAN_RTR_DATA;
	  tx_message.IDE=CAN_ID_STD;
	  tx_message.StdId=Std;
	  tx_message.Data[0]=(unsigned char)((num1>> 8)&0xff);
		tx_message.Data[1]=(unsigned char)(num1&0xff);
		tx_message.Data[2]=(unsigned char)((num2>> 8)&0xff);
    tx_message.Data[3]=(unsigned char)(num2&0xff);
    tx_message.Data[4]=(unsigned char)((num3>> 8)&0xff);
    tx_message.Data[5]=(unsigned char)(num3&0xff);
    tx_message.Data[6]=(unsigned char)((num4>> 8)&0xff);
    tx_message.Data[7]=(unsigned char)(num4&0xff);			
	  CAN_Transmit(CAN1,&tx_message);
}
	

/*****************CAN1的发送中断函数*****************/
void CAN1_TX_IRQHandler(void)
{
 if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 	
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);	
	}
//   delay_ms(1);
}

/*****************CAN1的接收中断函数*****************/
void CAN1_RX0_IRQHandler(void)
{
		CanRxMsg can_receive_message;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)	//判断是否接收结束
		{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		//清除FIFO0消息挂号中断标志位
        CAN_Receive(CAN1, CAN_FIFO0, &can_receive_message);	//获取接收数据
			
				CanReceiveMsgProcess(&can_receive_message);       		//将数据通过函数处理
    }
}


/*****************CAN1的数据处理函数*****************/
void CanReceiveMsgProcess(CanRxMsg *can_receive_message) 	//传入接收到数据的指针
{  

	switch(can_receive_message->StdId)	//判断标识符（根据电调说明书）
	{
		case 0x201:				
		
		m3508a.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
		m3508a.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
	    
        
			break;
		case 0x202:				
		
		 m3508b.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
		 m3508b.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]); 
	    
        
			break;
        case 0x203:				
		
		 m3508c.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
		m3508c.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
	    
        
			break;
        case 0x204:				
		
		 m3508d.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
		m3508d.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
	    
        
			break;
				case 0x205:				
		
		 m3508e.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
		 m3508e.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
	    
        
			break;
//        case 0x205:				
//		
//		 m3508e.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
//		 m3508e.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
//         m3508e.current= (can_receive_message->Data[4] << 8) | (can_receive_message->Data[5]);
//         m3508e.temp=  can_receive_message->Data[5];
//	    
//        
//			break;
//        case 0x206:				
//		
//		 m3508f.Angle= (can_receive_message->Data[0] << 8) | (can_receive_message->Data[1]);
//		 m3508f.Speed= (can_receive_message->Data[2] << 8) | (can_receive_message->Data[3]);
//	    
//        
//			break;
		
		}
        DataHande(&m3508a);
        DataHande(&m3508b);
		    DataHande(&m3508c);
		    DataHande(&m3508d);
		    DataHande(&m3508e);
}
//int LiMit(int input)
//{
//    if(input>50000)
//    {
//    input=input-65535;
//    }
//   return input;
//}
void DataHande(motorData *ms)
{    
    int res1, res2, delta;
	if(ms->Angle < ms->L_Angle){					//angle > last//可能的情况
		res1 = ms->Angle + 8192 - ms->L_Angle;	//正转，delta=+
		res2 = ms->Angle - ms->L_Angle;			//反转	delta=-
	}
	else{	//angle > last
		res1 = ms->Angle - 8192 - ms->L_Angle ;	//反转	delta -
		res2 = ms->Angle - ms->L_Angle;			//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	ms->G_Angle += delta;//输出偏移量-变化量
	ms->L_Angle = ms->Angle;//更新角度
    
}
