#include "can1.h"
#include "math.h"
#define ABS(x)		((x>0)? (x): (-x)) 


struct motorData m3508a,m3508b,m3508c,m3508d,m3508e,m3508f;

/*******************��ʼ��CAN1����*********************/
void CAN1_Configuration(void)   
{
		CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//ʹ��CAN1ʱ��
	
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); //PD0��CAN1_RX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); //PD1��CAN1_TX
		
//		/**********GPIO��ʼ��***********/
//    gpio.GPIO_Pin = GPIO_Pin_0;									
//		gpio.GPIO_Speed = GPIO_Speed_100MHz;				
//    gpio.GPIO_Mode = GPIO_Mode_AF;			//����
//    GPIO_Init(GPIOD, &gpio);
	
		gpio.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;;
    gpio.GPIO_Mode = GPIO_Mode_AF;			//����
    GPIO_Init(GPIOD, &gpio);
	
		/**********NVIC�жϳ�ʼ��***********/
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;				//�����ж�ͨ��
    nvic.NVIC_IRQChannelPreemptionPriority = 1;	//�����жϵ���ռ���ȼ�Ϊ1
    nvic.NVIC_IRQChannelSubPriority = 0;				//�����жϵ���Ӧ���ȼ�Ϊ1
    nvic.NVIC_IRQChannelCmd = ENABLE;						//IRQͨ��ʹ��    IRQ�����ж�����
    NVIC_Init(&nvic);				//��������Ĳ�����ʼ��NVIC�Ĵ���
       
//		nvic.NVIC_IRQChannel = CAN1_TX_IRQn;				//�����ж�ͨ��
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;	//�����жϵ���ռ���ȼ�Ϊ0
//    nvic.NVIC_IRQChannelSubPriority = 0;				//�����жϵ���Ӧ���ȼ�Ϊ1
//    nvic.NVIC_IRQChannelCmd = ENABLE;						//IRQͨ��ʹ��		IRQ�����ж�����
//    NVIC_Init(&nvic);				//��������Ĳ�����ʼ��NVIC�Ĵ���
    
    CAN_DeInit(CAN1);						
    CAN_StructInit(&can);
	
		/******CAN1��Ԫ����******/
    can.CAN_TTCM = DISABLE;						//��ʱ�䴥��ͨ��ģʽ
    can.CAN_ABOM = DISABLE;						//����Զ����߹���
    can.CAN_AWUM = DISABLE;						//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    can.CAN_NART = DISABLE;						//��ֹ�����Զ�����
    can.CAN_RFLM = DISABLE;						//���Ĳ�����,�µĸ��Ǿɵ�
    can.CAN_TXFP = ENABLE;						//���ȼ��ɱ��ı�ʶ������
    can.CAN_Mode = CAN_Mode_Normal;		//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ��
		
		/******CAN1����������******/
    can.CAN_SJW  = CAN_SJW_1tq;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ
    can.CAN_BS1 = CAN_BS1_7tq;				//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
    can.CAN_BS2 = CAN_BS2_7tq;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
    can.CAN_Prescaler = 3;  					//��Ƶϵ��(Fdiv)Ϊbrp+1
    CAN_Init(CAN1, &can);							//��ʼ��CAN1
		
		can_filter.CAN_FilterNumber = 10;											//������0
		can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale = CAN_FilterScale_32bit;		//32λ��ʶ������λģʽ
		can_filter.CAN_FilterIdHigh = 0x0000;									//32λ��ʶ��ID
		can_filter.CAN_FilterIdLow = 0x0000;
		can_filter.CAN_FilterMaskIdHigh = 0x0000;							//32λ��MASK����λ
		can_filter.CAN_FilterMaskIdLow = 0x0000;
		can_filter.CAN_FilterFIFOAssignment = 0;							//������0������FIFO0
		can_filter.CAN_FilterActivation=ENABLE;								//���������0
		CAN_FilterInit(&can_filter); 				//��������ʼ��
		
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//����FMP0�ж�
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);			//����TME�����ж� TME=0����Һţ�TME=1�������
}

/*****************���Ƶ������*****************/
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
	

/*****************CAN1�ķ����жϺ���*****************/
void CAN1_TX_IRQHandler(void)
{
 if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 	
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);	
	}
//   delay_ms(1);
}

/*****************CAN1�Ľ����жϺ���*****************/
void CAN1_RX0_IRQHandler(void)
{
		CanRxMsg can_receive_message;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)	//�ж��Ƿ���ս���
		{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		//���FIFO0��Ϣ�Һ��жϱ�־λ
        CAN_Receive(CAN1, CAN_FIFO0, &can_receive_message);	//��ȡ��������
			
				CanReceiveMsgProcess(&can_receive_message);       		//������ͨ����������
    }
}


/*****************CAN1�����ݴ�����*****************/
void CanReceiveMsgProcess(CanRxMsg *can_receive_message) 	//������յ����ݵ�ָ��
{  

	switch(can_receive_message->StdId)	//�жϱ�ʶ�������ݵ��˵���飩
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
	if(ms->Angle < ms->L_Angle){					//angle > last//���ܵ����
		res1 = ms->Angle + 8192 - ms->L_Angle;	//��ת��delta=+
		res2 = ms->Angle - ms->L_Angle;			//��ת	delta=-
	}
	else{	//angle > last
		res1 = ms->Angle - 8192 - ms->L_Angle ;	//��ת	delta -
		res2 = ms->Angle - ms->L_Angle;			//��ת	delta +
	}
	//��������ת���϶���ת�ĽǶ�С���Ǹ������
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	ms->G_Angle += delta;//���ƫ����-�仯��
	ms->L_Angle = ms->Angle;//���½Ƕ�
    
}
