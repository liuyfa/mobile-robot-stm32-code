#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"


//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 
unsigned char can_tx_success_flag = 0;

//float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dt时间内的左右轮速度,用于里程计计算



u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
//#endif
    //使能相关时钟
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;//ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
//#if CAN1_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
//	
//#endif
	return 0;
}   


// 
//#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数		

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
long Delta_Real_Position_Value[4] = {0};

char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};
float Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dt时间内的左右轮速度,用于里程计计算
extern u8 main_sta; 
extern u16 test_ms;
void odometry(float right,float left);
u8 enter_calculate_flag = 1;
void CAN1_RX0_IRQHandler(void)
{
	  static long Pre_Real_Position_Value[4] = {0};
  	CanRxMsg rx_message;
//	int i=0;
//  CAN_Receive(CAN1, 0, &RxMessage);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		test_ms ++;
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {
			
            if(rx_message.StdId == 0x1B)
            {
                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
								Delta_Real_Position_Value[0]=Real_Position_Value[0]-Pre_Real_Position_Value[0];
							  
//				        printf("a=%ld\n",Delta_Real_Position_Value[0]);				
            }
            else if(rx_message.StdId == 0x2B)
            {
                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
							  Delta_Real_Position_Value[1]=Real_Position_Value[1]-Pre_Real_Position_Value[1];
							  
//   			        printf("b=%ld\n",Delta_Real_Position_Value[1]);
							  
            }
						
						Milemeter_L_Motor=-Delta_Real_Position_Value[0];	
						Milemeter_R_Motor=Delta_Real_Position_Value[1];
						
						if(Milemeter_L_Motor<0)
						Milemeter_L_Motor=-Milemeter_L_Motor;
						
						if(Milemeter_R_Motor<0)
						Milemeter_R_Motor=-Milemeter_R_Motor;
						
						
						
						

					  main_sta|=0x02;//执行计算里程计数据步骤
						if(enter_calculate_flag)
						{
							enter_calculate_flag = 0;
							odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
						}
						else
						{
							enter_calculate_flag = 1;
						}
						
//            else if(rx_message.StdId == 0x3B)
//            {
//                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x4B)
//            {
//                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
//            else if(rx_message.StdId == 0x1C)
//            {
//                Real_Ctl1_Value[0] = rx_message.Data[0];
//                Real_Ctl2_Value[0] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x2C)
//            {
//                Real_Ctl1_Value[1] = rx_message.Data[0];
//                Real_Ctl2_Value[1] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x3C)
//            {
//                Real_Ctl1_Value[2] = rx_message.Data[0];
//                Real_Ctl2_Value[2] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x4C)
//            {
//                Real_Ctl1_Value[3] = rx_message.Data[0];
//                Real_Ctl2_Value[3] = rx_message.Data[1];
//            }
					
					Pre_Real_Position_Value[0]=Real_Position_Value[0];
					Pre_Real_Position_Value[1]=Real_Position_Value[1];
        }
                
    }
	
}
//#endif


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x00;	 // 标准标识符为0
  TxMessage.ExtId=0x00;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

	
}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}















//can发送数据

void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
	u16 i=0;
	u8 mbox;
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
	mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}

/****************************************************************************************
                                      配置指令
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
	u16 i=0;
	u8 mbox;
    unsigned short can_id = 0x0A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;
    
    tx_message.Data[0] = Temp_Time1;
    tx_message.Data[1] = Temp_Time2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}

/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
	u16 i=0;
	u8 mbox;
    unsigned short can_id = 0x01;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
	
    mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束

//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}

void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
	u16 i=0;
	u8 mbox;
    unsigned short can_id = 0x02;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    can_tx_success_flag = 0;
    mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}

void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    u16 i=0;
	u8 mbox;
	unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    

    mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	
//    can_tx_success_flag = 0;
//   CAN_Transmit(CAN1,&tx_message);
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}

/****************************************************************************************
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
	u16 i=0;
	u8 mbox;
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    mbox=CAN_Transmit(CAN1,&tx_message);
    i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
}












