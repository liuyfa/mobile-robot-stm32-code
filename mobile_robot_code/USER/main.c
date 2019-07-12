#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "can.h"
#include "odometry.h"

#include "stdbool.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "tim2_5_6.h"

/***********************************************  输出  *****************************************************************/

char odometry_data[23];   //发送给串口的里程计数据数组

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度

/***********************************************  输入  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值

extern u8 USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;                   //串口接收状态标记	


extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮速度,用于里程计计算

/***********************************************  变量  *****************************************************************/

u8 main_sta=0; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）

union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

union odometry  //里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度

/****************************************************************************************************************/
float Speed1=0; //电机B平均转速 r/min
float Speed2=0; //电机A平均转速 r/min

void Car_move(int right,int left)
{
	 
   CAN_RoboModule_DRV_Velocity_Mode(0,2,1000,right);
	 CAN_RoboModule_DRV_Velocity_Mode(0,1,1000,left);
	 delay_ms(5);
}

void data_pack(void)
{
	   u8 j=0;
	       //里程计数据获取
		x_data.odoemtry_float=position_x;//单位mm
		y_data.odoemtry_float=position_y;//单位mm
		theta_data.odoemtry_float=oriention;//单位rad
		vel_linear.odoemtry_float=velocity_linear;//单位mm/s
		vel_angular.odoemtry_float=velocity_angular;//单位rad/s
            
            //将所有里程计数据存到要发送的数组
		for(j=0;j<4;j++)
		{
				odometry_data[j+2]=x_data.odometry_char[j];
				odometry_data[j+6]=y_data.odometry_char[j];
				odometry_data[j+10]=theta_data.odometry_char[j];
				odometry_data[j+14]=vel_linear.odometry_char[j];
				odometry_data[j+18]=vel_angular.odometry_char[j];
		}
  odometry_data[0] = 0xaa;
	odometry_data[1] = 0xaa;
	
	odometry_data[2] = x_data.odometry_char[0];
	odometry_data[3] = x_data.odometry_char[1];
	odometry_data[4] = x_data.odometry_char[2];
	odometry_data[5] = x_data.odometry_char[3];
	
	odometry_data[6] = y_data.odometry_char[0];
	odometry_data[7] = y_data.odometry_char[1];
	odometry_data[8] = y_data.odometry_char[2];
	odometry_data[9] = y_data.odometry_char[3];
	
	odometry_data[10] = theta_data.odometry_char[0];
	odometry_data[11] = theta_data.odometry_char[1];
	odometry_data[12] = theta_data.odometry_char[2];
	odometry_data[13] = theta_data.odometry_char[3];
	
	odometry_data[14] = vel_linear.odometry_char[0];
	odometry_data[15] = vel_linear.odometry_char[1];
	odometry_data[16] = vel_linear.odometry_char[2];
	odometry_data[17] = vel_linear.odometry_char[3];
	
	odometry_data[18] = vel_angular.odometry_char[0];
	odometry_data[19] = vel_angular.odometry_char[1];
	odometry_data[20] = vel_angular.odometry_char[2];
	odometry_data[21] = vel_angular.odometry_char[3];
	

	
	odometry_data[22] = odometry_data[2]^odometry_data[3]^odometry_data[4]^odometry_data[5]^odometry_data[6]^
											odometry_data[7]^odometry_data[8]^odometry_data[9]^odometry_data[10]^odometry_data[11]^
											odometry_data[12]^odometry_data[13]^odometry_data[14]^odometry_data[15]^odometry_data[16]^
											odometry_data[17]^odometry_data[18]^odometry_data[19]^odometry_data[20]^odometry_data[21];

}


int main(void)
{ 
	u8 key;
	u8 i=0,t=0;
	u8 j=0,m=0;
	char* speed_right= 0;
	char* speed_left= 0;
	char Posx_str[20];
	char Posy_str[20];
	char MR_str[20];
	char ML_str[20];
  char odr[20];
	char odl[20];
	
	
	
	u8 cnt=0;
	u8 res;
	u8 mode=0;//CAN工作模式;0,普通模式;1,环回模式
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	TIM3_Int_Init(500-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms  
//	TIM1_Configuration();
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	
	LCD_Init();           //初始化LCD FSMC接口
	
	POINT_COLOR=BLUE;
	LCD_ShowString(30,50,200,16,16,"Debugging");	
	
	
//	LED_Init();					//初始化LED 
	KEY_Init(); 				//按键初始化  
   
	
/***********************************************  驱动器配置初始化  **********************************************************/
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps    
  CAN_RoboModule_DRV_Reset(0,0);                      //对0组所有驱动器进行复位     
  delay_ms(500);                                      //发送复位指令后的延时必须要有，等待驱动器再次初始化完成    
  CAN_RoboModule_DRV_Config(0,1,5,0);               //1号驱动器配置为100ms传回一次电流速度位置数据
  delay_us(200);                                      //此处延时为了不让传回数据时候4个不一起传
	CAN_RoboModule_DRV_Config(0,2,5,0);               //2号驱动器配置为100ms传回一次电流速度位置数据
  delay_us(200);     
  CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //0组的所有驱动器 都进入开环模式
  delay_ms(500);                            	//发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
	

	
 									  
while(1)
	{
		
		if(main_sta&0x01)//执行发送里程计数据步骤
		{
      data_pack(); 
			//发送数据要串口
			for(i=0;i<23;i++)
			{
				USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
				USART_SendData(USART1,odometry_data[i]);//发送一个字节到串口	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//等待发送结束			
			}
            
			main_sta&=0xFE;//执行计算里程计数据步骤
		}

		if(main_sta&0x02)//执行计算里程计数据步骤
		{
			
//			 odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计
			   main_sta&=0xFD;//执行发送里程计数据步骤
		} 
		
		
		if(main_sta&0x08)        //当发送指令没有正确接收时
		{
			      USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
            for(m=0;m<3;m++)
            {
                USART_SendData(USART1,0x00);	
                while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
            }		
            USART_SendData(USART1,'\n');	
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
            main_sta&=0xF7;
		}
		
		if(USART_RX_STA&0x8000)  // 串口1接收函数
		{			
            //接收左右轮速度
            for(t=0;t<4;t++)
            {
                rightdata.data[t]=USART_RX_BUF[t];
                leftdata.data[t]=USART_RX_BUF[t+4];
            }
            
            //储存左右轮速度
            odometry_right=rightdata.d;//单位mm/s
            odometry_left=leftdata.d;//单位mm/s
						
						Speed1=odometry_right;
						Speed2=-odometry_left;
			      USART_RX_STA=0;//清楚接收标志位
			
		}
		
		   sprintf(Posx_str,"%6.4f",position_x);
		   sprintf(Posy_str,"%6.4f",position_y);
		   sprintf(MR_str,"%6.4f",Milemeter_R_Motor);
		   sprintf(ML_str,"%6.4f",Milemeter_L_Motor);
		   sprintf(odr,"%6.4f",odometry_right);
		   sprintf(odl,"%6.4f",odometry_left);
		
		
		
		   
		  
		  LCD_ShowString(30,70,200,24,24,Posx_str);
  		LCD_ShowString(30,100,200,24,24,Posy_str);
   		LCD_ShowString(30,130,200,24,24,MR_str);
  		LCD_ShowString(30,160,200,24,24,ML_str);
		  LCD_ShowString(30,190,200,24,24,odr);
  		LCD_ShowString(30,220,200,24,24,odl);
		
		
		
		
		Car_move(Speed1,Speed2);	 //将接收到的左右轮速度赋给小车	
		
		
		
		
		
		
		
		
		
		
//		
//		key=KEY_Scan(0);

//     if(key==KEY0_PRES)//KEY0按下,发送一次数据
//		{
//		 CAN_RoboModule_DRV_Velocity_Mode(0,1,1000,0);
//		 delay_ms(10); 
//		 CAN_RoboModule_DRV_Velocity_Mode(0,2,1000,0);
//		 delay_ms(10);
//		 
//		}
//		else if(key==WKUP_PRES)//WK_UP按下，改变CAN的工作模式
//		{	   
//		 CAN_RoboModule_DRV_Velocity_Mode(0,1,0,0);
//		 delay_ms(10);
//		 CAN_RoboModule_DRV_Velocity_Mode(0,2,0,0);
//		 delay_ms(10);
//		}				
//		
		
		
	 }
}
