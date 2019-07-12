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

/***********************************************  ���  *****************************************************************/

char odometry_data[23];   //���͸����ڵ���̼���������

float odometry_right=0,odometry_left=0;//���ڵõ����������ٶ�

/***********************************************  ����  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //����õ�����̼���ֵ

extern u8 USART_RX_BUF[USART_REC_LEN];     //���ڽ��ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART_RX_STA;                   //���ڽ���״̬���	


extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dtʱ���ڵ��������ٶ�,������̼Ƽ���

/***********************************************  ����  *****************************************************************/

u8 main_sta=0; //������������������if��ȥ�������flag��1��ӡ��̼ƣ���2���ü�����̼����ݺ�������3���ڽ��ճɹ�����4���ڽ���ʧ�ܣ�

union recieveData  //���յ�������
{
	float d;    //�������ٶ�
	unsigned char data[4];
}leftdata,rightdata;       //���յ�����������

union odometry  //��̼����ݹ�����
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //Ҫ��������̼����ݣ��ֱ�Ϊ��X��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�

/****************************************************************************************************************/
float Speed1=0; //���Bƽ��ת�� r/min
float Speed2=0; //���Aƽ��ת�� r/min

void Car_move(int right,int left)
{
	 
   CAN_RoboModule_DRV_Velocity_Mode(0,2,1000,right);
	 CAN_RoboModule_DRV_Velocity_Mode(0,1,1000,left);
	 delay_ms(5);
}

void data_pack(void)
{
	   u8 j=0;
	       //��̼����ݻ�ȡ
		x_data.odoemtry_float=position_x;//��λmm
		y_data.odoemtry_float=position_y;//��λmm
		theta_data.odoemtry_float=oriention;//��λrad
		vel_linear.odoemtry_float=velocity_linear;//��λmm/s
		vel_angular.odoemtry_float=velocity_angular;//��λrad/s
            
            //��������̼����ݴ浽Ҫ���͵�����
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
	u8 mode=0;//CAN����ģʽ;0,��ͨģʽ;1,����ģʽ
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	TIM3_Int_Init(500-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms  
//	TIM1_Configuration();
	delay_init(168);    //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	
	LCD_Init();           //��ʼ��LCD FSMC�ӿ�
	
	POINT_COLOR=BLUE;
	LCD_ShowString(30,50,200,16,16,"Debugging");	
	
	
//	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//������ʼ��  
   
	
/***********************************************  ���������ó�ʼ��  **********************************************************/
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps    
  CAN_RoboModule_DRV_Reset(0,0);                      //��0���������������и�λ     
  delay_ms(500);                                      //���͸�λָ������ʱ����Ҫ�У��ȴ��������ٴγ�ʼ�����    
  CAN_RoboModule_DRV_Config(0,1,5,0);               //1������������Ϊ100ms����һ�ε����ٶ�λ������
  delay_us(200);                                      //�˴���ʱΪ�˲��ô�������ʱ��4����һ��
	CAN_RoboModule_DRV_Config(0,2,5,0);               //2������������Ϊ100ms����һ�ε����ٶ�λ������
  delay_us(200);     
  CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //0������������� �����뿪��ģʽ
  delay_ms(500);                            	//����ģʽѡ��ָ���Ҫ�ȴ�����������ģʽ������������ʱҲ������ȥ����
	

	
 									  
while(1)
	{
		
		if(main_sta&0x01)//ִ�з�����̼����ݲ���
		{
      data_pack(); 
			//��������Ҫ����
			for(i=0;i<23;i++)
			{
				USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
				USART_SendData(USART1,odometry_data[i]);//����һ���ֽڵ�����	
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//�ȴ����ͽ���			
			}
            
			main_sta&=0xFE;//ִ�м�����̼����ݲ���
		}

		if(main_sta&0x02)//ִ�м�����̼����ݲ���
		{
			
//			 odometry(Milemeter_R_Motor,Milemeter_L_Motor);//������̼�
			   main_sta&=0xFD;//ִ�з�����̼����ݲ���
		} 
		
		
		if(main_sta&0x08)        //������ָ��û����ȷ����ʱ
		{
			      USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����
            for(m=0;m<3;m++)
            {
                USART_SendData(USART1,0x00);	
                while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
            }		
            USART_SendData(USART1,'\n');	
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
            main_sta&=0xF7;
		}
		
		if(USART_RX_STA&0x8000)  // ����1���պ���
		{			
            //�����������ٶ�
            for(t=0;t<4;t++)
            {
                rightdata.data[t]=USART_RX_BUF[t];
                leftdata.data[t]=USART_RX_BUF[t+4];
            }
            
            //�����������ٶ�
            odometry_right=rightdata.d;//��λmm/s
            odometry_left=leftdata.d;//��λmm/s
						
						Speed1=odometry_right;
						Speed2=-odometry_left;
			      USART_RX_STA=0;//������ձ�־λ
			
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
		
		
		
		
		Car_move(Speed1,Speed2);	 //�����յ����������ٶȸ���С��	
		
		
		
		
		
		
		
		
		
		
//		
//		key=KEY_Scan(0);

//     if(key==KEY0_PRES)//KEY0����,����һ������
//		{
//		 CAN_RoboModule_DRV_Velocity_Mode(0,1,1000,0);
//		 delay_ms(10); 
//		 CAN_RoboModule_DRV_Velocity_Mode(0,2,1000,0);
//		 delay_ms(10);
//		 
//		}
//		else if(key==WKUP_PRES)//WK_UP���£��ı�CAN�Ĺ���ģʽ
//		{	   
//		 CAN_RoboModule_DRV_Velocity_Mode(0,1,0,0);
//		 delay_ms(10);
//		 CAN_RoboModule_DRV_Velocity_Mode(0,2,0,0);
//		 delay_ms(10);
//		}				
//		
		
		
	 }
}
