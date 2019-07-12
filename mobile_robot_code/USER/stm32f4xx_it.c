/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
 
extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];//�������ٶȻ�������
extern u8 main_sta;//����������ִ�б�־λ

//extern u8 bSpeed_Buffer_Index;
u8 bSpeed_Buffer_Index = 0;//���������ֱ��������������

//extern float Milemeter_L_Motor,Milemeter_R_Motor;      //�ۼƵ��һ�����е���� cm		




extern float pulse;//���A PID���ں��PWMֵ����
extern float pulse1;//���B PID���ں��PWMֵ����

/***************************************************************************************************/



//void TIM5_IRQHandler(void)//С���ٶȼ��㶨ʱ���жϺ���
//{
//	if ( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET ) 
//	{						      
//        if (hSpeedMeas_Timebase_500us !=0)//����������ɼ�ʱ����δ��
//        {
//            hSpeedMeas_Timebase_500us--;//��ʼ����	
//        }
//        else    //����������ɼ�ʱ��������
//        {
//            s32 wtemp2,wtemp1;
//            
//            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//�ָ�����������ɼ�ʱ����
//            
//            /************************ 1 ***************************/
//            
//            wtemp2 = ENC_Calc_Rot_Speed2(); //A ��ȡ�ı�����
//            wtemp1 = ENC_Calc_Rot_Speed1(); //B ��ȡ�ı�����
//            
////            //���Ϊָֹͣ����������ٶ�Ϊ�㣬������ٶȴ洢����ֹǰ���ٶȲ�̫�����С����ת
////            if((wtemp2 == 0) && (wtemp1 == 0))
////            {
////                pulse=pulse1=0;
////            }
//             
//            /************************ 2 ***************************/
//            
//            //���������������������������̼Ƽ���
//            Milemeter_L_Motor= (float)wtemp1; //����������
//            Milemeter_R_Motor= (float)wtemp2;
//            
//            main_sta|=0x02;//ִ�м�����̼����ݲ���

//            /************************ 3 ***************************/
//            
//            //��ʼ���������ֱ�����������
//            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
//            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
//            bSpeed_Buffer_Index++;//������λ
//            
//            //���������ֱ���������������ж�
//            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
//            {
//                bSpeed_Buffer_Index=0;//���������ֱ������������������
//            }
//            
//            /************************ 4 ***************************/
//            
//            ENC_Calc_Average_Speed();//�������ε����ƽ��������
//            Gain2(); //���Aת��PID���ڿ��� ��
//            Gain1(); //���Bת��PID���ڿ��� ��
//        }
//        
//		TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);//����жϱ�־λ    		 
//	}		 
//}



/***************************************************************************************************/


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
	
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
