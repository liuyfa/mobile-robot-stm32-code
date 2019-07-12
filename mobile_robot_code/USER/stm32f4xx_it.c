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
 
extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];//左右轮速度缓存数组
extern u8 main_sta;//主函数步骤执行标志位

//extern u8 bSpeed_Buffer_Index;
u8 bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变量

//extern float Milemeter_L_Motor,Milemeter_R_Motor;      //累计电机一次运行的里程 cm		




extern float pulse;//电机A PID调节后的PWM值缓存
extern float pulse1;//电机B PID调节后的PWM值缓存

/***************************************************************************************************/



//void TIM5_IRQHandler(void)//小车速度计算定时器中断函数
//{
//	if ( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET ) 
//	{						      
//        if (hSpeedMeas_Timebase_500us !=0)//电机编码数采集时间间隔未到
//        {
//            hSpeedMeas_Timebase_500us--;//开始倒数	
//        }
//        else    //电机编码数采集时间间隔到了
//        {
//            s32 wtemp2,wtemp1;
//            
//            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//恢复电机编码数采集时间间隔
//            
//            /************************ 1 ***************************/
//            
//            wtemp2 = ENC_Calc_Rot_Speed2(); //A 获取的编码数
//            wtemp1 = ENC_Calc_Rot_Speed1(); //B 获取的编码数
//            
////            //如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
////            if((wtemp2 == 0) && (wtemp1 == 0))
////            {
////                pulse=pulse1=0;
////            }
//             
//            /************************ 2 ***************************/
//            
//            //储存编码数（脉冲数），用于里程计计算
//            Milemeter_L_Motor= (float)wtemp1; //储存脉冲数
//            Milemeter_R_Motor= (float)wtemp2;
//            
//            main_sta|=0x02;//执行计算里程计数据步骤

//            /************************ 3 ***************************/
//            
//            //开始缓存左右轮编码数到数组
//            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
//            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
//            bSpeed_Buffer_Index++;//数组移位
//            
//            //缓存左右轮编码数到数组结束判断
//            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
//            {
//                bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
//            }
//            
//            /************************ 4 ***************************/
//            
//            ENC_Calc_Average_Speed();//计算三次电机的平均编码数
//            Gain2(); //电机A转速PID调节控制 右
//            Gain1(); //电机B转速PID调节控制 左
//        }
//        
//		TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);//清除中断标志位    		 
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
