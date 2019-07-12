#ifndef __TIM2_5_6_H
#define __TIM2_5_6_H

#include "stm32f4xx.h"
#include "sys.h"

void TIM2_PWM_Init(void);//PWM输出初始化
void TIM5_Configuration(void);//速度计算定时器初始化
void TIM1_Configuration(void);//里程计发布定时器初始化
void TIM1_UP_IRQHandler(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
#endif 
