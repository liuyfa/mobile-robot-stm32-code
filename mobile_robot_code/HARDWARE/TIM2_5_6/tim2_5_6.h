#ifndef __TIM2_5_6_H
#define __TIM2_5_6_H

#include "stm32f4xx.h"
#include "sys.h"

void TIM2_PWM_Init(void);//PWM�����ʼ��
void TIM5_Configuration(void);//�ٶȼ��㶨ʱ����ʼ��
void TIM1_Configuration(void);//��̼Ʒ�����ʱ����ʼ��
void TIM1_UP_IRQHandler(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
#endif 
