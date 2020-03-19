#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM8_Int_Init(u16 arr,u16 psc);
void TIM8_PWM_Init(u16 arr,u16 psc);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void time7_init(u16 per,u16 pre);
void NVIC_INIT(void);
void TIM1_Init(void);
static void Advance_TIM_Config(void);
static void TIM_GPIO_Config(void);

#endif

