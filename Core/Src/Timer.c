/*
 * Tmr1.c
 *
 *  Created on: Aug 3, 2021
 *      Author: ken chow
 */

#include <Timer.h>
#include "Rcc.h"

uint32_t getTimerFreq(TmrReg *tim){
	HAL_RCC_GetHCLKFreq();
}

void tmrConfigure(TmrReg *tim, Tmrconfig_1 cfg){
	tim->CR1 = (cfg & TIMER_CR1_MASK);
	tim->CR2 = ((cfg >> 16) & TIMER_CR2_MASK);
	tim->SMCR = ((cfg >> 32) & TIMER_SMCR_MASK);
	tim->DIER = ((cfg >> 48) & TIMER_DIER_MASK);
}
/*
void tmrConfigureCaptureCompare(TmrReg *tim, int channel, Capture_Compare_Config_Timer cfg){
	if(channel < 2)
		tim->CCMR1 = cfg & 0xFFFF;
	else
		tim->CCMR2 = (cfg >> 16) & 0xFFFF;
	tim->CCER = (cfg >> 32) & 0x3FFF;
}
*/
void CaptureCompare_config_P1(TmrReg *tim){
	tim->CCMR1 = 0x02;
	tim->CCMR2 = 0x68;
	tim->CCER = 0x101;
}

void CaptureCompare_config_P2_state1(TmrReg *tim){
	tim->CCMR2 = 0x50 ;
	tim->CCER = 0x100;
}

void CaptureCompare_config_P2_state2(TmrReg *tim){
	tim->CCMR2 = 0x40 ;
	tim->CCER = 0x100;
}

void CaptureCompare_config_P3(TmrReg *tim){
	tim->CCMR2 = 0x30;
	tim->CCER = 0x100;
}
void tmrConfigureEventGeneration(TmrReg *tim,TimerEventGenConfig cfg){
	tim->EGR = cfg & 0xFFFF;
}

void setTimerPrescaler(TmrReg *tim, uint16_t prescale){
	tim->PSC = prescale;
}

void setTimerARR(TmrReg *tim, uint16_t num){
	tim->ARR = num;
}

void set_PWM_Freq(TmrReg *tim, int frequency){
	uint32_t apb1freq = getTimerFreq(tim);
	setTimerARR(tim, ARR_Num);
	setTimerPrescaler(tim, (apb1freq/(ARR_Num * frequency)));
}

void setTimerCaptureCompareReg(TmrReg *tim, int channel, uint16_t number){
	if(channel == 0x01)
		tim->CCR1 = number;
	else if(channel == 0x02)
		tim->CCR2 = number;
	else if(channel == 0x03)
		tim->CCR3 = number;
	else
		tim->CCR4 = number;
}

void set_PWM_Duty_Cycle(TmrReg *tim, int channel, int duty){
	setTimerCaptureCompareReg(tim, channel, (duty/100.0)*ARR_Num);
}
