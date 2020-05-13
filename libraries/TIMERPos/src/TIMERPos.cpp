/*
  ZeroTimer.cpp - Arduino Zero M0 Timer Interrput library
  Copyright (c) 2017 Tamasa (@EHbtj).  All right reserved.
*/

#include "Arduino.h"
#include "TIMERPos.h"

uint16_t _prescaler;

void (*func1)();

TIMERPos::TIMERPos()
{
}

void TIMERPos::configureTimer(unsigned long period, void (*f)())
{

  // Enable MLK for TC4
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC4;

  // Enable GCLK for TC4 (timer counter input clock)
  GCLK->PCHCTRL[TC4_GCLK_ID].reg = (GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN) ;
  while (GCLK->SYNCBUSY.bit.GENCTRL);

  disableTimer();
  resetTimer();

  // Set Timer counter Mode to 16 bits and Prescaler 1024
  TC4->COUNT16.CTRLA.reg = (TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1024);
  uint16_t prescaler = 1024;
  while(timerIsSyncing());

  // Set TC4 mode as match frequency
  TC4->COUNT16.WAVE.reg |= TC_WAVE_WAVEGEN_MFRQ;
  while(timerIsSyncing());

  //set TC4 timer counter based off of the system clock and the user defined sample rate or waveform
  TC4->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / (prescaler / ((float)period / 1000)));
  while(timerIsSyncing());

  // Configure interrupt request
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);

  // Enable the TC4 interrupt request
  TC4->COUNT16.INTENSET.reg = 0;
  TC4->COUNT16.INTENSET.bit.MC0 = 1;
  while (timerIsSyncing()); //wait until TC4 is done syncing

  func1 = f;

  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);

}

void TC4_Handler()
{
  // If this interrupt is due to the compare register matching the timer count
  if (TC4->COUNT16.INTFLAG.bit.MC0 == 1) {
    TC4->COUNT16.INTFLAG.bit.MC0 = 1;
		(*func1)();
  }
}

void TIMERPos:: startTimer()
{
  TC4->COUNT16.COUNT.reg = 0;
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (timerIsSyncing()); //wait until snyc'd

}

void TIMERPos:: disableTimer()
{
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (timerIsSyncing());

}

bool TIMERPos::timerIsSyncing()
{
    return TC4->COUNT16.STATUS.reg & TC_SYNCBUSY_STATUS;
}

void TIMERPos::resetTimer()
{
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (timerIsSyncing());
  while (TC4->COUNT16.CTRLA.bit.SWRST);
}
