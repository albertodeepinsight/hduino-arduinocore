 /*
 * WDTPos.h - Library for watchdog on the SamD Zero series
 * Created by John V. - 2019 V 1.3.0
 * 
 * See ATSAMD2x datasheet, #18, page205 onwards
 * Defines Watchdog Class to Setup Watchdog timer in Normal mode with EarlyWarning interrupt.
 *  1. Sets Hardware WDT timer
 *     -> for stalled processor situations, generates a reset 
 *     -> 62ms upto 16s response
 *  2. Sets Software WDT using EarlyWarning timer (EWT) and software counter (Global Variable)
 *     -> for stalled software, generates a reset if software loop does not clear WDT on time
 *     -> Total soft-WDT intervals are 8,16,32 Seconds , 1,2,4,8,16 Minutes
 * 
 * V1.3.0 - added shutdown function for soft-watchdog : now possible to do an extra job before shutting down like saving your key data *thnks to Juraj
 * 
 * Released into the public domain.
 */
 
#include "Arduino.h"
#include "WDTPos.h"

voidFuncPtr WDT_Shutdown = NULL;
int WDTPosCounter;  // global counter for extended WDT via EW (early Warning) ISR

WDTPos::WDTPos()
{
}

void WDTPos::setup(unsigned int wdtpossetup) {
  // One-time initialization of watchdog timer.

    if (!wdtpossetup){                   // code 0x00 stops the watchdog
        NVIC_DisableIRQ(WDT_IRQn);        // disable IRQ
        NVIC_ClearPendingIRQ(WDT_IRQn);
        WDT->CTRLA.bit.ENABLE = 0;        // Stop watchdog now!
        while(WDT->SYNCBUSY.bit.ENABLE);
        }
    else {
    //  Split out the bits wdtpossetup =>  _ewtcounter=CNT[3] _w=EWen[1]  _x=DIV[4]  _y=PER[4]  _z=EW[4]
    
    
    _z = (char) (wdtpossetup&0x000f);       // lower quad bits = EWoffset
    _y = (char) ((wdtpossetup>>4)&0x000f);  // second quad bits = PER
    _x = (char) ((wdtpossetup>>8)&0x000f);  // third quad bits = DIV -> WARNING: Changing DIV CAN SCREW UP YOUR RTC !!!, best left at 0x4
    _w = (char) ((wdtpossetup>>12)&0x0001); // upper quad bits - bit[0] = EW enable
    _ewtcounter = 0x01<<( (wdtpossetup>>13)&0x0007); // make new software EWT counter value : upper quad :  3 bits -> 2^ bit[3:1]
    WDTPosCounter = _ewtcounter;            // SET Software EWT counter - used in ISR and Clear functions


  /*
  // Generic clock generator 2, divisor = 32 (2^(DIV+1))  = _x
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(_x);
  // Enable clock generator 2 using low-power 32.768kHz oscillator.
  // With /32 divisor above, this yields 1024Hz clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while(GCLK->STATUS.bit.SYNCBUSY);
  // WDT clock = clock gen 2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK2;

  */

  // Enable WDT early-warning interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0); // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
   
  WDT->INTENSET.bit.EW     = _w;       // Enable early warning interrupt - enable the multi cycle mechanism via WDT_Handler()
  WDT->EWCTRL.bit.EWOFFSET = _z;       // Early Warning Interrupt Time Offset 0x6 - 512 clockcycles = 0.5 seconds => trigger ISR
  WDT->CONFIG.bit.PER      = _y;       // Set period before hard WDT overflow <0x8 - 0xb>
  WDT->CTRLA.bit.WEN        = 0;          // Disable window mode
  while(WDT->SYNCBUSY.bit.ENABLE);       // Sync CTRL write
  WDT->CTRLA.bit.ENABLE     = 1;          // Start watchdog now!
  while(WDT->SYNCBUSY.bit.ENABLE);
}  
}

void WDTPos::clear() {
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;      // Clear WTD bit
  while(WDT->SYNCBUSY.bit.ENABLE);
  WDTPosCounter = _ewtcounter;         // Reset the early warning downcounter value
}

void WDT_Handler(void) {  // ISR for watchdog early warning, DO NOT RENAME!, need to clear
  WDTPosCounter--;                          // EWT down counter, makes multi cycle WDT possible
  if (WDTPosCounter<=0) {                   // Software EWT counter run out of time : Reset
         if (WDT_Shutdown != NULL) WDT_Shutdown(); // run extra Shutdown functions if defined
         WDT->CLEAR.reg = 0xFF;              // value different than WDT_CLEAR_CLEAR_KEY causes reset
         while(true);
         }
 else {
        WDT->INTFLAG.bit.EW = 1;              // Clear INT EW Flag
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; // Clear WTD bit
        while(WDT->SYNCBUSY.bit.ENABLE);
       }
}

void WDTPos::attachShutdown(voidFuncPtr callback)
{
  WDT_Shutdown = callback;
}

void WDTPos::detachShutdown()
{
  WDT_Shutdown = NULL;
}