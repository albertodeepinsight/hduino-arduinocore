/*
  RTC library for Arduino Zero.
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <time.h>
#include "hpl_osc32kctrl_config.h"

#include "RTCPos.h"

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900

// Default date & time after reset
#define DEFAULT_YEAR    2000    // 2000..2063
#define DEFAULT_MONTH   1       // 1..12
#define DEFAULT_DAY     1       // 1..31
#define DEFAULT_HOUR    0       // 1..23
#define DEFAULT_MINUTE  0       // 0..59
#define DEFAULT_SECOND  0       // 0..59

voidFuncPtr RTC_callBack = NULL;
bool _repeat_second = false;

RTCPos::RTCPos()
{
  _configured = false;
  _repeat_second = false;
}

void RTCPos::begin(bool resetTime)
{
  uint16_t tmp = 0;

  config32kOSC(); // External 32kHz Oscilator configuration

  configMCLK();  // MCLK configuration

  RTCMode2_disable(); // RTC Mode2 disable

  RTCMode2_configCTRLA(); // RTC->MODE2.CTRLA configuration

  RTCMode2_configALARM0(); // Alarm 0 configuration

  RTCMode2_enable();

  _configured = true;


}

void RTC_Handler(void)
{
  RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; // must clear flag at end

  if(_repeat_second){
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = (RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND + 1) % 60;
  }

  if (RTC_callBack != NULL) {
    RTC_callBack();
  }
}

void RTCPos::enableAlarm(Alarm_Match match)
{
  if (_configured) {
    if(match == MATCH_SS_REPEAT){
        _repeat_second = true;
        match = MATCH_SS;
    }
    RTC->MODE2.Mode2Alarm[0].MASK.reg = match;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::disableAlarm()
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].MASK.reg = 0x00;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::attachInterrupt(voidFuncPtr callback)
{
  RTC_callBack = callback;
}

void RTCPos::detachInterrupt()
{
  RTC_callBack = NULL;
}

void RTCPos::standbyMode()
{
  // Entering standby mode when connected
  // via the native USB port causes issues.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
}

/*
 * Get Functions
 */

uint8_t RTCPos::getSeconds()
{
  return RTC->MODE2.CLOCK.bit.SECOND;
}

uint8_t RTCPos::getMinutes()
{
  return RTC->MODE2.CLOCK.bit.MINUTE;
}

uint8_t RTCPos::getHours()
{
  return RTC->MODE2.CLOCK.bit.HOUR;
}

uint8_t RTCPos::getDay()
{
  return RTC->MODE2.CLOCK.bit.DAY;
}

uint8_t RTCPos::getMonth()
{
  return RTC->MODE2.CLOCK.bit.MONTH;
}

uint8_t RTCPos::getYear()
{
  return RTC->MODE2.CLOCK.bit.YEAR;
}

uint8_t RTCPos::getAlarmSeconds()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
}

uint8_t RTCPos::getAlarmMinutes()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
}

uint8_t RTCPos::getAlarmHours()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
}

uint8_t RTCPos::getAlarmDay()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
}

uint8_t RTCPos::getAlarmMonth()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
}

uint8_t RTCPos::getAlarmYear()
{
  return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
}

/*
 * Set Functions
 */

void RTCPos::setSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.SECOND = seconds;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MINUTE = minutes;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.HOUR = hours;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {

    RTC->MODE2.CLOCK.reg = (  seconds << RTC_MODE2_CLOCK_SECOND_Pos
                            | minutes << RTC_MODE2_CLOCK_MINUTE_Pos
                            | hours << RTC_MODE2_CLOCK_HOUR_Pos
                            | RTC->MODE2.CLOCK.bit.DAY << RTC_MODE2_CLOCK_DAY_Pos
                            | RTC->MODE2.CLOCK.bit.MONTH << RTC_MODE2_CLOCK_MONTH_Pos
                            | RTC->MODE2.CLOCK.bit.YEAR << RTC_MODE2_CLOCK_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);

  }
}

void RTCPos::setDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.DAY = day;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.MONTH = month;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.CLOCK.bit.YEAR = year;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setDate(uint8_t year, uint8_t month, uint8_t day)
{
  if (_configured) {
    RTC->MODE2.CLOCK.reg = (  RTC->MODE2.CLOCK.bit.SECOND << RTC_MODE2_CLOCK_SECOND_Pos
                            | RTC->MODE2.CLOCK.bit.MINUTE << RTC_MODE2_CLOCK_MINUTE_Pos
                            | RTC->MODE2.CLOCK.bit.HOUR << RTC_MODE2_CLOCK_HOUR_Pos
                            | day << RTC_MODE2_CLOCK_DAY_Pos
                            | month << RTC_MODE2_CLOCK_MONTH_Pos
                            | year << RTC_MODE2_CLOCK_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setDateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.CLOCK.reg = (  seconds << RTC_MODE2_CLOCK_SECOND_Pos
                            | minutes << RTC_MODE2_CLOCK_MINUTE_Pos
                            | hours << RTC_MODE2_CLOCK_HOUR_Pos
                            | day << RTC_MODE2_CLOCK_DAY_Pos
                            | month << RTC_MODE2_CLOCK_MONTH_Pos
                            | year << RTC_MODE2_CLOCK_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setAlarmSeconds(uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmMinutes(uint8_t minutes)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmHours(uint8_t hours)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.reg = ( seconds << RTC_MODE2_ALARM_SECOND_Pos
                                         | minutes << RTC_MODE2_ALARM_MINUTE_Pos
                                         | hours << RTC_MODE2_ALARM_HOUR_Pos
                                         | RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY << RTC_MODE2_ALARM_DAY_Pos
                                         | RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH << RTC_MODE2_ALARM_MONTH_Pos
                                         | RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR << RTC_MODE2_ALARM_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmDay(uint8_t day)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmMonth(uint8_t month)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmYear(uint8_t year)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmDate(uint8_t year, uint8_t month, uint8_t day)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.reg = ( RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND << RTC_MODE2_ALARM_SECOND_Pos
                                         | RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE << RTC_MODE2_ALARM_MINUTE_Pos
                                         | RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR << RTC_MODE2_ALARM_HOUR_Pos
                                         | day << RTC_MODE2_ALARM_DAY_Pos
                                         | month << RTC_MODE2_ALARM_MONTH_Pos
                                         | year << RTC_MODE2_ALARM_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

void RTCPos::setAlarmDateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  if (_configured) {
    RTC->MODE2.Mode2Alarm[0].ALARM.reg = ( seconds << RTC_MODE2_ALARM_SECOND_Pos
                                         | minutes << RTC_MODE2_ALARM_MINUTE_Pos
                                         | hours << RTC_MODE2_ALARM_HOUR_Pos
                                         | day << RTC_MODE2_ALARM_DAY_Pos
                                         | month << RTC_MODE2_ALARM_MONTH_Pos
                                         | year << RTC_MODE2_ALARM_YEAR_Pos);
    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);
  }
}

uint32_t RTCPos::getEpoch()
{
  RTC_MODE2_CLOCK_Type clockTime;
  clockTime.reg = RTC->MODE2.CLOCK.reg;

  return RTC->MODE2.CLOCK.bit.SECOND;

  struct tm tm;

  tm.tm_isdst = -1;
  tm.tm_yday = 0;
  tm.tm_wday = 0;
  tm.tm_year = clockTime.bit.YEAR + EPOCH_TIME_YEAR_OFF;
  tm.tm_mon = clockTime.bit.MONTH - 1;
  tm.tm_mday = clockTime.bit.DAY;
  tm.tm_hour = clockTime.bit.HOUR;
  tm.tm_min = clockTime.bit.MINUTE;
  tm.tm_sec = clockTime.bit.SECOND;

  return mktime(&tm);
}

uint32_t RTCPos::getY2kEpoch()
{
  return (getEpoch() - EPOCH_TIME_OFF);
}

void RTCPos::setAlarmEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    setAlarmDate(tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year - EPOCH_TIME_YEAR_OFF);
    setAlarmTime(tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
  }
}

void RTCPos::setEpoch(uint32_t ts)
{
  if (_configured) {
    if (ts < EPOCH_TIME_OFF) {
      ts = EPOCH_TIME_OFF;
    }

    time_t t = ts;
    struct tm* tmp = gmtime(&t);

    RTC_MODE2_CLOCK_Type clockTime;

    clockTime.bit.YEAR = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
    clockTime.bit.MONTH = tmp->tm_mon + 1;
    clockTime.bit.DAY = tmp->tm_mday;
    clockTime.bit.HOUR = tmp->tm_hour;
    clockTime.bit.MINUTE = tmp->tm_min;
    clockTime.bit.SECOND = tmp->tm_sec;

    RTC->MODE2.CLOCK.reg = clockTime.reg;

    RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_CLOCK);
  }
}

void RTCPos::setY2kEpoch(uint32_t ts)
{
  if (_configured) {
    setEpoch(ts + EPOCH_TIME_OFF);
  }
}

/*
 * Private Utility Functions
 */

/* Configure the 32768Hz Oscillator */
void RTCPos::config32kOSC()
{
  OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(CONF_XOSC32K_STARTUP) | (CONF_XOSC32K_ONDEMAND << OSC32KCTRL_XOSC32K_ONDEMAND_Pos)
	        | (CONF_XOSC32K_RUNSTDBY << OSC32KCTRL_XOSC32K_RUNSTDBY_Pos)
	        | (CONF_XOSC32K_EN1K << OSC32KCTRL_XOSC32K_EN1K_Pos) | (CONF_XOSC32K_EN32K << OSC32KCTRL_XOSC32K_EN32K_Pos)
	        | (CONF_XOSC32K_XTALEN << OSC32KCTRL_XOSC32K_XTALEN_Pos)
	        | (CONF_XOSC32K_ENABLE << OSC32KCTRL_XOSC32K_ENABLE_Pos);

  OSC32KCTRL->RTCCTRL.reg  = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K_Val;

}

/* Configure MCLK */
void RTCPos::configMCLK()
{
    MCLK->APBAMASK.reg |= MCLK_APBAMASK_RTC; // turn on digital interface clock
}


void RTCPos::RTCMode2_configCTRLA()
{
  RTC->MODE2.CTRLA.reg = 1 << RTC_MODE2_CTRLA_CLOCKSYNC_Pos // CLOCK Read Synchronization Enable: disabled
                         | 11 << RTC_MODE2_CTRLA_PRESCALER_Pos // Setting: 11
                         | 0 << RTC_MODE2_CTRLA_MATCHCLR_Pos   // Clear on Match: disabled
                         | 0 << RTC_MODE2_CTRLA_CLKREP_Pos     // Clock Representation: disabled
                         | 0x2 << RTC_MODE2_CTRLA_MODE_Pos;   // Operating Mode: 0x2

   RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_SWRST | RTC_MODE2_SYNCBUSY_ENABLE | RTC_MODE2_SYNCBUSY_CLOCKSYNC);

}

void RTCPos::RTCMode2_configALARM0()
{
  NVIC_EnableIRQ(RTC_IRQn); // enable RTC interrupt
  NVIC_SetPriority(RTC_IRQn, 0x00);

  RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0; // enable alarm interrupt
  RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = MATCH_OFF; // default alarm match is off (disabled)

  RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_ALARM0);

}

/* Wait for sync in write operations */

inline bool RTCPos::RTCMode2_isSyncing(hri_rtcmode2_syncbusy_reg_t reg)
{
  return (RTC->MODE2.SYNCBUSY.reg & reg);
}

void RTCPos::RTCMode2_waitforSync(hri_rtcmode2_syncbusy_reg_t reg)
{
    while (RTCMode2_isSyncing(reg))
    ;
}

void RTCPos::RTCMode2_disable()
{
  RTC->MODE2.CTRLA.reg &= ~RTC_MODE2_CTRLA_ENABLE; // disable RTC
   RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_SWRST | RTC_MODE2_SYNCBUSY_ENABLE | RTC_MODE2_SYNCBUSY_CLOCKSYNC);

}

void RTCPos::RTCMode2_enable()
{
  RTC->MODE2.CTRLA.reg |= RTC_MODE2_CTRLA_ENABLE; // enable RTC
   RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_SWRST | RTC_MODE2_SYNCBUSY_ENABLE | RTC_MODE2_SYNCBUSY_CLOCKSYNC);

}

void RTCPos::RTCMode2_reset()
{
  RTC->MODE2.CTRLA.reg |= RTC_MODE2_CTRLA_SWRST; // software reset
   RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_SWRST);
}

void RTCPos::RTCMode2_resetRemove()
{
  RTC->MODE2.CTRLA.reg &= ~RTC_MODE2_CTRLA_SWRST; // software reset remove
   RTCMode2_waitforSync(RTC_MODE2_SYNCBUSY_SWRST);
}
