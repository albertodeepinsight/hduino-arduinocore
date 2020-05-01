/*
  Simple RTC Alarm for HDuino Poseidon

  Demonstrates how to set an RTC alarm for the HDuino Poseidon

  This example code is in the public domain

  http://arduino.cc/en/Tutorial/SimpleRTCAlarm

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  25 Sept 2015
  modified
  21 Oct 2015
  modified by Alberto Rodríguez <alberto.rodriguez@deep-insight.es>
  28 Apr 2020
*/

#include <RTCPos.h>

/* Create an rtc object */
RTCPos rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
const byte day = 25;
const byte month = 9;
const byte year = 15;

void setup()
{
  Serial.begin(9600);

  rtc.begin(); // initialize RTC 24H format

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.setAlarmTime(16, 0, 10);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  
  rtc.attachInterrupt(alarmMatch);
}

void loop()
{

}

void alarmMatch()
{
  Serial.println("Alarm Match!");
}