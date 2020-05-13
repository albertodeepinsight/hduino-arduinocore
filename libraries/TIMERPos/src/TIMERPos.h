/*
  ZeroTimer.h - Arduino Zero M0 Timer Interrput library
  Copyright (c) 2017 Tamasa (@EHbtj).  All right reserved.
*/

#ifndef TIMERPos_h
#define TIMERPos_h

class TIMERPos
{
  public:
        TIMERPos();
		void configureTimer(unsigned long period, void (*f)());
		void startTimer();
		void disableTimer();
		void resetTimer();

  private:
        bool timerIsSyncing();

};

#endif