#ifndef PKAE_Timer_H
#define PKAE_Timer_H
#include "arduino.h"

class PKAE_Timer {
public:
     PKAE_Timer(unsigned int delay);
     void Reset();
     boolean IsTimeUp(unsigned int nDynamicDelay = 0, boolean lReset = true);
     unsigned long nStarted ;
     unsigned long nLastStarted ;
     unsigned int  nCount;


private:

     unsigned int _delay;


};

#endif