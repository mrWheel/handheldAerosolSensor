#include "PKAE_Timer.h"



PKAE_Timer::PKAE_Timer(unsigned int delay = 0){

     _delay = delay;
     nStarted = millis();
     nLastStarted=0;
     nCount=0;

}


void PKAE_Timer::Reset(){

     nLastStarted = nStarted;
     nStarted=millis();

}

boolean PKAE_Timer::IsTimeUp(unsigned int nDynamicDelay, boolean lReset){

     boolean lTimeUp = false;
     unsigned int nDelayCheck = max (_delay, nDynamicDelay);

     if ((millis() - nStarted) > nDelayCheck) {

          lTimeUp=true;
          if (lReset) {
               nCount++;
               nLastStarted = nStarted;
               nStarted=millis();
          }
     }

     return(lTimeUp);

}