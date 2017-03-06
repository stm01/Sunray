
// model R/C receiver

#ifndef MODELRC_H
#define MODELRC_H


#include <Arduino.h>


class ModelReceiverControl 
{
  public:
    bool enable;
    int remoteSteer ;  // range -100..100
    int remoteSpeed ;  // range -100..100      
    int remoteMow;  // range -100..100      
    int remoteSwitch;
    ModelReceiverControl();    
    void begin();
    void run();
    void print();
    void setRemotePPMState(unsigned long timeMicros, boolean remoteSpeedState, boolean remoteSteerState, 
      boolean remoteMowState, boolean remoteSwitchState);
  protected:
    unsigned long nextInfoTime;
    bool remoteSteerLastState;
    bool remoteSpeedLastState;
    bool remoteMowLastState;
    bool remoteSwitchLastState;
    unsigned long remoteSteerLastTime;
    unsigned long remoteSpeedLastTime;
    unsigned long remoteMowLastTime;
    unsigned long remoteSwitchLastTime;    
    int rcValue(int ppmTime);    
};

extern ModelReceiverControl RC;

#endif


