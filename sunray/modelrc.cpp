
#include "modelrc.h"
#include "motor.h"
#include "config.h"
#include "buzzer.h"


ModelReceiverControl RC;


// RC remote control helper
// convert ppm time (us) to percent (-100..+100)
// ppmtime: zero stick pos: 1500 uS     
//          right stick pos: 2000 uS    
//          left stick pos: 1000 uS
int ModelReceiverControl::rcValue(int ppmTime){
  int value = (int) (((double)((ppmTime) - 1500)) / 3.4);
  if ((value < 5) && (value > -5)) value = 0;  //  ensures exact zero position
  return value;
}

// RC remote control driver
// 1. save time (uS) and RC channel states (HI/LO)    
// 2. if new state is LO, evaluate ppm time for channel
void ModelReceiverControl::setRemotePPMState(unsigned long timeMicros, boolean remoteSpeedState, boolean remoteSteerState, 
  boolean remoteMowState, boolean remoteSwitchState){
  if (remoteSpeedState != remoteSpeedLastState) {    
    remoteSpeedLastState = remoteSpeedState;
    if (remoteSpeedState) remoteSpeedLastTime = timeMicros; else remoteSpeed = rcValue(timeMicros - remoteSpeedLastTime);
  }
  if (remoteSteerState != remoteSteerLastState) {    
    remoteSteerLastState = remoteSteerState;
    if (remoteSteerState) remoteSteerLastTime = timeMicros; else remoteSteer = rcValue(timeMicros - remoteSteerLastTime);
  }
  if (remoteMowState != remoteMowLastState) {    
    remoteMowLastState = remoteMowState;
    if (remoteMowState) remoteMowLastTime = timeMicros; else remoteMow = max(0, (rcValue(timeMicros - remoteMowLastTime)+100)/2);
  }  
  if (remoteSwitchState != remoteSwitchLastState) {    
    remoteSwitchLastState = remoteSwitchState;
    if (remoteSwitchState) remoteSwitchLastTime = timeMicros; else remoteSwitch = rcValue(timeMicros - remoteSwitchLastTime);
  }  
}


// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect){   
  unsigned long timeMicros = micros();
  boolean remoteSpeedState = digitalRead(pinRemoteSpeed);
  boolean remoteSteerState = digitalRead(pinRemoteSteer);
  boolean remoteMowState = digitalRead(pinRemoteMow);    
  boolean remoteSwitchState = digitalRead(pinRemoteSwitch);    
  RC.setRemotePPMState(timeMicros, remoteSpeedState, remoteSteerState, remoteMowState, remoteSwitchState);    
}


void ModelReceiverControl::begin(){
  DEBUGLN(F("ModelReceiver::begin"));  
  enable=false;
  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;  
  remoteSteerLastTime = remoteSpeedLastTime =remoteMowLastTime =remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;
  nextInfoTime=0;
  // R/C
  pinMode(pinRemoteMow, INPUT);
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinRemoteSpeed, INPUT); 
  pinMode(pinRemoteSwitch, INPUT);         

  // enable interrupts
  #ifdef __AVR__
    // R/C
    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT4);
    PCMSK0 |= (1<<PCINT5);
    PCMSK0 |= (1<<PCINT6);
    PCMSK0 |= (1<<PCINT1);      
  #else
    // Due interrupts   
    attachInterrupt(pinRemoteSpeed, PCINT0_vect, CHANGE);            
    attachInterrupt(pinRemoteSteer, PCINT0_vect, CHANGE);            
    attachInterrupt(pinRemoteMow, PCINT0_vect, CHANGE);   
    attachInterrupt(pinRemoteSwitch, PCINT0_vect, CHANGE);           
  #endif     
}

ModelReceiverControl::ModelReceiverControl(){  
}

void ModelReceiverControl::run(){
  if (!enable){
    if ( (millis() < 10000) && (abs(remoteSpeed > 20)) && (abs(remoteMow < 20)) ) {
      DEBUGLN(F("R/C ARMED"));
      Buzzer.sound(SND_READY, false);
      enable=true;
    }
  }  
  if (!enable) return;
  // control motor by R/C receiver
  float steer = 1.0 * (((double)remoteSteer)/100.0);
  if (remoteSpeed < 0) steer *= -1;
    
  float motorLeftSpeed  = 1.0 * (((double)remoteSpeed)/100.0) - steer;   
  float motorRightSpeed = 1.0 * (((double)remoteSpeed)/100.0) + steer; 
  float motorMowSpeed = 1.0 * (((double)remoteMow)/100.0);        
  Motor.setSpeedPWM(motorLeftSpeed, motorRightSpeed);
  Motor.setMowerPWM(motorMowSpeed);  
  if (millis() >= nextInfoTime){
    nextInfoTime = millis() + 1000;
    print();
  }
}

void ModelReceiverControl::print(){
   DEBUG(F("RC:"));
   DEBUG(remoteSpeed);
   DEBUG(F(","));
   DEBUG(remoteSteer);
   DEBUG(F(","));
   DEBUGLN(remoteMow);
}


  
