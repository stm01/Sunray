#include "bumper.h"
#include "config.h"
#include "motor.h"
#include "buzzer.h"
#include "pinman.h"
#include "robot.h"
#include <Arduino.h>

BumperClass Bumper;


ISR(BumperLeftInterruptRoutine) {
  Bumper.leftPressed = true;	
}

ISR(BumperRightInterruptRoutine) {
  Bumper.rightPressed = true;	
}

void BumperClass::run(){    
  if (millis() >= nextCheckTime){    
    if (pressed()){
      nextCheckTime = millis() + 4000;
      if (leftPressed) Robot.sensorTriggered(SEN_BUMPER_LEFT);
			if (rightPressed) Robot.sensorTriggered(SEN_BUMPER_RIGHT);
			DEBUGLN(F("BUMPER"));      
      //Motor.stopMowerImmediately();
      Motor.stopImmediately();
      Buzzer.sound(SND_OVERCURRENT, true);			
			leftPressed = false;
			rightPressed = false;
    }
  }
}

void BumperClass::begin()
{
  leftPressed = false;
	rightPressed = false;
	pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
	attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, LOW);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, LOW);
	
	PinMan.setDebounce(pinBumperLeft, 100);  // reject spikes shorter than usecs on pin
	PinMan.setDebounce(pinBumperRight, 100);  // reject spikes shorter than usecs on pin
	
  nextCheckTime = 0;
}


bool BumperClass::pressed()
{
  return (leftPressed || rightPressed);
}

