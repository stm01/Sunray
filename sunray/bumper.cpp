#include "bumper.h"
#include "config.h"
#include "motor.h"
#include "buzzer.h"
#include "pinman.h"
#include "robot.h"
#include <Arduino.h>

BumperClass Bumper;


ISR(BumperLeftInterruptRoutine) {	  
	if (!Bumper.enabled) return;	
	if (digitalRead(pinBumperLeft) == LOW) Bumper.leftPressed = true;	
    else Bumper.leftPressed = false;		
	Bumper.leftChanged = true;
}

ISR(BumperRightInterruptRoutine) {
  if (!Bumper.enabled) return;
	if (digitalRead(pinBumperRight) == LOW) Bumper.rightPressed = true;	
	  else Bumper.rightPressed = false;	
  Bumper.rightChanged = true;	
}


void BumperClass::begin()
{
  enabled = true;
	leftPressed = false;
	rightPressed = false;
	leftChanged = false;
	rightChanged = false;
	pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
	attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
	
	PinMan.setDebounce(pinBumperLeft, 100);  // reject spikes shorter than usecs on pin
	PinMan.setDebounce(pinBumperRight, 100);  // reject spikes shorter than usecs on pin
	
  nextCheckTime = 0;
}


bool BumperClass::pressed()
{
  return (leftPressed || rightPressed);
}

bool BumperClass::changed()
{
  bool res = ((leftChanged) || (rightChanged));
	leftChanged = false;
  rightChanged = false;	
  return res;	
}

void BumperClass::run()
{

}




