#include "bumper.h"
#include "config.h"
#include "motor.h"
#include "buzzer.h"
#include "robot.h"
#include <Arduino.h>

BumperClass Bumper;



void BumperClass::run(){  
  leftPressed = (digitalRead(pinBumperLeft) == LOW);
  rightPressed = (digitalRead(pinBumperRight) == LOW);

  if (millis() >= nextCheckTime){    
    if (pressed()){
      nextCheckTime = millis() + 4000;
      if (leftPressed) Robot.sensorTriggered(SEN_BUMPER_LEFT);
			if (rightPressed) Robot.sensorTriggered(SEN_BUMPER_RIGHT);
			DEBUGLN(F("BUMPER"));      
      //Motor.stopMowerImmediately();
      Motor.stopImmediately();
      Buzzer.sound(SND_OVERCURRENT, true);			
    }
  }
}

void BumperClass::begin()
{
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  nextCheckTime = 0;
}


bool BumperClass::pressed()
{
  return (leftPressed || rightPressed);
}

