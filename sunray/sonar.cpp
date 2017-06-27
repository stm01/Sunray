#include "sonar.h"
#include "config.h"
#include "motor.h"
#include "robot.h"
#include "buzzer.h"
#include "robot.h"
#include "pinman.h"
#include "RunningMedian.h"
#include <Arduino.h>

SonarClass Sonar;

#define MAX_DURATION 4000
#define OBSTACLE 2000

RunningMedian<unsigned int,10> sonarLeftMeasurements;
RunningMedian<unsigned int,10> sonarRightMeasurements;
RunningMedian<unsigned int,10> sonarCenterMeasurements;

volatile unsigned long startTime = 0;
volatile unsigned long echoTime = 0;
volatile unsigned long echoDuration = 0;
volatile byte idx = 0;
bool added = false;
unsigned long timeoutTime = 0;
unsigned long nextEvalTime = 0;



// HC-SR04 ultrasonic sensor driver (2cm - 400cm)
void startHCSR04(int triggerPin, int aechoPin){
  unsigned int uS;            
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);   
  digitalWrite(triggerPin, LOW);      
  /*// if there is no reflection, we will get 0  (NO_ECHO)
  uS = pulseIn(echoPin, HIGH, MAX_ECHO_TIME);  
  //if (uS == MAX_ECHO_TIME) uS = NO_ECHO;
  //if (uS < MIN_ECHO_TIME) uS = NO_ECHO;
  return uS;*/  
  
}

ISR(echoLeft){
  if (idx != 0) return;
  if (digitalRead(pinSonarLeftEcho) == HIGH) {    
    startTime = micros();           
		echoTime = 0;    
  } else {    
    echoTime = micros();            
    echoDuration = echoTime - startTime;          
  }
}

ISR(echoCenter){
  if (idx != 1) return;
  if (digitalRead(pinSonarCenterEcho) == HIGH) {    
    startTime = micros();           
		echoTime = 0;    
  } else {    
    echoTime = micros();            
    echoDuration = echoTime - startTime;          
  }
}

ISR(echoRight){
  if (idx != 2) return;
  if (digitalRead(pinSonarRightEcho) == HIGH) {    
    startTime = micros();           
		echoTime = 0;    
  } else {    
    echoTime = micros();            
    echoDuration = echoTime - startTime;          
  }
}

void SonarClass::run(){      
  if (!enabled) {
		distanceRight = distanceLeft = distanceCenter = 0;
		return;
	}
	if (echoDuration != 0) {            
    added = true;
    unsigned long raw = echoDuration;    
    if (raw > MAX_DURATION) raw = MAX_DURATION;
    if (idx == 0) sonarLeftMeasurements.add(raw);        
      else if (idx == 1) sonarCenterMeasurements.add(raw);        
      else sonarRightMeasurements.add(raw);        
    echoDuration = 0;
  }
  if (millis() > timeoutTime){                    
    if (!added) {                      
      if (idx == 0) sonarLeftMeasurements.add(MAX_DURATION);        
        else if (idx == 1) sonarCenterMeasurements.add(MAX_DURATION);        
        else sonarRightMeasurements.add(MAX_DURATION);             
    }    
    //if (millis() > nextSonarTime){        
    idx = (idx + 1) % 3;		
      //nextSonarTime = millis() + 100;
    //}
    echoDuration = 0;
		if (idx == 0) startHCSR04(pinSonarLeftTrigger, pinSonarLeftEcho);        
      else if (idx == 1) startHCSR04(pinSonarCenterTrigger, pinSonarCenterEcho);        
      else startHCSR04(pinSonarRightTrigger, pinSonarRightEcho);        				  		
		timeoutTime = millis() + 50;    			 // 10
		added = false;
  }
  if (millis() > nextEvalTime){
    nextEvalTime = millis() + 200;        				
		//sonar1Measurements.getAverage(avg);      
		sonarLeftMeasurements.getLowest(distanceLeft);   
		sonarRightMeasurements.getLowest(distanceRight);   
		sonarCenterMeasurements.getLowest(distanceCenter);   				
		
		if (distanceLeft < OBSTACLE) Robot.sensorTriggered(SEN_SONAR_LEFT);
		if (distanceRight < OBSTACLE) Robot.sensorTriggered(SEN_SONAR_RIGHT);
		if (distanceCenter < OBSTACLE) Robot.sensorTriggered(SEN_SONAR_CENTER);		
  }     
}

void SonarClass::begin()
{	
	enabled = true;
	pinMode(pinSonarLeftTrigger , OUTPUT);
  pinMode(pinSonarCenterTrigger , OUTPUT);  
  pinMode(pinSonarRightTrigger , OUTPUT);

  pinMode(pinSonarLeftEcho , INPUT);  
  pinMode(pinSonarCenterEcho , INPUT);    
  pinMode(pinSonarRightEcho , INPUT);  

	attachInterrupt(pinSonarLeftEcho, echoLeft, CHANGE); 
  attachInterrupt(pinSonarCenterEcho, echoCenter, CHANGE);
  attachInterrupt(pinSonarRightEcho, echoRight, CHANGE);
  
	
	PinMan.setDebounce(pinSonarCenterEcho, 100);  // reject spikes shorter than usecs on pin
	PinMan.setDebounce(pinSonarRightEcho, 100);  // reject spikes shorter than usecs on pin
	PinMan.setDebounce(pinSonarLeftEcho, 100);  // reject spikes shorter than usecs on pin
	verboseOutput = false;
}


bool SonarClass::obstacle()
{
  if (!enabled) return false;
	return ( (distanceLeft < OBSTACLE) || (distanceRight < OBSTACLE) ||(distanceCenter < OBSTACLE) );
}


