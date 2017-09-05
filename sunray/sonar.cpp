/*
License
Copyright (c) 2013-2017 by Alexander Grau

Private-use only! (you need to ask for a commercial-use)
 
The code is open: you can modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

The code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
  
 */

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
#define OBSTACLE_CM 40
#define ROUNDING_ENABLED false
#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57

// Conversion from uS to distance (round result to nearest cm or inch).
#define NewPingConvert(echoTime, conversionFactor) (max(((unsigned int)echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))



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
		distanceLeft = convertCm(distanceLeft);
		
		sonarRightMeasurements.getLowest(distanceRight);   
		distanceRight = convertCm(distanceRight);
		
		sonarCenterMeasurements.getLowest(distanceCenter);   				
		distanceCenter = convertCm(distanceCenter);
		
		if (distanceLeft < OBSTACLE_CM) Robot.sensorTriggered(SEN_SONAR_LEFT);
		if (distanceRight < OBSTACLE_CM) Robot.sensorTriggered(SEN_SONAR_RIGHT);
		if (distanceCenter < OBSTACLE_CM) Robot.sensorTriggered(SEN_SONAR_CENTER);		
  }     
}

void SonarClass::begin()
{	
	enabled = true;
	triggerBelow = OBSTACLE_CM;
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
	return ( (distanceLeft < OBSTACLE_CM) || (distanceRight < OBSTACLE_CM) ||(distanceCenter < OBSTACLE_CM) );
}


unsigned int SonarClass::convertCm(unsigned int echoTime) {
#if ROUNDING_ENABLED == false
	return (echoTime / US_ROUNDTRIP_CM);              // Convert uS to centimeters (no rounding).
#else
	return NewPingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
#endif
}

