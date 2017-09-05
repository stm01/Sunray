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
	if (Bumper.leftPressed) Robot.sensorTriggered(SEN_BUMPER_LEFT);
	if (Bumper.rightPressed) Robot.sensorTriggered(SEN_BUMPER_RIGHT);							
	if (Bumper.changed()){
		if (Bumper.pressed()){
			DEBUGLN(F("BUMPER PRESSED"));      						
			//Motor.stopImmediately();
			Buzzer.sound(SND_OVERCURRENT, true);						    			
		} else {
			DEBUGLN(F("BUMPER RELEASED"));      
		} 		
	}			
}


