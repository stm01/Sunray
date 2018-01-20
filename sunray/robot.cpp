/*
 robot state machine, tracking, mowing
 */
      
#include "robot.h"
#include "i2c.h"
#include "modelrc.h"
#include "buzzer.h"
#include "bumper.h"
#include "config.h"
#include "remotectl.h"
#include "flashmem.h"
#include "helper.h"
#include "motor.h"
#include "map.h"
#include "battery.h"
#include "pinman.h"
#include "settings.h"
#include "bt.h"
#include "sonar.h"
#include "robotmsg.h"


#define MAGIC 52

RobotClass Robot;



// RobotState {  STAT_IDLE, STAT_CAL_GYRO, STAT_TRACK, STAT_CREATE_MAP, STAT_MOW, STAT_RC };
const char* robotStateNames[] = { "IDLE", "GYRO", "TRAK", "MAP ", "MOW ", "R/C " };



RobotClass::RobotClass(){
}


// robot start
void RobotClass::begin(){
  watchdogSetup();
  watchdogEnable(10000); // 10 Seconds
 
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  

  Buzzer.begin();
  ROBOTMSG.begin(ROBOTMSG_BAUDRATE);    
//  receiveEEPROM_or_ERASE(); 
  I2C_reset();
  Wire.begin();            
	Settings.begin();
  PinMan.begin();  
  ADCMan.begin();      
  //RemoteCtl.begin();      
  
  //while (!Console) ; // required if using Due native port
  DEBUGLN(F("SETUP done!"));
  DEBUGLN(F("WD Reset in 10 Seconds"));
    
  //dameda, deaktivieren zum testen
  
  Battery.begin();    
  RANGING.begin(RANGING_BAUDRATE);
  Bumper.begin();	
  IMU.begin(); 
  Motor.begin();      
  Perimeter.begin(pinPerimeterLeft, pinPerimeterRight);
	Sonar.begin();
  Map.begin();
  DEBUG(F("freeRam="));
  DEBUGLN(freeRam());
  
  RC.begin();
  
  state = STAT_IDLE;
  lastState = state;
  mowState = MOW_LINE;
	lastMowState = mowState;
  mowPattern = PATTERN_NONE;
  trackState = TRK_RUN;  
	trackClockwise = true;

	sensorTriggerStatus = 0;
  nextControlTime = 0;  
  nextInfoTime = 0;  
  nextIMUTime = 0;
  loopCounter = 0;
  loopsPerSec = 0;
	loopsPerSecSmooth = 99;
  trackLineTimeout = 0;
  mowingAngle = 0;
  mowingDirection = 0;
  lastStartLineTime; 
	trackSpeedPerc = 0.5;
	trackRotationSpeedPerc = 0.3;
	rotationSpeedPerc = 0.3;
	reverseSpeedPerc = 0.3;
  
	if (!ADCMan.calibrationAvail) ADCMan.calibrate();
	ADCMan.printInfo();
  Buzzer.sound(SND_READY, true);
  DEBUGLN(F("Begin done!"));   
}

void RobotClass::configureBluetooth(){
  BluetoothConfig bt; 
  bt.setParams(F("Ardumower"), 1234, 115200, true);
}

void RobotClass::resetSensorTriggers(){
	sensorTriggerStatus = 0;
}

// robot main loop
void RobotClass::run(){
  DEBUG(F("MILLIS:"));
  DEBUG(F(millis()));
  watchdogReset();
  DEBUGLN(F("WD Reset!"));
  
  if (millis() >= nextInfoTime){
    nextInfoTime = millis() + 1000;
    loopsPerSec = loopCounter;
		loopsPerSecSmooth = 0.99 * loopsPerSecSmooth + 0.01 * loopsPerSec;
		if (state != STAT_IDLE) {
			if (loopsPerSecSmooth < 50){
				setIdle();
				DEBUGLN(F("MAIN LOOP ERROR (loopsPerSec) - check I2C cables"));            
				Buzzer.sound(SND_TILT, true); 	   
			}
		}
    loopCounter=0;

    //DEBUG("DIST=");
    //DEBUGLN(Motor.distToLine);
    //ADCMan.printInfo();
    //DEBUG("ADCconvs=");
    //DEBUGLN(ADCMan.getConvCounter());
        
		resetSensorTriggers();		
    
	  if ( (!RC.enable) && (IMU.needGyroCal()) ) {     
	    Motor.setPaused(true);
      lastState = state;
	    IMU.startGyroCalibration();
      state = STAT_CAL_GYRO;
    }		
		if (Motor.paused){
			if (IMU.state != IMU_CAL_GYRO){
		      state = lastState;
          Motor.setPaused(false);
          Perimeter.resetTimedOut();
		    }
		}
  }
  if (millis() >= nextIMUTime){    
    nextIMUTime = millis() + 10; // 100 Hz
    IMU.run();
		loopCounter++;
  }

  Buzzer.run();
	ADCMan.run();
	Sonar.run();

  if (millis() >= nextControlTime){    
    nextControlTime = millis() + 200; // 5 Hz

    if ( (state != STAT_CHG) && (Battery.chargerConnected()) ){
      Motor.stopImmediately();
      state = STAT_CHG;      
    }
    Bumper.run();
    RC.run();
    Motor.run();        
    Perimeter.run();
    IMU.run();
	  Map.run();
    //RemoteCtl.run();    
    Battery.run();    

    stateMachine();
				
    if ( (!RC.enable) && ((state != STAT_IDLE) && (state != STAT_CAL_GYRO)) && (state != STAT_CHG) ) 
    {
	    if (Perimeter.signalTimedOut()){
        setIdle();
	      DEBUG(F("PERIMETER TIMEOUT: smag="));
        DEBUG(Perimeter.getSmoothMagnitude(IDX_LEFT));            
        DEBUG(F(","));
        DEBUGLN(Perimeter.getSmoothMagnitude(IDX_RIGHT));            
	      Buzzer.sound(SND_PERIMETER_TIMEOUT, true); 	   
      }
		
	    if ( (fabs(IMU.ypr.roll/PI*180)>30) || (fabs(IMU.ypr.pitch/PI*180)>30) ){
  	    setIdle();
	      DEBUGLN(F("TILT"));
	      Buzzer.sound(SND_TILT, true); 	   
	    }
	  }		

    /*ROBOTMSG.print(F("!04"));     
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Map.robotState.x);
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Map.robotState.x);
    ROBOTMSG.print(F(","));       
    ROBOTMSG.println(Map.robotState.orientation);  */
    
    //loopCounter++;
  }
	
	RobotMsg.run();

  //serveHTTP();    
}

void RobotClass::setIdle(){
  state = STAT_IDLE;
  Motor.stopImmediately(); 
}

void RobotClass::startMapping(){
  Map.clearOutline();
  state = STAT_CREATE_MAP;
  trackState = TRK_FIND;  
  mowState = MOW_LINE;  
  Motor.travelLineDistance(10000, IMU.getYaw(), trackSpeedPerc);
}

void RobotClass::startLaneMowing(){
  mowPattern = PATTERN_LANES;
  state = STAT_MOW;
  mowState = MOW_LINE;
  mowingAngle = IMU.getYaw();
  mowingDirection = mowingAngle-PI/2;
  Motor.travelLineDistance(3000, mowingAngle, 1.0);
}

void RobotClass::startRandomMowing(){
  mowPattern = PATTERN_RANDOM;
  state = STAT_MOW;
  mowState = MOW_LINE;
  mowingAngle = IMU.getYaw();
  mowingDirection = mowingAngle-PI/2;
  Motor.travelLineDistance(100000, mowingAngle, 1.0);
}

void RobotClass::startTrackingForEver(){  
  mowPattern = PATTERN_NONE;
  state = STAT_TRACK;  
  trackState = TRK_FIND;  
  Motor.travelLineDistance(10000, IMU.getYaw(), trackSpeedPerc);
}



String RobotClass::getStateName(){
  return String(robotStateNames[state]);
}



void RobotClass::sensorTriggered(uint16_t sensorID){
	sensorTriggerStatus |= sensorID;	
}

// -------------------------------- state machine --------------------------------


void RobotClass::track(){
  float leftMag = Perimeter.getMagnitude(IDX_LEFT);
  float rightMag = Perimeter.getMagnitude(IDX_RIGHT);  	
	boolean leftIn = (leftMag < 0);
  boolean rightIn = (rightMag < 0);  	
	float rotationSign = 1;
	if (trackClockwise) {
		leftIn = 	(rightMag < 0);
		rightIn = 	(leftMag < 0);
		rotationSign = -1;
	}	  
	float startDist;       	
  switch (trackState){
    case TRK_FIND:      
	    if  ((!leftIn) || (!rightIn)){
        Motor.stopImmediately();
        trackState = TRK_RUN;
        trackAngle = IMU.getYaw();
        Map.distributeParticlesOutline();
      }
      break;
    case TRK_RUN:	    
	    if ((leftIn) && (!rightIn)){        
        Motor.travelLineTime(300, trackAngle, trackSpeedPerc);
        //Motor.travelLineDistance(5, IMU.getYaw(), speed);
        //Motor.setSpeedPWM(speed, speed);
      }
      else {       
       if (!leftIn) {          
          //Motor.setSpeedPWM(-speedRot, speedRot);
          //Motor.rotateAngle(IMU.getYaw()+PI/180.0*3, speedRot);          
          Motor.rotateTime(300, rotationSign * trackRotationSpeedPerc);
          trackAngle = IMU.getYaw();
        } else {          
          //Motor.setSpeedPWM(speedRot, -speedRot);
          //Motor.rotateAngle(IMU.getYaw()-PI/180.0*3, speedRot);          
          Motor.rotateTime(300, rotationSign * (-trackRotationSpeedPerc) );
          trackAngle = IMU.getYaw();
        }                    
      }      
      
      if (state == STAT_CREATE_MAP){
        startDist = Map.distanceToStart(Map.robotState.x,Map.robotState.y);
        //DEBUGLN(startDist);
		    if (startDist < 0.3) {
          Motor.stopImmediately();
          Buzzer.sound(SND_READY, true);
          state = STAT_IDLE;
          Map.correctOutline();
          Map.transferOutlineToMap();
          //Map.distributeParticlesOutline();
		      Map.saveMap();
          if (mowPattern != PATTERN_NONE){
            trackState = TRK_ROTATE;
            Motor.rotateAngle(IMU.getYaw()+PI/180.0*90, fabs(trackRotationSpeedPerc));
          }
        }
      }	  
	    break;
    case TRK_ROTATE:
      if (Motor.motion == MOT_STOP){
        mowingAngle = IMU.getYaw();
        mowingDirection = mowingAngle-PI/2;
        state = STAT_IDLE;
        /*state = STAT_MOW;
        mowState = MOW_LINE;
        Motor.travelLineDistance(300, mowingAngle, speed);*/
      }
      break;	  
  }
  /*if (sign(mag) != sign(lastPerimeterMag)){
    // inside/outside transition
	  trackLineTimeout = millis() + 1500;
  }
  lastPerimeterMag = mag;*/
}


// random mowing
void RobotClass::mowRandom(){  	  
	if (Bumper.pressed()){ 
	  if (mowState != MOW_AVOID_OBSTACLE){
			// avoid obstacle			
			lastMowState = mowState;
			if (Motor.speedRpmSet >= 0){			
				obstacleSide = FRONT;
			} else {
				obstacleSide = BACK;
			}			
			mowState = MOW_AVOID_OBSTACLE;		
		}		
	} else {
		if (!Perimeter.isInside()){
		  if (mowState != MOW_AVOID_ESCAPE){
				// avoid escaping perimeter		  											
				lastMowState = mowState;
				mowState = MOW_AVOID_ESCAPE;		
			}
		}	
	}
	
	switch (mowState){
    case MOW_AVOID_OBSTACLE:
		  if (Bumper.pressed()){ 
				if (obstacleSide == FRONT){
					Motor.travelLineTime(300, IMU.getYaw(), -0.5);
				}	else {
					Motor.travelLineTime(300, IMU.getYaw(), 0.5);
				}
			} else {				
				mowState = MOW_REV;			
			}
			break;				
		case MOW_AVOID_ESCAPE:
			if (!Perimeter.isInside()){
				//Motor.rotateTime(300, rotationSpeedPerc);								
				//Motor.rotateAngle(IMU.getYaw()+PI/180.0*45, rotationSpeedPerc);
				Motor.travelLineTime(300, IMU.getYaw(), -0.5);	
			}	else {
				mowState = MOW_REV;
			}	
			break;					
		case MOW_ROTATE:
      if (Motor.motion == MOT_STOP){        
        mowState = MOW_LINE;
        Motor.travelLineDistance(100000, mowingAngle, 1.0);
        //DEBUGLN(F("MOW_LINE"));        
      }
      break;
    case MOW_REV:
      if (Motor.motion == MOT_STOP){
        mowingAngle = scalePI( mowingAngle + PI + ((float)random(-90, 90)/180.0*PI ));
        rotateAngle = mowingAngle;        
        Motor.rotateAngle(rotateAngle, rotationSpeedPerc);
        mowState = MOW_ROTATE;
        //DEBUGLN(F("MOW_ROTATE"));
      }
      break;    
    case MOW_LINE:
      //if (!Perimeter.isInside()) Motor.stopSlowly();
      if ( (!Perimeter.isInside()) || (Motor.motion == MOT_STOP)  ){              
				Motor.stopImmediately();
        Motor.travelLineDistance(50, mowingAngle, -reverseSpeedPerc);
        mowState = MOW_REV;
        //DEBUGLN(F("MOW_REV"));
      }
      break;
  }
}


// lane-by-lane mowing
void RobotClass::mowLanes(){  	  		
	switch (mowState){
		case MOW_ROTATE:
      if (Motor.motion == MOT_STOP){        
        Motor.travelLineDistance(15, rotateAngle, 1.0);
        mowState = MOW_ENTER_LINE;
        //DEBUGLN(F("MOW_ENTER_LINE"));        
      }
      break;
    case MOW_REV:
      if (Motor.motion == MOT_STOP){      
				unsigned long duration = millis()-lastStartLineTime;
				DEBUG(F("duration="));
				DEBUGLN(duration);
				if (duration < 5000){
					DEBUGLN(F("new lane direction"));
					mowingAngle = scalePI( mowingDirection + PI );
					mowingDirection = mowingAngle-PI/2;
				} else {
					mowingAngle = scalePI( mowingAngle + PI );
				}
				float enterDelta = PI/4;
				float yaw = IMU.getYaw();
				float deltaAngle = distancePI(mowingAngle, mowingDirection); // w-x
				if (deltaAngle > 0) rotateAngle = mowingAngle + enterDelta;
					else rotateAngle = mowingAngle - enterDelta;
        
        Motor.rotateAngle(rotateAngle, rotationSpeedPerc);
        mowState = MOW_ROTATE;
        //DEBUGLN(F("MOW_ROTATE"));
      }
      break;
    case MOW_ENTER_LINE:
      if (Motor.motion == MOT_STOP){
        Motor.travelLineDistance(100000, mowingAngle, 1.0);
        mowState = MOW_LINE;
        //DEBUGLN(F("MOW_LINE"));
        lastStartLineTime = millis();
      }
      break;
    case MOW_LINE:
      //if (!Perimeter.isInside()) Motor.stopSlowly();
      if ( (!Perimeter.isInside()) || (Motor.motion == MOT_STOP)  ){              
				Motor.stopImmediately();
        Motor.travelLineDistance(50, mowingAngle, -reverseSpeedPerc);
        mowState = MOW_REV;
        //DEBUGLN(F("MOW_REV"));
      }
      break;
  }
}


/*
if outside:
* if 'inside->outside' transition in forward motion => reverse until inside
* if 'inside->outside' transition in reverse motion => forward until inside

if inside:
* if bumper in forward motion => slightly reverse
* if bumper in reverse motion => slightly forward
*/

void RobotClass::stateMachine(){    
	 		
	switch (state){		
		case STAT_CAL_GYRO:	      
			break;
		case STAT_IDLE:
			break;
		case STAT_MOW:						
			switch (mowPattern){
				case PATTERN_RANDOM: 
					mowRandom();
					break;
				case PATTERN_LANES:
					mowLanes();
					break;				
			}			
			break;      
		case STAT_CREATE_MAP:
		case STAT_TRACK:
			track();
			break;
	}
}
