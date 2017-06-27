/* ROBOTMSG protocol - Requires 'processing_sunray' app on the client side (PC/Mac/Linux)
 * 
 * robot messages
 *  01 : sensor data
 *  03 : map bitmap data  
 *  05 : perimeter outline data
 *  11 : tracking forever
 *  12 : start mapping
 *  13 : mowing lanes
 *  14 : mowing random
 *  15 : particles data
 *  16 : distribute particles on perimeter
 *  17 : robot motion data (distance, orientation) 
 *  70 : configure bluetooth  
 *  75 : erase microcontroller flash memory
 *  76 : eeprom data
 
 * battery messages
 *  88 : battery data
 
 * perimeter messages
 *  84 : perimeter settings
 
 * sonar messages
 *  87 : sonar data (verbose)
 
 * motor messages
 *  00 : stop immediately
 *  02 : set motor pwm (left, right)
 *  85 : travel angle distance (speed, distance, orientation)
 *  06 : travel line distance (speed, distance, orientation)
 *  07 : travel line time (speed, time, orientation) 
 *  08 : rotate angle (speed)
 *  09 : rotate time (speed) 
 *  74 : set mow motor pwm
 *  83 : motor settings 
 *  86 : motor controller data
 
 * ADC messages
 *  71 : calibrate ADC
 
 * IMU messages
 *  04 : IMU data (verbose)
 *  10 : calibrate gyro
 *  72 : calibrate compass 
 *  73 : toggle verbose
 *  78 : compass calibration data (centre X,Y,Z,radii X,Y,Z)
 *  79 : IMU self test
 *  80 : start compass calibration  
 *  81 : stop compass calibration  
 *  82 : IMU settings
 
 * ranging messages
 *  77 : ranging data (time, address, distance, power)

*/ 

 
#include "robotmsg.h"
#include "config.h"
#include "robot.h"
#include "motor.h"
#include "imu.h"
#include "map.h"
#include "bt.h"
#include "battery.h"
#include "sonar.h"
#include "helper.h"
#include "flashmem.h"
#ifndef __AVR
  #include <Reset.h>
#endif


RobotMsgClass RobotMsg;


void RobotMsgClass::begin(){
	nextInfoTime = 0;	
}

void RobotMsgClass::receiveEEPROM_or_ERASE(){
  boolean received = false;
  int addr;
  byte data;
  int cmd;
  char ch;
  
  // wait for serial data (Arduino IDE ERASE command or PC EEPROM data) 
  while (millis() < 1000);  
  
  Flash.verboseOutput=false;    
  while (ROBOTMSG.available()){                           
    char ch = ROBOTMSG.read();    
    switch (ch){     
      // Arduino Due IDE sends:  €€#N#w00000000,4#  ( https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/ )
      case 'N': 
        ch = ROBOTMSG.read();
        if (ch == '#'){          
            #ifndef __AVR__                       
              initiateReset(1);   // Arduino Due ERASE trigger			  
			  tickReset();  // https://forums.adafruit.com/viewtopic.php?f=19&t=47844&start=30
            #endif
            while(true);
        }
        break;
      case '?':               
        cmd = ROBOTMSG.parseInt();                
        switch (cmd) {                    
          case 76: addr  = ROBOTMSG.parseInt();
                   data  = ROBOTMSG.parseInt();                   
                   Flash.write(addr, data);                   
                   /*DEBUG(addr);
                   DEBUG(F("="));
                   DEBUG(data);
                   DEBUGLN(F(", "));*/
                   received = true;
                   break;          
        }
        break;      
    }                 
    if (!ROBOTMSG.available()) delay(200);
  }
  Flash.verboseOutput=true;
  if (received) DEBUGLN(F("EEPROM received"));  
    else DEBUGLN(F("ERROR receiving EEPROM"));  
  //Flash.dump();
}


void RobotMsgClass::sendPerimeterOutline(){
  ROBOTMSG.print(F("!05"));        
  for (int i=0; i < Map.perimeterOutlineSize; i++){
    robot_state_t pt = Map.outlineParticles[i];
    ROBOTMSG.print(F(","));  
    ROBOTMSG.print(pt.x);
    ROBOTMSG.print(F(","));  
    ROBOTMSG.print(pt.y);    
  }  
  ROBOTMSG.println();
}

void RobotMsgClass::sendParticles(){
  ROBOTMSG.print(F("!15,"));  
  for (int i=0; i < OUTLINE_PARTICLES; i++){    
    ROBOTMSG.print(Map.outlineParticles[i].x);      
    ROBOTMSG.print(F(","));                     
    ROBOTMSG.print(Map.outlineParticles[i].y);      
    if (i < OUTLINE_PARTICLES-1) ROBOTMSG.print(",");                     
  }  
  ROBOTMSG.println();  
}


void RobotMsgClass::sendMap(){
  ROBOTMSG.print(F("!03,"));    
  ROBOTMSG.print(Map.mapScaleX);  
  ROBOTMSG.print(F(","));  
  ROBOTMSG.print(Map.mapScaleY);  
  ROBOTMSG.print(F(","));  
  ROBOTMSG.print(MAP_SIZE_X);
  ROBOTMSG.print(F(","));
  ROBOTMSG.print(MAP_SIZE_Y);  
  ROBOTMSG.print(F(","));
  map_data_t d;
  byte r,g,b,col;
  for (int y=0; y < MAP_SIZE_Y; y++){    
    for (int x=0; x < MAP_SIZE_X; x++){        
	    d = Map.mapData[y][x];		
		  if (d.s.state == MAP_DATA_STATE_MOWED){
          r=0; g=255; b=0;		  
      } else {
          col = 255.0*1.0/((float)(32-d.s.signal));         
          if (d.s.side == 1) {
            r=255; g=255-col; b=255-col;
			      //r=255; g=0; b=0;
          } else {
            r=255-col; g=255-col; b=255;
			      //r=0; g=0; b=255;
		      }
      }
		  //ROBOTMSG.write(Map.mapData[y][x].v);      
		  ROBOTMSG.print(r);      
		  ROBOTMSG.print(F(","));      		
		  ROBOTMSG.print(g);      
		  ROBOTMSG.print(F(","));      		
		  ROBOTMSG.print(b);      		
		  ROBOTMSG.print(F(","));      				
    }   
	  //ROBOTMSG.flush();
    //delay(100); 
  }  
  ROBOTMSG.println();
}

	
void RobotMsgClass::printSensorData(){
/*ROBOTMSG.write(0xAA);
    ROBOTMSG.write(0xBB);
    ROBOTMSG.write(0x01);
    printInt(17);    
    printLong(millis());
    ROBOTMSG.write(Robot.loopsPerSec);
    printFloat(Motor.motorLeftRpmCurr);
    printFloat(Motor.motorRightRpmCurr);
    printFloat(IMU.getYaw());   */

    ROBOTMSG.print(F("!01"));     
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(millis());
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Robot.state);   
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Robot.loopsPerSec);     
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Motor.motorLeftTicks);    
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Motor.motorRightTicks);  
    ROBOTMSG.print(F(","));     
    ROBOTMSG.print(Motor.motorLeftRpmCurr, 2);   
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Motor.motorRightRpmCurr, 2);     
    ROBOTMSG.print(F(","));     
    ROBOTMSG.print(Perimeter.getMagnitude(IDX_LEFT));     
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(Perimeter.getMagnitude(IDX_RIGHT));     
    ROBOTMSG.print(F(","));      
    //ROBOTMSG.print(IMU.getYaw(), 3);       
	  ROBOTMSG.print(Motor.angleRadCurr, 3);       
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(IMU.acc.x, 3);   
    ROBOTMSG.print(F(","));            
    ROBOTMSG.print(IMU.acc.y, 3);   
    ROBOTMSG.print(F(","));            
    ROBOTMSG.print(IMU.acc.z, 3);   
    ROBOTMSG.print(F(","));                
    ROBOTMSG.print(IMU.gravity.x, 3);        
    ROBOTMSG.print(F(","));          
    ROBOTMSG.print(IMU.gravity.y, 3);        
    ROBOTMSG.print(F(","));      
    ROBOTMSG.print(IMU.gravity.z, 3);    
    ROBOTMSG.print(F(","));          
    ROBOTMSG.print(IMU.comYaw, 3);     
    ROBOTMSG.print(F(","));          
    ROBOTMSG.print(IMU.ypr.pitch, 3);     
    ROBOTMSG.print(F(","));          
    ROBOTMSG.print(IMU.ypr.roll, 3);    
    ROBOTMSG.print(F(","));          
    ROBOTMSG.print(Map.robotState.x);    
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Map.robotState.y);
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Motor.motorLeftSense);
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Motor.motorRightSense);    
    ROBOTMSG.print(F(","));       
    ROBOTMSG.print(Motor.motorMowSense);
    ROBOTMSG.print(F(","));           
    ROBOTMSG.print(Motor.motorLeftFriction);
    ROBOTMSG.print(F(","));               
    ROBOTMSG.print(Motor.motorRightFriction);
    ROBOTMSG.print(F(","));               
    ROBOTMSG.print(Map.overallProb);
    ROBOTMSG.print(F(","));               
    ROBOTMSG.print(Battery.batteryVoltage);                   
    ROBOTMSG.print(F(","));         
    ROBOTMSG.print(Motor.motion);   
    ROBOTMSG.print(F(","));             
    ROBOTMSG.print(IMU.state);       
    ROBOTMSG.print(F(","));             
    ROBOTMSG.print(Motor.distanceCmSet);   
    ROBOTMSG.print(F(","));             
    ROBOTMSG.print(Motor.angleRadSet);   
		ROBOTMSG.print(F(","));	
	  ROBOTMSG.print(Robot.sensorTriggerStatus);  
		ROBOTMSG.print(F(","));
		ROBOTMSG.print(Sonar.distanceLeft);
		ROBOTMSG.print(F(","));
		ROBOTMSG.print(Sonar.distanceCenter);
		ROBOTMSG.print(F(","));
		ROBOTMSG.print(Sonar.distanceRight);						
    ROBOTMSG.println(); 		
}

void RobotMsgClass::readRobotMessages(){
  float pwmLeft;
  float pwmRight;
  float distance;
  float angle;
  float speed;  
  float duration;
  
    char ch = ROBOTMSG.read();    
    switch (ch){     
      case '?':               
        int cmd = ROBOTMSG.parseInt();                
        //DEBUG(ch);
        //DEBUGLN(cmd);
        switch (cmd) {          
          case 70: Robot.configureBluetooth(); break;          
          case 6:  distance = ROBOTMSG.parseFloat();
                   angle = ROBOTMSG.parseFloat();
                   speed = ROBOTMSG.parseFloat();
                   Motor.travelLineDistance(distance, angle, speed); 
                   break;                     
          case 7: duration = ROBOTMSG.parseFloat();
                  angle = ROBOTMSG.parseFloat();
                  speed = ROBOTMSG.parseFloat();
                  Motor.travelLineTime(duration, angle, speed); 
                  break;
          case 8: angle = ROBOTMSG.parseFloat();
                  speed = ROBOTMSG.parseFloat();
                  Motor.rotateAngle(scalePI(Motor.angleRadCurr+angle), speed); 
                  break;
          case 9: duration = ROBOTMSG.parseFloat();                  
                  speed = ROBOTMSG.parseFloat();
                  Motor.rotateTime(duration, speed); 
                  break;                    
          case 10: IMU.startGyroCalibration(); break;
          case 71: ADCMan.calibrate(); 
                   break;
          //case 72: IMU.startCompassCalibration(); break;
          case 11: Robot.trackClockwise = ROBOTMSG.parseInt(); 
								   Robot.startTrackingForEver(); break;
          case 12: Robot.startMapping(); break;
          case 13: Robot.startLaneMowing(); break;
          case 14: Robot.startRandomMowing(); break;
          case 73: IMU.verboseOutput = ROBOTMSG.parseInt(); 
									 Motor.verboseOutput = ROBOTMSG.parseInt(); 
									 Map.verboseOutput = ROBOTMSG.parseInt(); 
					         break;      
          case 74: Motor.setMowerPWM(ROBOTMSG.parseFloat()); break;
          case 0: Robot.setIdle(); break;
          case 2: pwmLeft = ROBOTMSG.parseFloat();
                    pwmRight = ROBOTMSG.parseFloat();                   
                    Motor.setSpeedPWM(pwmLeft, pwmRight);                    
                    /*DEBUG(pwmLeft);
                    DEBUG(",");
                    DEBUGLN(pwmRight);*/
                    break;
          case 3: sendMap(); break;         
          case 15: sendParticles(); break;
          case 16: Map.distributeParticlesOutline(); break;
          case 5: sendPerimeterOutline(); break;       
          case 78: /*IMU.comCentre.x = ROBOTMSG.parseFloat();
                  IMU.comCentre.y = ROBOTMSG.parseFloat();
                  IMU.comCentre.z = ROBOTMSG.parseFloat();
                  IMU.comRadius.x = ROBOTMSG.parseFloat();
                  IMU.comRadius.y = ROBOTMSG.parseFloat();
                  IMU.comRadius.z = ROBOTMSG.parseFloat();                  */
                  IMU.comCalA_1[0] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[1] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[2] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[3] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[4] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[5] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[6] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[7] = ROBOTMSG.parseFloat();
                  IMU.comCalA_1[8] = ROBOTMSG.parseFloat();
                  IMU.comCalB[0] = ROBOTMSG.parseFloat();
                  IMU.comCalB[1] = ROBOTMSG.parseFloat();
                  IMU.comCalB[2] = ROBOTMSG.parseFloat();
                  IMU.saveCalib();    
                  break;
          case 79: IMU.runSelfTest(); break;
          case 80: IMU.startCompassCalibration(); break;
          case 81: IMU.stopCompassCalibration(); break;
          case 82: IMU.useGyro = ROBOTMSG.parseInt(); 
                   IMU.gyroBiasDpsMax = ROBOTMSG.parseFloat();
                   IMU.mode = ((IMUMode)ROBOTMSG.parseInt());
                   DEBUGLN(F("received IMU settings"));
                   break;
          case 83: Motor.rpmMax = ROBOTMSG.parseFloat();
									 Robot.reverseSpeedPerc = ROBOTMSG.parseFloat();
									 Robot.rotationSpeedPerc = ROBOTMSG.parseFloat();
									 Robot.trackSpeedPerc = ROBOTMSG.parseFloat();
									 Robot.trackRotationSpeedPerc = ROBOTMSG.parseFloat();
									 Motor.robotMass = ROBOTMSG.parseFloat();
									 Motor.motorFrictionMin = ROBOTMSG.parseFloat();
									 Motor.motorFrictionMax = ROBOTMSG.parseFloat();
									 Motor.mowSenseMax = ROBOTMSG.parseFloat();
									 Motor.imuPID_Kp = ROBOTMSG.parseFloat();
                   Motor.imuPID_Ki = ROBOTMSG.parseFloat();
                   Motor.imuPID_Kd = ROBOTMSG.parseFloat();
                   Motor.motorLeftPID.Kp = ROBOTMSG.parseFloat();
                   Motor.motorLeftPID.Ki = ROBOTMSG.parseFloat();
                   Motor.motorLeftPID.Kd = ROBOTMSG.parseFloat();
                   Motor.motorRightPID.Kp = Motor.motorLeftPID.Kp;
                   Motor.motorRightPID.Ki = Motor.motorLeftPID.Ki;
                   Motor.motorRightPID.Kd = Motor.motorLeftPID.Kd;									 
									 Motor.stuckMaxDiffOdometryIMU = ROBOTMSG.parseFloat();
									 Motor.stuckMaxIMUerror = ROBOTMSG.parseFloat();
                   DEBUGLN(F("received motor settings"));
                   break;          
		  case 84: Perimeter.timedOutIfBelowSmag = ROBOTMSG.parseInt();
		           Perimeter.timeOutSecIfNotInside = ROBOTMSG.parseInt();
				       Perimeter.swapCoilPolarity = ROBOTMSG.parseInt();
				       DEBUGLN(F("received perimeter settings"));
				       break;
          case 85:  distance = ROBOTMSG.parseFloat();
                   angle = ROBOTMSG.parseFloat();
                   speed = ROBOTMSG.parseFloat();
                   Motor.travelAngleDistance(distance, scalePI(Motor.angleRadCurr+angle), speed); 
                   break;                     
              
        }
        break;      
    }               
}


void RobotMsgClass::run(){
	unsigned long time;
  int addr;
  float power;
  float distance;
  
  if (millis() >= nextInfoTime){
    nextInfoTime = millis() + 1000;        
    printSensorData();    
	}
	
	if ( ROBOTMSG.available()  ){        	     
    readRobotMessages();    
  }
	
	if ( (RANGING.available())  ){               
    char ch = RANGING.read();    
    switch (ch){     
      case '!':               
        int cmd = RANGING.parseInt();                
        //DEBUG(ch);
        //DEBUGLN(cmd);
        switch (cmd) {          
          case 77:  time = RANGING.parseInt();
                   addr = RANGING.parseInt();
                   distance = RANGING.parseFloat();
                   power = RANGING.parseFloat();
                   ROBOTMSG.print(F("!77,"));
                   ROBOTMSG.print(time);
                   ROBOTMSG.print(F(","));
                   ROBOTMSG.print(addr);
                   ROBOTMSG.print(F(","));
                   ROBOTMSG.print(distance);
                   ROBOTMSG.print(F(","));
                   ROBOTMSG.println(power);
                   break;                     
        }
    }
  }   	
}



	
	