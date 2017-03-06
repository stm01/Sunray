#include "remotectl.h"
#include "adcman.h"
#include "imu.h"
#include "perimeter.h"
#include "config.h"
#include "robot.h"
#include "motor.h"
#include "map.h"


RemoteControl RemoteCtl;


RemoteControl::RemoteControl(){
  pfodCmdComplete = false;
  pfodCmd = "";    
  pfodState = PFOD_OFF;  
  nextPlotTime = 0;
}

void RemoteControl::begin(){
  PFODAPP.begin(PFODAPP_BAUDRATE);
}

void RemoteControl::processMainMenu(String pfodCmd){      
  //if (pfodCmd == "mn50") ADCMan.calibrate();
    //else if (pfodCmd == "mn51") IMU.startCompassCalibration(); 
    if (pfodCmd == "mn0") Robot.setIdle();
    else if (pfodCmd == "mn1") Motor.rotateAngle(IMU.getYaw() + PI/180*45, 0.3);
  	else if (pfodCmd == "mn2") Motor.rotateAngle(IMU.getYaw() - PI/180*45, 0.3);
  	else if (pfodCmd == "mn3") Motor.travelLineDistance(30, IMU.getYaw(), 0.5);
  	else if (pfodCmd == "mn4") Motor.travelLineDistance(30, IMU.getYaw(), -0.5);  
  	else if (pfodCmd == "mn5") Motor.travelLineDistance(10000, IMU.getYaw(), 0.5); 
  	else if (pfodCmd == "mn6") Robot.startTrackingForEver(); 	
  	else if (pfodCmd == "mn7") Robot.startLaneMowing(); 	
  	else if (pfodCmd == "mn8") Robot.startRandomMowing(); 	  
    sendMainMenu(true);
}


void RemoteControl::sendMainMenu(boolean update){
  if (update) PFODAPP.print("{:"); else {
    PFODAPP.print(F("{.Main`1000"));    
  }
  PFODAPP.print(F("|du~Freq "));
  PFODAPP.print(Robot.loopsPerSec); 
  PFODAPP.print(F("|du~Odo "));
  PFODAPP.print(Motor.motorLeftTicks); 
  PFODAPP.print(F(" "));
  PFODAPP.print(Motor.motorRightTicks); 
  PFODAPP.print(F("|du~Spd "));
  PFODAPP.print(Motor.motorLeftRpmCurr); 
  PFODAPP.print(F(" "));
  PFODAPP.print(Motor.motorRightRpmCurr);   
  PFODAPP.print(F("|du~Peri "));
  PFODAPP.print(Perimeter.getMagnitude(0)); 
  PFODAPP.print(F("|du~IMU "));  
  PFODAPP.print(IMU.getYaw()/PI*180.0); 
  PFODAPP.print(F("|mn0~Stop"));   
  PFODAPP.print(F("|mn1~Rot+45")); 
  PFODAPP.print(F("|mn2~Rot-45")); 
  PFODAPP.print(F("|mn3~Forw30cm")); 
  PFODAPP.print(F("|mn4~Rev30cm")); 
  PFODAPP.print(F("|mn5~Forw")); 
  PFODAPP.print(F("|mn6~Track")); 
  PFODAPP.print(F("|mn7~MowLane")); 
  PFODAPP.print(F("|mn8~MowRand")); 
  PFODAPP.print(F("|mp~Map")); 
  PFODAPP.print(F("|pl~Plot")); 
  PFODAPP.print(F("|mn50~ADCCal")); 
  PFODAPP.print(F("|mn51~ComCal"));   
  PFODAPP.println("}");
}

void RemoteControl::run(){
  if (pfodState == PFOD_PLOT){
    if (millis() >= nextPlotTime){
	  nextPlotTime = millis() + 50;
	  PFODAPP.print((float(millis())/1000.0f));
    PFODAPP.print(F(","));
    PFODAPP.print(IMU.acc.x);
	  PFODAPP.println();
	}
  }
  else if (pfodState == PFOD_MAP){
    /*static int lastPerimeterOutlineSize=0;
	  if ((Map.perimeterOutlineSize != lastPerimeterOutlineSize) && (Map.perimeterOutlineSize > 0)){	  
	    point_t pt = Map.perimeterOutline[Map.perimeterOutlineSize-1];	  
	    PFODAPP.print(pt.x);
      PFODAPP.print(F(","));  
      PFODAPP.print(pt.y);
	    PFODAPP.println();
	    lastPerimeterOutlineSize = Map.perimeterOutlineSize;
	  }*/
  }
  bool res = false;
  while(PFODAPP.available() > 0){
    res = true;
    if (PFODAPP.available() > 0) {
      char ch = PFODAPP.read();
      //DEBUG("pfod ch=");
      //DEBUGLN(ch);
      if (ch == '}') pfodCmdComplete = true; 
        else if (ch == '{') pfodCmd = "";
        else pfodCmd += ch;                
    }
    if (pfodCmdComplete) {
      DEBUG(F("pfod cmd="));
      DEBUGLN(pfodCmd);      
      pfodState = PFOD_MENU;    	  
      if (pfodCmd == ".") sendMainMenu(false);      
        else if (pfodCmd == "pl") {
		      PFODAPP.println(F("{=plot`1000|time s`0|accY`1}"));          
          pfodState = PFOD_PLOT;
		      nextPlotTime = 0;
		    }
        else if (pfodCmd == "mp") {          
		      PFODAPP.println(F("{=map|position`0~~~x|`~~~y}"));				  
          pfodState = PFOD_MAP;
		      nextPlotTime = 0;
		      Robot.startMapping(); 
		    }
		    else if (pfodCmd.startsWith("mn")) processMainMenu(pfodCmd);		
        else {
          // no match
          PFODAPP.println(F("{}"));
        }
      PFODAPP.flush();
      pfodCmd = "";
      pfodCmdComplete = false;
    }
  }  
  //return res;
}


