#include "battery.h"
#include "config.h"
#include "motor.h"
#include "buzzer.h"
#include "adcman.h"
#include <Arduino.h>

BatteryClass Battery;

void BatteryClass::begin()
{
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  enableCharging(false);
  allowSwitchOff(false);
  resetIdle();
  
  ADCMan.setupChannel(pinChargeCurrent, 1, false);
  ADCMan.setupChannel(pinBatteryVoltage, 1, false);   
  ADCMan.setupChannel(pinChargeVoltage, 1, false);   
  
  nextCheckTime = 0;
	timeMinutes=0;
  chargerConnectedState = false;    
  
  batteryFactor = (47+5.1) / 5.1;    // ADC voltage to battery voltage
  currentFactor = 1.0;         // ADC voltage to current ampere
  batMonitor = true;              // monitor battery and charge voltage?    
  batGoHomeIfBelow = 23.7;     // drive home voltage (Volt)  
  batSwitchOffIfBelow = 21.7;  // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = 1;      // switch off battery if idle (minutes)
  batFullCurrent  = 0.4;      // current flowing when battery is fully charged
  startChargingIfBelow = 26.0; // start charging if battery Voltage is below  
  batteryVoltage = 0;
}


void BatteryClass::enableCharging(bool flag){
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag;
  digitalWrite(pinChargeRelay, flag);      
	nextPrintTime = 0;  	   	   	
}

bool BatteryClass::chargerConnected(){
  return (chargingVoltage > 5);
}
  

bool BatteryClass::shouldGoHome(){
  return (batteryVoltage < batGoHomeIfBelow);
}

void BatteryClass::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void BatteryClass::allowSwitchOff(bool flag){
  switchOffAllowed = flag;
  if (flag) resetIdle();
}

void BatteryClass::print(){
	ROBOTMSG.print(F("!88,"));
	ROBOTMSG.print(timeMinutes);
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(batteryVoltage);
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(chargingVoltage);    
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(chargingCurrent);			
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(chargingEnabled);
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(chargerConnected());
	ROBOTMSG.print(F(","));       
	ROBOTMSG.print(switchOffAllowed);			
	ROBOTMSG.println();        			
}

void BatteryClass::run(){  
  chargingVoltage = ((float)ADCMan.getVoltage(pinChargeVoltage)) * batteryFactor;  
  float w = 0.9;
  if (batteryVoltage < 5) w = 0;
  batteryVoltage = w * batteryVoltage + (1-w) * ((float)ADCMan.getVoltage(pinBatteryVoltage)) * batteryFactor;  
  chargingCurrent = 0.9 * chargingCurrent + 0.1 * ((float)ADCMan.getVoltage(pinChargeCurrent)) * currentFactor;    
	
	if (chargerConnected()){           
      if (!chargerConnectedState){
	      chargerConnectedState = true;		    
		    DEBUGLN(F("CHARGER CONNECTED"));      	              
        Buzzer.sound(SND_OVERCURRENT, true);
        enableCharging(true);        				
	    }
  } else {
      if (chargerConnectedState){        
        chargerConnectedState = false;
        DEBUGLN(F("CHARGER DISCONNECTED"));              				
        enableCharging(false);				
      }
  }      		
  
  if (millis() >= nextCheckTime){    
    nextCheckTime = millis() + 5000;  	   	   	
    timeMinutes = (millis()-chargingStartTime) / 1000 /60;
    if (switchOffAllowed){
      if (millis() >= switchOffTime){
        Buzzer.sound(SND_OVERCURRENT, true);
        DEBUGLN(F("SWITCHING OFF"));              
        digitalWrite(pinBatterySwitch, LOW);    
      } else digitalWrite(pinBatterySwitch, HIGH);  
    }
    	  
    if (chargerConnected()){           
      if (chargerConnectedState){	      
        // charger in connected state
        if (chargingEnabled){
          if ((timeMinutes > 180) || (chargingCurrent < batFullCurrent)) {        
            // stop charging
            enableCharging(false);
          }          
        } else {
           if (batteryVoltage < startChargingIfBelow) {
              // start charging
              enableCharging(true);
              chargingStartTime = millis();  
          }        
        }
      }      
    } 
		
		if (millis() >= nextPrintTime){
			nextPrintTime = millis() + 60000;  	   	   	
			print();			
			/*DEBUG(F("charger conn="));
			DEBUG(chargerConnected());
			DEBUG(F(" chgEnabled="));
			DEBUG(chargingEnabled);
			DEBUG(F(" chgTime="));      
			DEBUG(timeMinutes);
			DEBUG(F(" charger: "));      
			DEBUG(chargingVoltage);
			DEBUG(F(" V  "));    
			DEBUG(chargingCurrent);   
			DEBUG(F(" A "));         
			DEBUG(F(" bat: "));
			DEBUG(batteryVoltage);
			DEBUG(F(" V  "));    
			DEBUG(F("switchOffAllowed="));   
			DEBUG(switchOffAllowed);      
			DEBUGLN();      */					
    }	
  }
}


