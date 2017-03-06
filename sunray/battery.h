#ifndef BATTERY_H
#define BATTERY_H



class BatteryClass {
  public:
    bool batMonitor;
	  float batteryFactor;
    float currentFactor;
    float batGoHomeIfBelow;
    float startChargingIfBelow;
    float batSwitchOffIfBelow;  // switch off battery if below voltage (Volt)
    int batSwitchOffIfIdle;      // switch off battery if idle (minutes)  
    float batFullCurrent;
	  float batteryVoltage;
	  float chargingVoltage;
	  float chargingCurrent;
    bool chargingEnabled;
	  float ADCRef;
    void begin();            
    void run();	  
	  bool chargerConnected();
    void enableCharging(bool flag);   	  
    void allowSwitchOff(bool flag);      
    bool shouldGoHome();    
    void resetIdle();
  protected:           
    bool chargerConnectedState;
    bool switchOffAllowed;
    unsigned long switchOffTime;
    unsigned long chargingStartTime;
	  unsigned long nextCheckTime;	  
};

extern BatteryClass Battery;

#endif

