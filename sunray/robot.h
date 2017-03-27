
/* 
Particle filter:
The robot mower uses odometry, an IMU (gyro+compass) and a perimeter to localize itself on the perimeter 
and within the perimeter. A particle filter estimates the robot position based on moved distance 
and absolute angle. 

Gyro correction by compass:
During normal operation only the gyro is used for heading estimation - because a gyro drifts, every 3 minutes 
all motors are turned off and the gyro is corrected by the compass.
*/

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>  

#include "pid.h"
#include "imu.h"
#include "adcman.h"
#include "perimeter.h"
#include "remotectl.h"
 

// finate state machine states
enum RobotState {  STAT_IDLE, STAT_CAL_GYRO, STAT_TRACK, STAT_CREATE_MAP, STAT_MOW, STAT_RC, STAT_CHG };
typedef enum RobotState RobotState;

// tracking
enum TrackState {  TRK_RUN, TRK_FIND, TRK_ROTATE };
typedef enum TrackState TrackState;

// mowing
enum MowState { MOW_ROTATE, MOW_REV, MOW_LINE, MOW_ENTER_LINE } ;
typedef enum MowState MowState;

// mow pattern
enum MowPattern { PATTERN_NONE, PATTERN_RANDOM, PATTERN_LANES } ;
typedef enum MowPattern MowPattern;


class RobotClass
{
  public:    
    float batteryFactor;
    float mowingAngle;
    float rotateAngle;
    float trackAngle;
    float mowingDirection;      
	  unsigned long lastStartLineTime;
    unsigned long loopCounter;
	  RobotState state;
	  RobotState lastState;
	  TrackState trackState;
	  MowState mowState;
	  MowPattern mowPattern;
	  byte loopsPerSec;
    RobotClass();
    void begin();
    void run();    
	  String getStateName();
	  void startTrackingForEver();
	  void setIdle();
	  void startMapping();
	  void startLaneMowing();
	  void startRandomMowing();
	  void configureBluetooth();
	  void sendMap();
    void sendParticles();
    void sendPerimeterOutline();
  protected:    
    int lastPerimeterMag;
	  unsigned long trackLineTimeout;
    unsigned long nextInfoTime;
    unsigned long nextIMUTime; 
	  unsigned long nextControlTime; 
    String content;
    String receivedData;
    int hdrLen;
	  void stateMachine();
	  void track();
	  void mow();	
    void receiveEEPROM_or_ERASE();
    void printSensorData();
    void readRobotMessages();
    void sendHTTPNotFound();
    void initHTTP();
    void finishHTTP();
    void sendInfo();
    void serveHTTP();
};    

extern RobotClass Robot;

#endif

