/* 

robot main class (robot start, robot main loop, state machine)

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
enum RobotState {  STAT_IDLE, STAT_CAL_GYRO, STAT_TRACK, STAT_CREATE_MAP, STAT_MOW, STAT_RC, STAT_CHG};
typedef enum RobotState RobotState;

// tracking
enum TrackState {  TRK_RUN, TRK_FIND, TRK_ROTATE };
typedef enum TrackState TrackState;

// mowing
enum MowState { MOW_ROTATE, MOW_REV, MOW_LINE, MOW_ENTER_LINE, MOW_AVOID_OBSTACLE, MOW_AVOID_ESCAPE } ;
typedef enum MowState MowState;

// mow pattern
enum MowPattern { PATTERN_NONE, PATTERN_RANDOM, PATTERN_LANES } ;
typedef enum MowPattern MowPattern;

// obstacle side
enum Side { FRONT, BACK };
typedef enum Side Side;

// sensor trigger IDs
#define SEN_BUMPER_LEFT          (1L<<0)
#define SEN_BUMPER_RIGHT         (1L<<1)
#define SEN_SONAR_LEFT           (1L<<2)
#define SEN_SONAR_CENTER  	     (1L<<3)
#define SEN_SONAR_RIGHT   	     (1L<<4)
#define SEN_PERIMETER_LEFT       (1L<<5)
#define SEN_PERIMETER_RIGHT      (1L<<6)
#define SEN_MOTOR_FRICTION_LEFT  (1L<<7)
#define SEN_MOTOR_FRICTION_RIGHT (1L<<8)
#define SEN_MOTOR_FRICTION_MOW   (1L<<9)
#define SEN_MOTOR_STUCK          (1L<<10)
#define SEN_MOTOR_ERROR_LEFT     (1L<<11)
#define SEN_MOTOR_ERROR_RIGHT    (1L<<12)
#define SEN_MOTOR_ERROR_MOW      (1L<<13)



class RobotClass
{
  public:    
    float mowingAngle;
    float rotateAngle;
    float trackAngle;
		float reverseSpeedPerc;
		float trackSpeedPerc;
		float trackRotationSpeedPerc;
		float rotationSpeedPerc;
    float mowingDirection;      
		uint16_t sensorTriggerStatus; // bitmap of triggered sensors
	  unsigned long lastStartLineTime;
    unsigned long loopCounter;
	  RobotState state;
	  RobotState lastState;
	  TrackState trackState;
		bool trackClockwise;
	  MowState mowState;
		MowState lastMowState;
	  MowPattern mowPattern;
		Side obstacleSide;
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
		void sensorTriggered(uint16_t sensorID);
		void resetSensorTriggers();
  protected:    
    int lastPerimeterMag;
	  unsigned long trackLineTimeout;
    unsigned long nextInfoTime;
    unsigned long nextIMUTime; 
	  unsigned long nextControlTime;     
	  void stateMachine();
	  void track();
	  void mowLanes();	    
		void mowRandom();	    
    void printSensorData();
    void readRobotMessages();    		
};    

extern RobotClass Robot;

#endif

