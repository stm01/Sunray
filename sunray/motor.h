#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;

// type of robot motion
enum MotorMotion {MOT_PWM, MOT_LINE_TIME, MOT_LINE_DISTANCE, MOT_ROTATE_TIME, MOT_ROTATE_ANGLE, MOT_STOP, MOT_CAL_RAMP, MOT_ANGLE_DISTANCE } ;
typedef enum MotorMotion MotorMotion;


/* motor controller - controls motion of robot via simple commands (travel on line and angle, rotate to angle)

   example usage:  
	 
	 Motor.begin();	 
	 Motor.travelLineDistance(distanceCm, absoluteAngle, speedPercent);
	 while (true){}
	   Motor.run();		 
	 }	

*/

// PWM speed:
//    0..255  forward
//   -1..-255 reverse

class MotorClass {
  public:
    float deltaControlTimeSec;
    PID motorLeftPID;
    PID motorRightPID;
    PID imuPID;
    MotorMotion motion;  // curren robot motion

    bool lowPass;
    bool isStucked;
    bool paused;      // is paused?
    int  pwmMax;
    int  pwmMaxMow;
    int  rpmMax;    
    int motorLeftPWMSet;
    int motorRightPWMSet;    

    int motorLeftTicks; // left motor odometry ticks 
    int motorLeftTicksZero;
    int motorRightTicks;
    int motorRightTicksZero;
    float ticksPerCm;  // ticks per cm
    int ticksPerRevolution; // ticks per revolution
    float wheelBaseCm;  // wheel-to-wheel diameter

    float distToLine;
    float motorPosX;
    float motorPosY;
    float speedDpsCurr;
    float distanceCmCurr;
	  float distanceCmAvg;
    int distanceCmSet;
    float angleRadCurr;
    float angleRadSet;    
    float angleRadSetStartX;
    float angleRadSetStartY;
    float speedRpmSet;
    float mowerPWMSet;
    float mowerPWMCurr;
    int speedDpsSet;

    float motorLeftPWMCurr;  // left motor pwm
    float motorRightPWMCurr; // right motor pwm

    float motorLeftRpmCurr;  // left motor rpm
    float motorRightRpmCurr; // right motor rpm

    bool motorLeftSwapDir;
    bool motorRightSwapDir;

    float motorLeftEff;
    float motorRightEff;

    float motorLeftSense; // left motor current 
    float motorRightSense; // right  motor current 
    float motorMowSense;  // mower motor current 
    float motorSenseMax;
    float mowSenseMax;
    
    void begin();
    void run();
    // rpm: 1.0 = max
		/* travel for a certain distance and steer to given relative angle */
    void travelAngleDistance(int distanceCm, float angleRad, float speedRpmPerc);
		/* travel for a certain distance and keep robot on line given by start point and angle */
    void travelLineDistance(int distanceCm, float angleRad, float speedRpmPerc);
    /* travel for a certain time and and keep robot on line given by start point and angle */
		void travelLineTime(int durationMS, float angleRad, float speedRpmPerc);
		/* rotate (pos/neg direction given by pos/neg speed) for a certain time */
    void rotateTime(int durationMS, float speedRpmPerc);
		/* rotate to certain absolute angle */
    void rotateAngle(float angleRad, float speedRpmPerc);    
    // pwm/rpm: 1.0 = max
		/* manually control left/right motors by given speed in percent */
    void setSpeedPWM(float leftPWMPerc, float rightPWMPerc);
    /* stop immediately (brake) */
		void stopImmediately();
		/* stop mower motor immediately */
    void stopMowerImmediately();
		/* stop slowly (do not brake) */ 
  	void stopSlowly();
    void calibrateRamp();
		/* pause current motion (robot will stop and continue with action after unpause) */
    void setPaused(bool flag);
		/* not used */
	  void setIsStucked();
		/* set mower motor speed in percent */
    void setMowerPWM(float pwmPerc);
  protected:     
    unsigned long lastControlTime;	
	  unsigned long motorStopTime;		
    unsigned long overCurrentTimeout;
    void speedControl();
    void speedControlLine();
    void speedControlAngle();
    void speedControlRotateAngle();
    void speedPWM( MotorSelect motor, int speedPWM );    
	  void setMC33926(int pinDir, int pinPWM, int speed);    
    void checkFault();
    void resetFault();    
};

extern MotorClass Motor;

#endif

