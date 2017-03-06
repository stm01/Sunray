#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;

enum MotorMotion {MOT_PWM, MOT_LINE_TIME, MOT_LINE_DISTANCE, MOT_ROTATE_TIME, MOT_ROTATE_ANGLE, MOT_STOP, MOT_CAL_RAMP, MOT_ANGLE_DISTANCE } ;
typedef enum MotorMotion MotorMotion;


// PWM speed:
//    0..255  forward
//   -1..-255 reverse

class MotorClass {
  public:
    float deltaControlTimeSec;
    PID motorLeftPID;
    PID motorRightPID;
    PID imuPID;
    MotorMotion motion;

    bool lowPass;
    bool isStucked;
    bool paused;
    int  pwmMax;
    int  pwmMaxMow;
    int  rpmMax;    
    int motorLeftPWMSet;
    int motorRightPWMSet;    

    int motorLeftTicks;
    int motorLeftTicksZero;
    int motorRightTicks;
    int motorRightTicksZero;
    float ticksPerCm;
    int ticksPerRevolution;
    float wheelBaseCm;

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

    float motorLeftPWMCurr;
    float motorRightPWMCurr;

    float motorLeftRpmCurr;
    float motorRightRpmCurr;

    bool motorLeftSwapDir;
    bool motorRightSwapDir;

    float motorLeftEff;
    float motorRightEff;

    float motorLeftSense;
    float motorRightSense;
    float motorMowSense;
    float motorSenseMax;
    float mowSenseMax;
    
    void begin();
    void run();
    // rpm: 1.0 = max
    void travelAngleDistance(int distanceCm, float angleRad, float speedRpmPerc);
    void travelLineDistance(int distanceCm, float angleRad, float speedRpmPerc);
    void travelLineTime(int durationMS, float angleRad, float speedRpmPerc);
    void rotateTime(int durationMS, float speedRpmPerc);
    void rotateAngle(float angleRad, float speedRpmPerc);    
    // pwm/rpm: 1.0 = max
    void setSpeedPWM(float leftPWMPerc, float rightPWMPerc);
    void stopImmediately();
    void stopMowerImmediately();
  	void stopSlowly();
    void calibrateRamp();
    void setPaused(bool flag);
	  void setIsStucked();
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

