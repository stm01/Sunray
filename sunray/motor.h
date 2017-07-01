// MC33926 motor controller
// integrated speed, angle and line controller (PID)

// motor controller - controls motion of robot via simple commands (travel on line and angle, rotate to angle)

// example usage:  
	 
//	 Motor.begin();	 
//	 Motor.travelLineDistance(distanceCm, absoluteAngle, speedPercent);
//	 while ( Motor.motion != MOT_STOP ){
//		 if (obstacle) Motor.stopImmediately();
//	   Motor.run();		 
//	 }	



#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;

// type of robot motion
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
		float imuAnglePID_Kp;
		float imuAnglePID_Ki;
		float imuAnglePID_Kd;
		float imuPID_Kp;
		float imuPID_Ki;
		float imuPID_Kd;
    MotorMotion motion;  // curren robot motion

		bool verboseOutput; 
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
		int wheelDiameter; // wheel diameter mm
    int ticksPerRevolution; // ticks per revolution
    float wheelBaseCm;  // wheel-to-wheel diameter

    float distToLine;
    float motorPosX;
    float motorPosY;
    float speedDpsCurr;
    float distanceCmCurr;
	  float distanceCmAvg;
    int distanceCmSet;  // set line distance (cm)
    float angleRadCurr; // current (absolute) robot angle
		float angleRadCurrDeltaIMU; // robot angle delta IMU
		float angleRadCurrDeltaOdometry; // robot angle delta odometry		
    float angleRadSet;  // set (absolute) start line angle
    float angleRadSetStartX;
    float angleRadSetStartY;
    float speedRpmSet;
    float mowerPWMSet;
    float mowerPWMCurr; // current mower motor pwm
    int speedDpsSet;

    float motorLeftPWMCurr;  // current left motor pwm
    float motorRightPWMCurr; // current right motor pwm

    float motorLeftRpmCurr;  // current left motor rpm
    float motorRightRpmCurr; // current right motor rpm		
		float motorLeftRpmLast;
		float motorRightRpmLast;
		float motorLeftRpmAcceleration;  // left motor acceleration
		float motorRightRpmAcceleration;  // right motor acceleration

    bool motorLeftSwapDir;
    bool motorRightSwapDir;
		
    float robotMass;
		float motorLeftFriction; // wheel friction
    float motorRightFriction;

    float motorLeftSense; // left motor sense 
    float motorRightSense; // right  motor sense 
    float motorMowSense;  // mower motor sense
    float motorFrictionMin; // min. allowed gear motor friction
		float motorFrictionMax; // max. allowed gear motor friction
    float mowSenseMax;    // max. allowed mower sense
		
		float diffOdoIMU;
		float stuckMaxIMUerror;
		float stuckMaxDiffOdometryIMU;
    
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
		void resetPID();
};

extern MotorClass Motor;

#endif

