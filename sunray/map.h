/*

The robot mower uses odometry, an IMU (gyro+compass) and a perimeter to localize itself on the perimeter 
and within the perimeter. A particle filter estimates the robot position based on moved distance 
and absolute angle.

map and particle filter  (modelled after this course: https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373 )

example usage:
  1) start:                  Map.begin();
  2) start mapping:          Map.clearOutline();
                             Robot.state = STAT_CREATE_MAP;
  3) stop mapping:           Robot.state = STAT_MOW
                             Map.correctOutline();
                             Map.transferOutlineToMap();
                             Map.distributeParticlesOutline();
  4) call this in main loop: Map.run();

*/

#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include "robot.h"

#define MAP_SIZE_X 20
#define MAP_SIZE_Y 20

// outline OR particles count
#define OUTLINE_PARTICLES 150 


// map data
struct map_data_struc {
  unsigned char signal    : 5; // lowest bits
  unsigned char state     : 2; 
  unsigned char side      : 1; // highest bits
};

union map_data_t {
  uint8_t v;
  struct  map_data_struc s;
};

typedef union map_data_t map_data_t;


#define MAP_DATA_SIGNAL_MAX 31

#define MAP_DATA_STATE_OBSTACLE 2
#define MAP_DATA_STATE_MOWED    1
#define MAP_DATA_STATE_UNMOWED  0

#define MAP_DATA_SIDE_OUT       1
#define MAP_DATA_SIDE_IN        0


struct point_t {
  float x;
  float y;
};

typedef struct point_t point_t;


struct polar_t {
  float r;
  float phi;
};

typedef struct polar_t polar_t;


struct robot_state_t {
  float x;
  float y;
//  float orientation;
};

typedef struct robot_state_t robot_state_t;



class MapClass {
    public:
		  bool verboseOutput;
      bool mapValid;
      robot_state_t robotState; // current robot position estimation (meter)
      float steeringNoise; // robot steering noise sigma (rad units)
      float distanceNoise;  // distance sensor measurement noise sigma (m)
      float measurementNoise; // perimeter sensor measurement noise sigma (magnitude)
      float overallProb; // overall probability
      robot_state_t particlesState; // all particles center (meter)
      float particlesDistanceX; // all particles diameter (meter)
      float particlesDistanceY;
      robot_state_t outlineParticles[OUTLINE_PARTICLES]; // outline OR particles data
      float mapScaleX; // meter to pixel
      float mapScaleY;
	    int perimeterWireLengthMeter;
      float currMapX;
      float currMapY;
      int lastMotorLeftTicks;
      int lastMotorRightTicks;
  	  int perimeterOutlineSize;
      map_data_t mapData[MAP_SIZE_Y][MAP_SIZE_X];
      void begin();
      void run();
      void clearOutline();
      void exampleOutline();
      void correctOutline();
      void transferOutlineToMap();
      float distanceToStart(float x, float y);
      float distanceToParticles(float x, float y);
      void robotMotion(float course, float distance);
      void particlesMotion(float course, float distance);
      inline bool isXYOnMapMeter(float x, float y);
      bool isXYOnMap(int x, int y);
      void setMapData(int xp, int yp, map_data_t value);
      void setMapDataMeter(float x, float y, map_data_t value, int thickness = 1);
      inline map_data_t getMapDataMeter(float x, float y);
      void distributeParticlesOutline();
      void computeParticlesState();
      void setParticlesState(float x, float y, float orientation);
      inline float measurementProb(int particleIdx, float leftMag, float rightMag);
      void sense(float leftMag, float rightMag);
      void resetMapDataOutside(int x, int y);
      void resetMapDataInside(int x, int y);
      void resetMapDataSignal(int x, int y, int signalStrength);	  
  	  boolean loadMap();
	    void saveMap();      
    protected:
	    float distAvgSum;  
	    void loadSaveMap(boolean readflag);
      unsigned long nextSampleTime;
};

extern MapClass Map;


#endif

