/* Ardumower software "Sunray" (v0.2 experimental)
   features:
   * mapping & localization
 
   required hardware: 
   * Ardumower PCB: Arduino Due, MC33926 motor drivers+protector board, two perimeter coils (left+right)
   * Ardumower motors with odometry
   * perimeter wire (Perimeter sender v2)
   * replaced IMU: GY-88  (requires to remove RTC module to work properly)
*/

#include "robot.h"


void setup(){
  Robot.begin();
} 

void loop(){  
  Robot.run();
}



