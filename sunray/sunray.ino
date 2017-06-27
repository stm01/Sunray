/* Ardumower software "Sunray" (v0.2 experimental)
   features:
   * mapping & localization
 
   required hardware: 
   * Ardumower PCB 1.3: Arduino Due, MC33926 motor drivers, two perimeter coils (left+right)
   * Ardumower motors with odometry
   * perimeter wire (Perimeter sender v2)
   * IMU: GY-88  
*/

#include "robot.h"


void setup(){
  Robot.begin();
} 

void loop(){  
  Robot.run();
}



