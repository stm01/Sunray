/* Ardumower software "Sunray" (v0.2 experimental)
   features:
   * mapping & localization
 
   required hardware: 
   * Ardumower PCB 1.3: Arduino Due, MC33926 motor drivers, two perimeter coils (left+right)
   * Ardumower motors with odometry
   * perimeter wire (Perimeter sender v2)
   * IMU: GY-88  
	 
License
Copyright (c) 2013-2017 by Alexander Grau

Private-use only! (you need to ask for a commercial-use)

The code is open: you can modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

The code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
  
	 
*/

#include "robot.h"

void watchdogSetup(){}
void setup(){
  Robot.begin();
} 

void loop(){  
  Robot.run();
}
