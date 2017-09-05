/*
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

#include "server.h"
#include "config.h"
#include "motor.h"
#include "buzzer.h"
#include "map.h"
#include "battery.h"
#include "robot.h"
#include "pinman.h"
#include "RunningMedian.h"
#include "helper.h"
#include <Arduino.h>

ServerClass Server;

void ServerClass::begin(){
	hdrLen=0;
}

void ServerClass::sendHTTPNotFound(){
  content = "HTTP/1.0 404 \r\n";
  content += "\r\n";
}

void ServerClass::initHTTP(){
  content = "HTTP/1.1 200 OK\r\n";
  //content += "Access-Control-Allow-Origin: *\r\n";
  //content += "Content-Type: text/html\r\n";
  content += "Connection: keep-alive\r\n";
  //content += "Connection: close\r\n";
  content += "Content-Length: #L#\r\n";
  content += "\r\n";
  hdrLen = content.length();
}

void ServerClass::finishHTTP(){
  content.replace("#L#", String( content.length() - hdrLen) );
}

void ServerClass::sendInfo(){
  initHTTP();
  content += "cb(['INFO',";
  content += millis();  
  content += ",";
  content += Robot.state;  
  content += ",";  
  content += Robot.loopsPerSec;
  content += ",";
  content += Map.robotState.x;
  content += ",";
  content += Map.robotState.y;  
  content += ",";
  content += IMU.getYaw();
  content += ",";
  content += IMU.ypr.pitch;     
  content += ",";
  content += IMU.ypr.roll;    
  content += ",";
  content += IMU.comYaw;    
  content += ",";
  content += ",";
  content += IMU.gyro.z;    
  content += ",";  
  content += Perimeter.getMagnitude(IDX_LEFT);
  content += ",";
  content += Perimeter.getMagnitude(IDX_RIGHT);
  content += ",";
  content += Motor.motorLeftPWMCurr;
  content += ",";
  content += Motor.motorRightPWMCurr;
  content += ",";  
  content += Motor.motorLeftTicks;
  content += ",";  
  content += Motor.motorRightTicks;
  content += ",";
  content += Motor.motorLeftRpmCurr;
  content += ",";
  content += Motor.motorRightRpmCurr;
  content += ",";     
  content += IMU.com.x;
  content += ",";     
  content += IMU.com.y;
  content += ",";     
  content += IMU.com.z;  
  content += ",";
  content += IMU.acc.x;
  content += ",";     
  content += IMU.acc.y;
  content += ",";     
  content += IMU.acc.z;  
  content += ",";     
  content += Motor.motorLeftSense;
  content += ",";
  content += Motor.motorRightSense;    
  content += ",";
  content += Motor.motorMowSense;
  content += ",";
  content += Motor.motorLeftFriction;
  content += ",";
  content += Motor.motorRightFriction;
  content += ",";
  content += Map.overallProb;
  content += ",";
  content += Battery.batteryVoltage;                   
  content += ",";
  content += Motor.motion;   
  content += ",";
  content += IMU.state;       
  content += ",";
  content += Motor.distanceCmSet;   
  content += ",";
  content += Motor.angleRadSet;  
  content += "]);";  
  finishHTTP();
}


void ServerClass::serveHTTP(){
  if (content.length() > 0) {
    int cnt = min(1500, (int)content.length());
    ROBOTMSG.write(content.c_str(), cnt);
    content.remove(0, cnt);      
  }
    
  while (ROBOTMSG.available()){
    char ch = ROBOTMSG.read();
    if (ch != 0) receivedData += ch;    
  }
  if (receivedData.length() == 0) return;

  if (content.length() > 0) return;
  
  int getidx = receivedData.indexOf("GET /");
  if (getidx != -1) {
    // HTTP request
    receivedData.remove(0, getidx);
    int endidx = receivedData.indexOf("\r\n\r\n");    
    if (endidx == -1) {
      return;
    }
    int idx;
    if ( (idx = receivedData.indexOf("favicon.ico ")) != -1) {
      sendHTTPNotFound();
    } else if ( (idx = receivedData.indexOf("cmd=led")) != -1) {      
      // LED request      
      //state = !state;
      sendInfo();
    } else if ( (idx = receivedData.indexOf("cmd=mow_lanes")) != -1) {
      Robot.startLaneMowing();
      sendInfo();
    } else if ( (idx = receivedData.indexOf("cmd=stop")) != -1) {
      Motor.stopImmediately();
      Robot.state = STAT_IDLE;
      sendInfo();     
    } else if ( (idx = receivedData.indexOf("cmd=pwm")) != -1) {
      int leftpwm = parseFloatValue(receivedData, "l");
      int rightpwm = parseFloatValue(receivedData, "r");
      Motor.setSpeedPWM(((float)leftpwm)/100, ((float)rightpwm)/100);      
      sendInfo();
    } else {
      // default site request
      sendInfo();
    }
    receivedData.remove(0, endidx+4);
  }  
}


