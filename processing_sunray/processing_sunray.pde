// Ardumower 'Sunray' 0.2 - PC remote control (to be run in 'Processing 3') 

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


import processing.serial.*;
import processing.net.*;
//import processing.video.*;

// configuration
String comPort = "COM5"; // COM5 / COM7
boolean useTcp = false;
String tcpHost = "raspberrypi.local";
String logFile = ""; //"outdoor_mow_rand.log";  // if file exists, playback mode, otherwise record mode
int logPlaySpeed = 1; // 1..100
int logPlayIntervalMillis = 10; // 1..10
int tcpPort = 8083;
boolean useSatMap = false;
double centerLat = 52.267312;    
double centerLon = 8.609331;
boolean demo = false;
double meterPerPixel=0;

Robot robot = new Robot();
//Movie mov;


void setup () {
  // set the window size:
  //size(1200, 600);
  size(1200, 700, P3D);  
  robot.setup(this);
  //String url =  "http://raspberrypi.local:8081";
  //img = loadImage(url, "jpg");
  //mov = new Movie(this, url);  
  //mov.play();    
}

/*
void movieEvent(Movie m) {
  m.read();
}
*/

void draw () {
  robot.draw();   
}

void serialEvent (Serial myPort) {
  robot.serialEvent(myPort);
}

void keyPressed() {
}

void mousePressed() {
  robot.mousePressed();
}

void mouseClicked(){
  robot.mouseClicked();
}

void mouseMoved(){
  robot.mouseMoved();
}

void mouseDragged(){
  robot.mouseDragged();
}

void mouseReleased(){
  robot.mouseReleased();
}




/*void movieEvent(Movie m) {
  m.read();
}*/