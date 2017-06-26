// Ardumower 'Sunray' 0.2 - PC remote control (to be run in 'Processing 3') 

import processing.serial.*;
import processing.net.*;
//import processing.video.*;

// configuration
String comPort = "COM5";
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