// Ardumower 'Sunray' 0.2 - PC remote control (to be run in 'Processing 3') 

import processing.serial.*;
import processing.net.*;
//import processing.video.*;

// configuration
String comPort = "COM9";
boolean useTcp = false;
String tcpHost = "raspberrypi.local";
int tcpPort = 8083;
boolean useSatMap = false;
double centerLat = 52.267312;    
double centerLon = 8.609331;
boolean demo = true;


Robot robot = new Robot();
PImage satImg;
double meterPerPixel=0;
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
  if (useSatMap) loadSatImage(centerLat, centerLon);    
}

/*
void movieEvent(Movie m) {
  m.read();
}
*/

void draw () {
  robot.draw();   
  tint(255, 127);  // Display at half opacity
  if (satImg != null) image(satImg, 0, 0);
  //image(mov, 0, 0);
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



void loadSatImage(double lat, double lon){
  String satFileName = sketchPath() + "\\map_" + lat + "_" + lon + ".png";
  println(satFileName);
  File f = new File(satFileName);
  int zoom = 20;
  meterPerPixel = (Math.cos(lat * Math.PI/180) * 2 * Math.PI * 6378137) / (256 * Math.pow(2, zoom));
  print("meterPerPixel="+meterPerPixel);
  if (!f.exists()){ 
    String url = "http://maps.google.com/maps/api/staticmap?center=";    
    url += lat + "," + lon;
    url += "&zoom=" + zoom + "&size=640x640&maptype=satellite&sensor=false";
    println(url);
    satImg = loadImage(url, "png");
    satImg.save(satFileName);
  } else {
    satImg = loadImage(satFileName, "png");
  }
}


/*void movieEvent(Movie m) {
  m.read();
}*/