// compass calibration & visualization
// for Ardumower Sunray
// requires: 
//  -Processing 3.0
//  -Bluetooth connection (adjust COM port below)
//  
// 1. start robot
// 2. run this app
// 3. rotate robot until ellipsoid is filled with 10,000 points - calibration is done!

import processing.opengl.*;
import processing.serial.*;
// import processing.sound.*;  // requires Processing Sound Library (menu Tools->Add tools->Library->'Sound library for Processing')
import java.awt.event.*;

// -------- configuration --------
boolean useFileInput = true;     // use file as compass data input
boolean useFileOutput = false;    // save received compass data to file

boolean useSerialInput = false;   // use serial port as compass data input
boolean useSerialOutput = false;   // use serial port for sending compass calibration data (true/false) 
String serialport = "COM14";
//String filename = "data/magpoints.txt";   // file for data input/output
String filename = "data/magpoints_gy88_indoor.txt";
// -------------------------------

// calibration mode  
static final int CAL_FIT_ELLIPSOID = 0;
static final int CAL_FIT_ELLIPSOID_ROTATED = 1;
static final int CAL_MIN_MAX = 2;  

int serialbaud = 115200;
//int calibMode = CAL_MIN_MAX; 
int calibMode = CAL_FIT_ELLIPSOID_ROTATED;
boolean useTwoPhaseOutlierRemoval = true;
float outlierThreshold = 0.08;
int totcount = 20000;
//SoundFile sound;
PImage img;
Serial port = null;
int nextSolveTime = 0;
int nextStatTime = 0;
int nextSaveTime = millis() + 10000;
int lastSolveCount = 0;
int lastSaveCount = 0;

int packetsReceived =0;
float zoom = 0.5;
boolean useRaw = true;  

PFont font;
int count = 0;
boolean sendVerbose = true; 

String line;
PVector currPoint = new PVector(0,0,0);
PVector[] vecs = new PVector[totcount];
boolean[] outlierFlags = new boolean[totcount];

float xc, yc, zc;
float x_max = 0;
float x_min = 0;
float y_max = 0; 
float y_min = 0; 
float z_max = 0;
float z_min = 0;
float x_length = 0;
float y_length = 0;
float z_length = 0;
float length_mean = 0;
float r_mean = 0;
float x_gain = 1, y_gain = 1, z_gain = 1;
float spheredim = 100;
float std = 0;
int outliers = 0;
//float[] centre = {0, 0, 0};
//float[] radii = {100, 100, 100};
Matrix B = new Matrix(3, 1);
Matrix A_1 = new Matrix(3, 3);
    

void writeData() {
  println("writeData " + count);
  PrintWriter output = createWriter(filename); 
  for (int i = 0; i < count; i++) {
    PVector v = vecs[i];
    output.println(v.x + "," + v.y + "," + v.z);
  }
  output.flush();
  output.close();
}

void setup() {
  img = loadImage("calibration.png");
  //sound = new SoundFile(this, "clockalarm.mp3");  
  size(1000, 700, OPENGL);
  //println("setup");
  frame.addMouseWheelListener(new MouseWheelInput());
  font = createFont("ArialMT", 32, true);
  if ((useSerialInput) || (useSerialOutput)) {
    println("using serial port "+serialport + " " + serialbaud);
    port = new Serial(this, serialport, serialbaud); 
    port.bufferUntil('\n'); 
    delay(2000);      
    port.clear();
    //port.buffer(60);
  }
  if (useFileInput) {
    readData();    
    autoSolve();    
  };
}

PVector getPoint(int idx) {  
  PVector v;
  if (idx != -1) v = vecs[idx];
    else v = currPoint;          
  PVector res = new PVector();
  if (useRaw) {
    res.x = v.x;
    res.y = v.y;
    res.z = v.z;
  } else {
    //res.x = (v.x - (float)centre[0]) / ((float)radii[0])*100.0;
    //res.y = (v.y - (float)centre[1]) / ((float)radii[1])*100.0;
    //res.z = (v.z - (float)centre[2]) / ((float)radii[2])*100.0;
    
    // calA = A_1 * (rawA - B)
    Matrix A = new Matrix(3,1);
    A.set(0, 0, v.x);
    A.set(1, 0, v.y);
    A.set(2, 0, v.z);
    A = A_1.times(A.minus(B));                  
    res.x = ((float)A.get(0, 0))/10.0;
    res.y = ((float)A.get(1, 0))/10.0;
    res.z = ((float)A.get(2, 0))/10.0;
  }
  return res;
}  

/*
main loop draw
 */
void draw() {
  PVector v;
  //readfromserial();
  if ((sendVerbose) && (millis() >= 2000)) {
    if (useSerialInput) {
      println("sending verbose mode to serial port");
      port.write("?73\n");
      sendVerbose = false;
    }
  }

  background(0);  
  translate(width/2, height/2);
  if (useRaw) image(img, width/2-img.width, height/2-img.height);

  //print text
  textFont(font, 20);
  fill(255, 255, 255);  
  text("keys: +/- zoom, s solve, v toggle verbose", -width/2 + 20, height/2 - 20);  
  float mag = 0;
  if (useSerialInput){
    v = getPoint(-1);
    mag = sqrt(sq(v.x)+sq(v.y)+sq(v.z));
  }
  text("std: " + std + "  r_mean: " + r_mean + "  mag: " + mag, -width/2 + 20, height/2 - 40);
  text("min: " + x_min + "," + y_min + "," + z_min, -width/2 + 20, height/2 - 80);
  text("max: " + x_max + "," + y_max + "," + z_max, -width/2 + 20, height/2 - 60);    
  //"  Gain: " + x_gain + ", " + y_gain + ", " + z_gain, -width/2 + 20, height/2 - 20);
  text("point count: " + count + " / " + totcount + " outliers: "+outliers, -width/2 + 20, height/2 - 100);
  text("data pkts: " + packetsReceived, -width/2 + 20, height/2 - 120);
  String s = "compass input: ";
  if (useSerialInput) s += serialport + "," + serialbaud;
    else s += filename;
  s += ", calib output: ";
  if (useSerialOutput) s+= serialport;
    else s+= "none";
  text(s, -width/2+ 20, height/2-140);
  if (useRaw) s = "please rotate robot so its nose is pointing in all directions in 3d space... (" 
    + ((int)(((float)count) / ((float)totcount) * 100)) + "% complete)";  
    else s = "calibration done (" + float2String( 100.0-std) + "% quality)";  
  text(s, -width/2 + 20, -height/2+40);

  //rotate plan
  float z = radians(0.7 * mouseX);
  float r = -0.007 * (mouseY - 400);
  rotateY(z);
  rotateX(r * cos(z));
  rotateZ(r * sin(z));

  //scale zoom
  scale(zoom, -zoom, zoom);
    
  float sc = x_length/500;
  sc = constrain(sc, 1, 10);

  //print box
  noFill();
  stroke(100);
  strokeWeight(1*sc);
  box(1200);

  //print axis-X
  stroke(255, 0, 0);
  strokeWeight(5*sc);
  line (300, 0, 0, 275, 0, 25);
  line (300, 0, 0, 275, 0, -25); 
  line (320, -20, 0, 340, 20, 0);
  line (320, 20, 0, 340, -20, 0);
  line(-300, 0, 0, 300, 0, 0);

  //print axis-Y
  stroke(0, 0, 255);
  strokeWeight(5*sc);
  line (0, 0, 300, 25, 0, 275);
  line (0, 0, 300, -25, 0, 275); 
  line (0, 20, 320, 0, 0, 330);
  line (0, 0, 330, 0, 20, 340);
  line (0, 0, 330, 0, -20, 330);
  line(0, 0, -300, 0, 0, 300);

  //print axis-Z
  stroke(0, 255, 0);
  strokeWeight(5*sc);
  line (0, 300, 0, 25, 275, 0);
  line (0, 300, 0, -25, 275, 0);
  line (-10, 350, 0, 10, 350, 0);
  line (10, 350, 0, -10, 320, 0); 
  line (-10, 320, 0, 10, 320, 0); 
  line(0, -300, 0, 0, 300, 0);

  //print sphere
  noFill();
  stroke(255, 30);
  sphere(spheredim);
  
  //print points
  for (int i = 0; i < count; i++) {
    v = getPoint(i);    
    if (outlierFlags[i]) {
      stroke(255, 0, 0);
      strokeWeight(5*sc);
    } else {
      stroke(255);
      strokeWeight(3*sc);
    }       
    point(v.x, v.z, v.y);
  }
  // current point
  if (useSerialInput){  
    v = getPoint(-1);       
    stroke(255, 255, 0);
    strokeWeight(20*sc);
    point(v.x, v.z, v.y);
  }
  
  // center
  stroke(0, 255, 0);
  strokeWeight(20*sc);       
  point(xc, zc, yc);
}


void findOutliers(boolean findCenter, boolean findRadius) {
  xc = 0;
  yc = 0;
  zc = 0;
  if (findCenter){
    for (int i = 0; i < count; i++) {
      PVector v = getPoint(i); 
      xc += v.x/((float)count);
      yc += v.y/((float)count);
      zc += v.z/((float)count);    
    }
  }
  if (findRadius){
    r_mean = 0;  
    for (int i = 0; i < count; i++) {
      PVector v = getPoint(i);
      float r = sqrt(pow(v.x-xc, 2) + pow(v.y-yc, 2) + pow(v.z-zc, 2));        
      r_mean += r / ((float)count);
    }
  } else r_mean = 100; 
  outliers = 0;
  for (int i = 0; i < count; i++) {
    PVector v = getPoint(i);
    float r = sqrt(pow(v.x-xc, 2) + pow(v.y-yc, 2) + pow(v.z-zc, 2));
    if (abs(r-r_mean)/r_mean < outlierThreshold) {                 
      outlierFlags[i]=false;
    } else {
      outliers++;
      outlierFlags[i]=true;
    }
  }
  println("outliers:"+outliers);
}


void calcStats() {  
  //  if (useRaw) findOutliers(true, true);
  std = 0;
  x_max = -9999;
  x_min = 9999;
  y_max = -9999; 
  y_min = 9999; 
  z_max = -9999;
  z_min = 9999;
  for (int i = 0; i < count; i++) {
    PVector v = getPoint(i);    
    if (!outlierFlags[i]) { 
      x_min = min(x_min, v.x);
      x_max = max(x_max, v.x);
      y_min = min(y_min, v.y);
      y_max = max(y_max, v.y);
      z_min = min(z_min, v.z);
      z_max = max(z_max, v.z);
      std += abs(sqrt( v.x*v.x + v.y*v.y + v.z*v.z ) - 100.0);                 
    } 
  }
  std /= ((float)count-outliers);
  x_length = x_max - x_min;
  y_length = y_max - y_min;
  z_length = z_max - z_min;
  xc = x_min + x_length/2;
  yc = y_min + y_length/2;
  zc = z_min + z_length/2;    
  length_mean = (x_length + y_length + z_length) / 3;
  x_gain = x_length / length_mean;
  y_gain = y_length / length_mean;
  z_gain = z_length / length_mean;
  r_mean = 0;
  for (int i = 0; i < count; i++) {
    if (!outlierFlags[i]){
      PVector v = getPoint(i);
      float r = sqrt(pow(v.x-xc, 2) + pow(v.y-yc, 2) + pow(v.z-zc, 2));      
      //r_mean = 0.999 * r_mean + 0.001 * r; 
      r_mean += r / ((float)count);
    }
  }    
  //println("center: "+xc+","+yc+","+zc);
}


/*
mouse zoom
 */
class MouseWheelInput implements MouseWheelListener {
  void mouseWheelMoved(MouseWheelEvent e) {
    zoom -= 0.05 * e.getWheelRotation();
    zoom = constrain(zoom, 0.01, 5);
  }
}

void autoSave() {
  if (!useSerialInput) return;
  if (lastSaveCount == count) return;
  if (millis() >= nextSaveTime) {
    nextSaveTime = millis() + 10000;
    lastSaveCount = count;
    if (useFileOutput) writeData();
  }
}

void addPoint(float x, float y, float z) {  
  if (count >= totcount) {
    //sound.play();
    autoSolve();
    return;
  }
  vecs[count] = new PVector(x, y, z);  
  count++;
}


void keyPressed() { 
  if (key == 'v') {
    //port.write("v\n");    
    println("sending verbose mode to serial port");
    port.write("?73\n");
  }
  if (key == '+') {        
    zoom += 0.1;
    println("zoom "+zoom);
  } 
  if (key == '-') {            
    zoom -= 0.1;
    println("zoom "+zoom);
  } 
  if (key == 's') {
    solve();
  }
  zoom = constrain(zoom, 0.01, 5);
}

void mousePressed() {
  if (useRaw) {    
    solve();    
  } else {
    useRaw = true;
    calcStats();    
  }
  zoom = 400/x_length;
  zoom = constrain(zoom, 0.01, 5);
}

void serialEvent(Serial port) {    
  try {
    String data = port.readStringUntil('\n');  
    if (data != null) {    
      data = trim(data);                // trim off whitespaces.
      packetsReceived++;    
      /*if ( (verboseOutput) || (!data.startsWith("!")) ) {
       inString = data;
       println(">"+inString);  
       }*/
      //println(inString);
      if ( (useSerialInput) && (data.startsWith("!04")) ) {
        // IMU data      
        String[] list = splitTokens(data, ",");
        if (list.length >= 11) {
          float comX = Float.parseFloat(list[1]);
          float comY = Float.parseFloat(list[2]);
          float comZ = Float.parseFloat(list[3]);
          currPoint.x=comX;
          currPoint.y=comY;
          currPoint.z=comZ;          
          addPoint(comX, comY, comZ);
          if (millis() >= nextStatTime) {
            nextStatTime = millis() + 1000;
            calcStats();
          }
          autoSave();
          //autoSolve();          
        }
      }
    }
  } 
  catch (Exception e) {
    println("EX: "+e);
  }
} 

void autoSolve() {
  if (lastSolveCount == count) return;
  if (millis() >= nextSolveTime) {
    nextSolveTime = millis() + 10000;
    lastSolveCount = count;
    solve();    
  }
}

void readData() {    
  println("readData");
  BufferedReader reader = createReader(filename);
  while (true) {
    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }
    if (line == null) {
      // Stop reading because of an error or file is empty      
      calcStats();
      return;
    } else {
      String[] pieces = split(line, ",");
      if (pieces == null) return;
      if (pieces.length == 3) {    
        float x = float(pieces[0]);
        float y = float(pieces[1]);
        float z = float(pieces[2]);
        addPoint(x, y, z);
      }
    }
  }
}


String float2String(float number) {
  String s = String.format("%.2f", number);
  s = s.replaceAll(",", ".");
  return s;
}

public void printMatrix(Matrix a) {
  for (int i=0; i < a.getRowDimension(); i++) {
    for (int j=0; j < a.getColumnDimension(); j++) {
      print(float2String((float)a.get(i, j)) + "  ");
    }
    println();
  }
}


void solve() {
  useRaw = true;  
  fit();
  useRaw = false;
  if (useTwoPhaseOutlierRemoval){   
    findOutliers(false, false);
    useRaw = true;
    fit();
  }
  println("A_1=");
  printMatrix(A_1);
  println("B=");
  printMatrix(B);
  // calA = A_1 * (rawA - B)
  // vector rawA: your raw measurement [x y z]
  //println("centre: "+float2String(centre[0]) + "," + float2String(centre[1]) + "," +float2String(centre[2]) );
  //println("radii: "+float2String(radii[0]) + "," + float2String(radii[1]) + "," + float2String(radii[2]) );
  useRaw = false;
  calcStats();
  if (useSerialOutput) {
    println("sending calibration data to serial port");
    //port.write("?78," + float2String(centre[0]) + "," + float2String(centre[1]) + "," +float2String(centre[2])+ ","  
    //  + float2String(radii[0]) + "," + float2String(radii[1]) + "," + float2String(radii[2]) + "\n");
    port.write("?78," + float2String((float)A_1.get(0,0)) + "," + float2String((float)A_1.get(0,1)) + "," + float2String((float)A_1.get(0,2)) + ","
                      + float2String((float)A_1.get(1,0)) + "," + float2String((float)A_1.get(1,1)) + "," + float2String((float)A_1.get(1,2)) + ","
                      + float2String((float)A_1.get(2,0)) + "," + float2String((float)A_1.get(2,1)) + "," + float2String((float)A_1.get(2,2)) + ","
                      + float2String((float)B.get(0,0))   + "," + float2String((float)B.get(1,0))   + "," + float2String((float)B.get(2,0)) +  "\n");
  }  
}
  
  
void fit(){  
  double[][] coord = new double[count-outliers][3];
  ArrayList<double[]> coordList = new ArrayList<double[]>();
  int idx=0;
  for (int i=0; i < count; i++) {
    PVector v = getPoint(i);    
    if (!outlierFlags[i]) { 
      coord[idx][0]=v.x;
      coord[idx][1]=v.y;
      coord[idx][2]=v.z;
      double[] pt = new double[3];
      pt[0] = v.x;
      pt[1] = v.y;
      pt[2] = v.z;
      coordList.add(pt);                        
      idx++;
    }
  }
  println("FitEllipsoid start");
  try {
    // calA = A_1 * (rawA - B)
    if (calibMode == CAL_FIT_ELLIPSOID_ROTATED){
      CompassCalibration cal = new CompassCalibration();
      cal.calculate(coordList);
      B = cal.getMatrixB();
      A_1 = cal.getMatrixA_1();
    } 
    else if (calibMode == CAL_FIT_ELLIPSOID) {    
      Object[] res = FitEllipsoid.yuryPetrov(coord);  
      double[] c = ((double[])res[0]);
      double[] r = ((double[])res[1]);
      B.set(0,0, ((float)c[0]));
      B.set(1,0, ((float)c[1]));
      B.set(2,0, ((float)c[2]));
      A_1.set(0,0, 1.0/((float)r[0])*1000.0);
      A_1.set(1,1, 1.0/((float)r[1])*1000.0);
      A_1.set(2,2, 1.0/((float)r[2])*1000.0);      
    } 
    else if (calibMode == CAL_MIN_MAX){
      B.set(0,0, (x_max+x_min)/2 );
      B.set(1,0, (y_max+y_min)/2 );
      B.set(2,0, (z_max+z_min)/2 );
      A_1.set(0,0, 1.0/(x_length/2) *1000.0);
      A_1.set(1,1, 1.0/(y_length/2) *1000.0);
      A_1.set(2,2, 1.0/(z_length/2) *1000.0);          
    }
  } 
  catch (Exception e) {
    println("EX: "+e);
  }    
  println("FitEllipsoid end");  
}