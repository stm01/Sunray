import java.io.*;


class Robot {
  

static final float manualSpeed = 0.2f;
boolean verboseOutput = false;
  
  
static final String EEPROM_FILENAME = "eeprom.bin";

String[] states = {"IDLE", "GYRO", "TRAK", "MAP", "MOW"};
static final int STAT_IDLE       = 0;
static final int STAT_CAL_GYRO   = 1;
static final int STAT_TRACK      = 2;
static final int STAT_CREATE_MAP = 3;
static final int STAT_MOW        = 4;

String[] trackStates = {"TRK_RUN", "TRK_FIND", "TRK_ROTATE"};
static final int TRK_RUN         = 0;
static final int TRK_FIND        = 1;
static final int TRK_ROTATE      = 2;

String[] mowStates = {"MOW_ROTATE", "MOW_REV", "MOW_LINE", "MOW_ENTER_LINE"};
static final int MOW_ROTATE      = 0;
static final int MOW_REV         = 1;
static final int MOW_LINE        = 2;
static final int MOW_ENTER_LINE  = 3;

String[] patterns = {"PATTERN_NONE", "PATTERN_RANDOM", "PATTERN_LANES"};
static final int PATTERN_NONE    = 0;
static final int PATTERN_RANDOM  = 1;
static final int PATTERN_LANES   = 2;

String[] imuStates = {"IMU_RUN", "IMU_CAL_GYRO", "IMU_CAL_COM"};
static final int IMU_RUN      = 0;
static final int IMU_CAL_GYRO = 1;
static final int IMU_CAL_COM  = 2;

String[] motions = {"MOT_PWM", "MOT_LINE_TIME", "MOT_LINE_DISTANCE", "MOT_ROTATE_TIME", "MOT_ROTATE_ANGLE", "MOT_STOP", "MOT_CAL_RAMP", "MOT_ANGLE_DISTANCE"};
static final int MOT_PWM            = 0;
static final int MOT_LINE_TIME      = 1;
static final int MOT_LINE_DISTANCE  = 2;
static final int MOT_ROTATE_TIME    = 3;
static final int MOT_ROTATE_ANGLE   = 4;
static final int MOT_STOP           = 5;
static final int MOT_CAL_RAMP       = 6;


//Variables
Map map = new Map();
HashMap<Integer, Integer> eeprom = new HashMap<Integer, Integer>();
String menu;
String menuResponse;
Serial mySerial;
Client myTcp;
String inString;
boolean mouseClicked = false;
boolean updateScreen = false;
int screenw = 1200;
int screenh = 600;
int plotw = 300;
int ploth = 80;
int time = 0;
float yaw = 0;
float pitch = 0;
float roll = 0;
float comYaw = 0;
int odoL = 0;
int odoR = 0;
int nextSendTime = 0;
int nextComPlotTime = 0;
float speedL = 0;
float speedR = 0;
float joyLeft = 0; 
float joyRight = 0;
float batteryVoltage = 0;
boolean joyActive = false;
int state = 0;
int imuState = IMU_RUN;
int motion = MOT_STOP;
int frq = 0;
boolean mouseDragged = false;
int packetsReceived = 0;
String bitmap;
String outline;
String particles;
PFont pf;
float comX=0;
float comY=0;
float comZ=0;
float comMag=0;
float periLeft = 0;
float periRight = 0;
float distanceCmSet = 0;
float angleRadSet = 0;
FloatList dataProb = new FloatList();
FloatList dataEffL = new FloatList();
FloatList dataEffR = new FloatList();
FloatList dataSenseL = new FloatList();
FloatList dataSenseR = new FloatList();
FloatList dataSenseMow = new FloatList();
FloatList dataSpeedL = new FloatList();
FloatList dataSpeedR = new FloatList();
FloatList dataOdoL = new FloatList();
FloatList dataOdoR = new FloatList();
FloatList dataPeriL = new FloatList();
FloatList dataPeriR = new FloatList();
FloatList dataYaw = new FloatList();
FloatList dataComYaw = new FloatList();
FloatList dataFrq = new FloatList();
FloatList dataComX = new FloatList();
FloatList dataComY = new FloatList();
FloatList dataComZ = new FloatList();
FloatList dataComMag = new FloatList();
FloatList dataRanging1 = new FloatList();
FloatList dataRanging2 = new FloatList();
FloatList dataRanging3 = new FloatList();
FloatList dataPIDimuError = new FloatList();
FloatList dataRadDeltaOdometry = new FloatList();
FloatList dataRadDeltaIMU = new FloatList();
FloatList dataPIDleftError = new FloatList();
FloatList dataPIDrightError = new FloatList();
  
  
  // rescale to -PI..+PI
float scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d);
  else if (d < -PI) return (2*PI+d);
  else return d;
}

String float2String(float number){
  String s = String.format("%.2f", number);
  s = s.replaceAll(",", ".");
  return s;
}

void sendPWM(float leftPWM, float rightPWM){
  if (demo){
    speedL = leftPWM;
    speedR = rightPWM;
    yaw -= leftPWM - rightPWM;    
    return;
  }
  if (demo) return;
  //println("sendPWM "+leftPWM + ", "+rightPWM);  
  sendPort("?02,"+ float2String(leftPWM) + "," + float2String(rightPWM) + "\n");  
}

void sendTravelLineDistance(float distance, float angle, float speed){
  sendPort("?06,"+ float2String(distance) + "," + float2String(angle) + "," + float2String(speed) + "\n");
}

void sendTravelLineTime(float duration, float angle, float speed){
  sendPort("?07,"+ float2String(duration) + "," + float2String(angle) + "," + float2String(speed) + "\n");
}

void sendRotateAngle(float angle, float speed){
  sendPort("?08,"+ float2String(angle) + "," + float2String(speed) + "\n");
}

void sendRotateTime(float duration, float speed){
  sendPort("?09,"+ float2String(duration) + "," + float2String(speed) + "\n");
}


void sendPort(String s){
  if (demo) return;
  if (useTcp) myTcp.write(s);
    else mySerial.write(s);
}


void setup(PApplet parent){
  println("-----setup-----");

  // List all the available serial ports
  print("Available serial ports: ");
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  if (!demo) {    
    if (useTcp) myTcp =  new Client(parent, tcpHost, tcpPort);  
      else mySerial = new Serial(parent, comPort, 115200, 'N', 8, 1);          
    // A serialEvent() is generated when a newline character is received :    
    if (mySerial != null) {      
      mySerial.bufferUntil('\n');      
      delay(1000);      
      mySerial.clear();      
      delay(1000);
    }
    loadEEPROM();
    sendPort("?83,0.7,0.1,3.0,2.0,0.03,0.03\n"); // motor settings
    sendPort("?84,10,15,0\n"); // perimeter settings
    sendPort("?82,1,0.01,0\n"); // IMU settings
    
    //if (mySerial != null) mySerial.buffer(32);    
  }  

  background(255);      // set inital background:
  
  pf = createFont("Arial Bold",14,true);
  textFont(pf,14);
  textAlign(LEFT);
  stroke(0, 0, 0);  
  //frameRate(30);
}

void drawMainMenu(){
  String menu = "{.main menu|mn0~stop|mn1~track|";
  menu += "mn2~mapping is ";
  if (map.stateMapping) menu += "ON";
    else menu += "OFF";    
  menu += "|mn3~lane mow|mn19~line|mn20~line angle +90deg";
  //mn4~request map|mn5~request outline|mn12~request particles|mn13~reset particles";
  menu += "|mn6~ADC cal|mn16~MPU selftest|mn17~IMU start cal|mn18~IMU stop cal|mn8~IMU verbose|mn9~mow 50% ON|mn11~mow ON|mn10~mow OFF|mn15~rotate +90deg}";
  drawMenu(330, 200, menu);
}

void drawMenu(int px, int py, String data){
  int w = 200;
  int h = 20;
  int selidx = -1;  
  stroke(0,0,0);  
  if (!data.startsWith("{")) return;
  if (!data.endsWith("}")) return;
  data = data.replaceAll("\\{","");
  data = data.replaceAll("\\}","");  
  String[] list = splitTokens(data, "|");
  if (list.length == 0) return;
  for (int i=0; i < list.length; i++){
    String[] items = splitTokens(list[i], "~");
    if (items.length >= 1){      
      fill(255,255,255);
      if ((mouseX >= px) && (mouseX <= px + w) && (mouseY >= py+i*h) && (mouseY <= py+i*h+h-1)){
        selidx = i;
        fill(0,0,0);        
      }      
      rect(px, py+i*h, w, h, 10);     
      fill(0,0,0);
      if (selidx == i) fill(255,255,255);      
      String title;
      textAlign(LEFT);
      if (items.length >=2) {
        title = items[1];
        text(title, px+10, py+i*h+h/2+5);
        if ((selidx ==i) && (mouseClicked)){
          mouseClicked = false;
          menuResponse = items[0];
        }
      } else { 
        title = items[0].replaceAll("\\.", "");
        textAlign(CENTER);
        text(title, px+w/2, py+i*h+h/2+5);
      }               
    }        
  }  
}

void drawInfo(int px, int py){
  int x = px;
  int y = py;
  int w = 20;
  fill(0,0,0);         
  text("--info--",                    x,y+0*w);  
  text("port:  "+comPort,       x,y+1*w);
  text("pkts:  "+packetsReceived, x,y+2*w);
  text("time:  "+time,                 x,y+3*w);
  text("frq:   "+frq,                  x,y+4*w);
  text("state: "+states[state],                x,y+5*w);
  text("motion: "+motions[motion],                x,y+6*w);
  text("imuState: "+imuStates[imuState],                x,y+7*w);
  text("mapScale: "+float2String(map.mapScaleX)+ ", "+float2String(map.mapScaleY), x,y+8*w);
  text("particlesDist: "+float2String(map.particlesDistance), x,y+9*w);
  text("odo:   "+odoL+", "+odoR,         x,y+10*w);  
  text("X:     "+float2String(map.robotState.x),         x,y+11*w);
  text("Y:     "+float2String(map.robotState.y),         x,y+12*w);
  text("dist:  "+float2String(map.overallDist),          x,y+13*w);
  text("voltage:  "+float2String(batteryVoltage),          x,y+14*w);
}

void plot(int idx, float minY, float maxY, FloatList list, String label, int posX, int posY, int r, int g, int b){  
  int h = ploth;
  int w = plotw;
  int px = posX;
  int oldpy = 0;  
  int py;
  float rangeY = abs(maxY-minY);   
  float stepY = ((float)h)/rangeY;  
  stroke(0,0,0);
  fill(255,255,255);
  rect(px, posY, w, h, 10);
  // zero line
  stroke(200,200,200);
  py = posY + h-((int)( (0-minY) *stepY ));
  line(px, py, px+w, py);
  fill(r,g,b);
  stroke(r,g,b);
  text(label, posX+w+10, posY+idx*20+15);
  if (list.size() < 2) return;
  text(float2String(list.get(list.size()-1)), posX+w+90, posY+idx*20+15);
  for (int i=0; i < list.size(); i++){
    //println(list.get(i));
    float value = max(minY, min(maxY, list.get(i)));
    py = posY + h-((int)( (value-minY) *stepY ));    
    if (i > 0) line(px-1, oldpy, px, py);    
    px++;
    oldpy = py;
  }
}

void addPlotData(FloatList list, float value){
  list.append( value );
  if (list.size() > 300) list.remove(0);
}

void drawPlots(int px, int py){
  int x = px;
  int y = py;  
  plot(0, -25, 25,     dataSpeedL, "speedL", x, y+0*ploth, 255, 0, 0);
  plot(1, -25, 25,     dataSpeedR, "speedR", x, y+0*ploth, 0, 127, 0);    
  plot(0, -181, 181,   dataYaw,    "yaw",    x, y+1*ploth, 0, 0, 255);
  plot(1, -181, 181,   dataComYaw, "comYaw", x, y+1*ploth, 120, 120, 0);  
  plot(0, -2200, 2200, dataPeriL,   "periL",   x, y+2*ploth, 255, 0, 255);  
  plot(1, -2200, 2200, dataPeriR,   "periR",   x, y+2*ploth, 0, 127, 0);
  plot(0, -500, 500, dataComZ,   "comZ",   x, y+3*ploth, 0, 0, 255);
  plot(1, -900, 900, dataComX,   "comX",   x, y+3*ploth, 120, 120, 0);
  plot(2, -900, 900, dataComY,   "comY",   x, y+3*ploth, 0, 127, 0);  
  plot(3, -900, 900, dataComMag,   "comMag",   x, y+3*ploth, 255, 0, 0);
  plot(0, -0.1, 1, dataSenseL,   "senL",   x, y+4*ploth, 255, 0, 0);
  plot(1, -0.1, 1, dataSenseR,   "senR",   x, y+4*ploth, 0, 127, 0);
  plot(2, -0.1, 1, dataSenseMow,   "senMow",   x, y+4*ploth, 0, 0, 255);
  plot(0, -0.1, 0.3, dataEffL,   "effL",   x, y+5*ploth, 255, 0, 0);
  plot(1, -0.1, 0.3, dataEffR,   "effR",   x, y+5*ploth, 0, 127, 0);   
  plot(0, -0.1, 1.1, dataProb,   "prob",   x, y+6*ploth, 255, 0, 0);
  /*plot(0, -0.2, 5, dataRanging1,   "rang1",   x, y+7*ploth, 255, 0, 0);
  plot(1, -0.2, 5, dataRanging2,   "rang2",   x, y+7*ploth, 0, 127 , 0);
  plot(2, -0.2, 5, dataRanging3,   "rang3",   x, y+7*ploth, 0, 0, 255);*/
  plot(0, -5, 5, dataPIDimuError,   "imuE",   x, y+7*ploth, 255, 0, 0);
  //plot(1, -500, 500, dataPIDleftError,   "leftE",   x, y+7*ploth, 0, 127 , 0);
  //plot(2, -500, 500, dataPIDrightError,   "rightE",   x, y+7*ploth, 0, 0, 255);
  plot(1, -0.1, 0.1, dataRadDeltaOdometry,   "delOdo",   x, y+7*ploth, 0, 127, 0);  
  plot(2, -0.1, 0.1, dataRadDeltaIMU,   "delIMU",   x, y+7*ploth, 0, 0, 255);
  
}

void drawJoystick(int px, int py){
  pushMatrix();
  int cx = px;
  int cy = py;
  int w = 150;
  fill(0, 255, 0);
  stroke(0, 0, 0);
  strokeWeight(2);
  ellipse(cx, cy, w, w);
  stroke(127,127,127);
  line(cx-w/2, cy, cx+w/2, cy);
  line(cx, cy-w/2, cx, cy+w/2);
  int mx = mouseX;
  int my = mouseY;
  float dx = (float(mx-cx))/((float)w/2);
  float dy = (float(my-cy))/((float)w/2);
  if ((mouseDragged) && (abs(dx) < 1.5) && (abs(dy) < 1.5)){ 
    dx = min(1.0, max(-1.0, dx));
    dy = min(1.0, max(-1.0, dy));
    mx = mouseX;
    my = mouseY;
    joyActive = true;
  } else {
    mx = cx;
    my = cy;
    dx = 0;
    dy = 0;
    if (joyActive) {
      sendPWM(0, 0);
    }
    joyActive = false;
  }
  translate(0,0,1);
  stroke(0,0,0);
  fill(0,0,0);  
  ellipse(mx, my, 20, 20);
  float maxspeed = 1.0;
  float steer = maxspeed / 2.0 * ((float)dx);
  if (dy < 0) steer *= -1;
  joyLeft = min(maxspeed, max(-maxspeed, Math.round(100.0*(maxspeed * -1*dy - steer))/100.0  ));
  joyRight = min(maxspeed, max(-maxspeed, Math.round(100.0*(maxspeed * -1*dy + steer))/100.0  ));                  
  fill(0, 0, 0);
  text("joy: " + joyLeft + ","+ joyRight, 280,30);
  if (millis() >= nextSendTime){
    nextSendTime = millis() + 500;
    if (joyActive) {
      sendPWM(joyLeft, joyRight);
    }
  }
  popMatrix();
}

void processMenu(){
  if (menuResponse == null) return;
  println("menu: "+menuResponse);
  if (menuResponse.equals("mn0")) {
    // stop    
    map.stateMapping = false;
    map.stateLocalize = true;
    map.stateLocalizeOutline = false;
    map.stateMowing = false;
    sendPort("?00\n");
  }
  if (menuResponse.equals("mn1")) {
    // track
    //map.overallDist = 0;
    map.distributeParticlesOutline();    
    map.stateMapping = false;
    map.stateLocalize=true;
    map.stateLocalizeOutline=true;
    map.stateMowing = false;
    sendPort("?11\n");    
  }
  if (menuResponse.equals("mn2")) {
    // mapping
    if (map.stateMapping){
      map.stopMapping();
      sendPort("?00\n");
    } else {
      map.startMapping();       
      //sendPort("?12\n");    
      sendPort("?11\n");
    }
  }
  if (menuResponse.equals("mn3")) {
    // lane mow
    map.stateMapping=false;
    map.stateLocalize=true;
    map.stateLocalizeOutline=false;
    map.stateMowing = true;
    sendPort("?13\n");
  }
  if (menuResponse.equals("mn4")) sendPort("?03\n");
  if (menuResponse.equals("mn5")) sendPort("?05\n");
  if (menuResponse.equals("mn6")) sendPort("?71\n");  
  if (menuResponse.equals("mn8")) sendPort("?73\n");
  if (menuResponse.equals("mn9")) sendPort("?74,0.5\n");  
  if (menuResponse.equals("mn10")) sendPort("?74,0\n");
  if (menuResponse.equals("mn11")) sendPort("?74,1.0\n");  
  if (menuResponse.equals("mn14")) sendPort("?75\n");
  //if (menuResponse.equals("mn15")) sendPort("?08," + scalePI(PI/180.0*20.0) + ",0.1\n");
  if (menuResponse.equals("mn15")) sendRotateAngle(scalePI(PI/180.0*90.0), manualSpeed);  
  if (menuResponse.equals("mn16")) sendPort("?79\n");
  if (menuResponse.equals("mn17")) sendPort("?80\n");
  if (menuResponse.equals("mn18")) sendPort("?81\n");
  //if (menuResponse.equals("mn19")) sendPort("?06,10000," + scalePI(yaw+PI/180.0*90.0) + ",1.0\n");
  if (menuResponse.equals("mn19")) sendTravelLineDistance(10000, scalePI(yaw), manualSpeed);  
  if (menuResponse.equals("mn20")) sendPort("?85,10000," + scalePI(PI/180.0*90.0) + ",1.0\n");
  if (menuResponse.equals("mn13")) {
    // reset particles
    map.distributeParticlesOutline();
    //map.overallDist = 0;
    //mySerial.write("?16\n");
  }
    
  menuResponse = null;
}

void saveMagData(String line){  
  //println("saveMagData " + magDataCount);
  try{    
    BufferedWriter writer = new BufferedWriter(new FileWriter("d:\\temp\\magdata.csv", true));  
    writer.write(line + "\r\n");    
    writer.close();
  } catch (IOException e){
    println("error writing");
  }
}

void draw () {
  if (myTcp != null){
    while (myTcp.available() > 32) {
      String data = myTcp.readStringUntil('\n');
      processDataReceived(data);
    }
  }
  /*if (millis() >= nextSendTime){
    nextSendTime = millis() + 50;
    mySerial.write("?99\n");
  }*/
  if (demo){
    //yaw = scalePI(yaw + random(-PI/10, PI/10));
    //comYaw = scalePI(yaw + PI/4); 
    /*pitch = scalePI(pitch + PI/300);
    roll = scalePI(roll + PI/300);*/
    addPlotData(dataSpeedL, speedL );
    addPlotData(dataSpeedR, speedR );    
    //addPlotData(dataPeriL, random(-100, 100) );
    //addPlotData(dataPeriR, random(-100, 100) );
    addPlotData(dataYaw,  yaw/PI*180.0);
    //addPlotData(dataComYaw,  comYaw/PI*180.0);
    /*addPlotData(dataComX, random(1000, 2000) );
    addPlotData(dataComY, random(0, 500) );
    addPlotData(dataComZ, random(-1000, -500) );
    addPlotData(dataEffL, random(-1, 1) );    
    addPlotData(dataEffR, random(-1, 1) );*/
    addPlotData(dataProb, map.overallProb );
    map.stateLocalize = true;
    map.stateLocalizeOutline = true;
    float dist = 0;    
    dist = (speedL+speedR)/2.0 * 0.01;
    MapData md = map.getMapDataMeter(map.robotState.x, map.robotState.y);
    if (md != null){      
      periLeft = md.signal*50;
      periRight = md.signal*50;
      if (md.inside) {
        periLeft *= -1;
        periRight *= -1;
      }
    }
    addPlotData(dataPeriL, periLeft);
    addPlotData(dataPeriR, periRight);       
    if ( (abs(speedL) > 0) || (abs(speedR) > 0) ) map.run(yaw, dist, periLeft, periRight);
  }  
 //  if (!updateScreen) return;
  updateScreen=false;
  /*float fov = PI/3.0; 
  float cameraZ = (height/2.0) / tan(fov/2.0);   
  perspective(fov, float(width)/float(height), cameraZ/2.0, cameraZ*2.0);*/
  background(210);
  lights();  
  pointLight(100, 200, 160,   500, 0, 100); // color, pos
  //ambientLight(255, 255, 255);
  //drawHelp();
  drawInfo(550, 200);
  drawJoystick(400, 100);
  drawCompass(580,100);
  drawPlots(710, 10);  
  drawRobot(130,420);
  drawMainMenu();
  fill(0,0,0);
  text("console", 10,600);
  if (inString != null){           
    text(inString,10,630);    
  }
  text("map", 10,30);
  map.draw();  
         
  if (!focused){
    //text("click window for keyboard input!", 10,550);    
  }
  processMenu();  
}

void drawNeedle(int cx, int cy, float course){
  //float yaw = course+PI/2;
  float yaw = course;
  int w = 150;
  int px = Math.round(cos(yaw) * w/2);  
  int py = Math.round(sin(yaw) * w/2);    
  line(cx+px,cy-py, cx-px,cy+py);  
  line(cx+px,cy-py, Math.round(cx + cos(yaw-0.1)*w*0.4), Math.round( cy - sin(yaw-0.1)*w*0.4 ));
  line(cx+px,cy-py, Math.round(cx + cos(yaw+0.1)*w*0.4), Math.round( cy - sin(yaw+0.1)*w*0.4 ));    
}

void drawCompass(int cx, int cy){
  int w = 150;
  fill(200, 200, 255);
  stroke(0, 0, 0);  
  strokeWeight(2);
  ellipse(cx, cy, w, w);
  stroke(200,200,200);
  line(cx, cy, cx+w/2, cy);  
  strokeWeight(4);
  stroke(180,180,0);
  drawNeedle(cx,cy,comYaw);
  stroke(0,0,255);
  drawNeedle(cx,cy,yaw);  
  strokeWeight(2);
  translate(0,0,1);
  rect(cx-30,cy-10,60,20,10);
  fill(0,0,0);  
  text(float2String(yaw/PI*180.0), cx-20, cy+5);  
}

void drawRobot(int px, int py){    
  noStroke();
  // body
  fill(255, 80, 80);
  //fill(255);
  pushMatrix();
    translate(px, py, 0);
    //rotateY(-PI/180*10);
    pushMatrix();
      rotateZ(pitch);
      rotateX(roll);         
      scale(2, 0.5, 1);
      box(50);
      // tires
      scale(0.5, 2.0, 0.1);
      fill(120, 120, 255);
      pushMatrix();
        translate(20, 10, -250);  
        sphere(30); 
      popMatrix(); 
      translate(20, 10, 250);  
      sphere(30);   
    popMatrix();
    // indicate level axis    
    fill(255, 0, 0);
    pushMatrix();
      scale(250, 1, 1);
      box(1);
    popMatrix();
    fill(0, 0, 255);
    pushMatrix();      
      scale(1, 1, 250);
      box(1);
    popMatrix();
    fill(0, 200, 0);
    pushMatrix();
      scale(1, 200, 1);
      box(1);
    popMatrix();        
  popMatrix();
  fill(0,0,255);  
  text("pitch: "+float2String(pitch/PI*180.0), px+60, py-30);
  fill(255,0,0);
  text("roll: "+float2String(roll/PI*180.0), px+70, py+20);
}

void processDataReceived(String data) {    
  if (data != null) {    
    data = trim(data);                // trim off whitespaces.
    packetsReceived++;    
    if ( (verboseOutput) || (!data.startsWith("!")) ) {
      inString = data;
      println(">"+inString);  
    }
    //println(inString);
    if (data.startsWith("!17")){
      // robot motion
      String[] list = splitTokens(data, ",");
      if (list.length >= 3){
        float distance = Float.parseFloat(list[1]);
        float yaw = Float.parseFloat(list[2]);
        map.run(yaw, distance, periLeft, periRight); 
      }      
    }    
    if (data.startsWith("!01")){
      String[] list = splitTokens(data, ",");
      if (list.length >= 33){
        time = Integer.parseInt(list[1]);
        state = Integer.parseInt(list[2]);
        frq = Integer.parseInt(list[3]);
        odoL = Integer.parseInt(list[4]);
        odoR = Integer.parseInt(list[5]);
        speedL = Float.parseFloat(list[6]);
        speedR = Float.parseFloat(list[7]);        
        yaw =  Float.parseFloat(list[10]);
        comYaw = Float.parseFloat(list[17]);
        pitch = Float.parseFloat(list[18]);
        roll = Float.parseFloat(list[19]);      
        //map.robotState.x = Float.parseFloat(list[20]);
        //map.robotState.y = Float.parseFloat(list[21]);
        periLeft = Integer.parseInt(list[8]);
        periRight = Integer.parseInt(list[9]);
        addPlotData(dataPeriL, periLeft);
        addPlotData(dataPeriR, periRight);
        addPlotData(dataSpeedL, speedL);      
        addPlotData(dataSpeedR, speedR);
        addPlotData(dataYaw, yaw/PI*180.0);
        addPlotData(dataComYaw, comYaw/PI*180.0);        
        addPlotData(dataSenseL, Float.parseFloat(list[22]));
        addPlotData(dataSenseR, Float.parseFloat(list[23]));
        addPlotData(dataSenseMow, Float.parseFloat(list[24]));
        addPlotData(dataEffL, Float.parseFloat(list[25]));        
        addPlotData(dataEffR, Float.parseFloat(list[26]));        
        //addPlotData(dataProb, Float.parseFloat(list[27]));
        addPlotData(dataProb, map.overallProb);
        batteryVoltage = Float.parseFloat(list[28]);
        motion = Integer.parseInt(list[29]);
        imuState = Integer.parseInt(list[30]);
        distanceCmSet = Float.parseFloat(list[31]);
        angleRadSet = Float.parseFloat(list[32]);
        
        //map.run(yaw, 0, periLeft, periRight);
        
        String line =   time + "," + yaw + "," + pitch + "," + roll;                          
        //saveMagData(line);
      }
    }
    if (data.startsWith("!77")){
      // ranging data
      String[] list = splitTokens(data, ",");
      if (list.length >= 5){        
        int time = Integer.parseInt(list[1]);
        int addr  = Integer.parseInt(list[2]);                
        float distance  = Float.parseFloat(list[3]);
        float power  = Float.parseFloat(list[4]);
        if (addr == 1) addPlotData(dataRanging1, distance);
        if (addr == 2) addPlotData(dataRanging2, distance);
        if (addr == 3) addPlotData(dataRanging3, distance);
      }
    }
    if (data.startsWith("!86")){
      // motor controller data
      String[] list = splitTokens(data, ",");
      //println(data);      
      if (list.length >= 9){
        addPlotData(dataPIDimuError, Float.parseFloat(list[1]));        
        //  addPlotData(dataPIDleftError, Float.parseFloat(list[5]));
        //  addPlotData(dataPIDrightError, Float.parseFloat(list[6]));
        addPlotData(dataRadDeltaOdometry, Float.parseFloat(list[7]));        
        addPlotData(dataRadDeltaIMU, Float.parseFloat(list[8]));        
      }
    }    
    if (data.startsWith("!76")){
      // eeprom data
      String[] list = splitTokens(data, ",");
      if (list.length >= 3){        
        int addr = Integer.parseInt(list[1]);
        int value  = Integer.parseInt(list[2]);                
        eeprom.put(new Integer(addr), new Integer(value));        
        saveEEPROM();        
      }
    }  
    if (data.startsWith("!04")){
      // IMU data (verbose)
      String[] list = splitTokens(data, ",");
      if (list.length >= 13){        
        String line =   millis() + "," + list[1] + "," + list[2] + "," + list[3] + ","    
                      + list[4] + "," + list[5] + "," + list[6] + ","
                      + list[7] + "," + list[8] + "," + list[9] + "," + list[10];
        //saveMagData(line);        
        comX = 0.99 * comX + 0.01 * Float.parseFloat(list[1]);
        comY = 0.99 * comY + 0.01 * Float.parseFloat(list[2]);
        comZ = 0.99 * comZ + 0.01 * Float.parseFloat(list[3]);
        comMag = 0.99 * comMag + 0.01 * sqrt(sq(comX) + sq(comY) + sq(comZ)); 
        if (millis() >= nextComPlotTime){
          nextComPlotTime = millis() + 1000;
          addPlotData(dataComX, comX);
          addPlotData(dataComY, comY);
          addPlotData(dataComZ, comZ);
          addPlotData(dataComMag, comMag);
          //pitch = Float.parseFloat(list[11]);
          //roll = Float.parseFloat(list[12]);
        }
      }
    }
    if (data.startsWith("!03")){
       // map
       bitmap = data;       
    }
    if (data.startsWith("!05")){
      // outline
      outline = data;       
    }
    if (data.startsWith("!15")){
      // particles
      particles = data;       
    }           
    updateScreen=true;
  }
}
 
  
void serialEvent (Serial mySerial) {    
 try{
  String data = mySerial.readStringUntil('\n');
  processDataReceived(data);
 } catch (Exception e){
   println("EX: "+e);
 }
}


void mousePressed() {
  //println("mouse=" + mouseX + "," + mouseY); 
  //line(mouseX, 10, mouseX, 90);
  mouseDragged = true;
}

void mouseClicked(){
  mouseClicked = true;
  int x = mouseX;
  int y = mouseY;
  if (map.isInsideWindow(x,y)){
    map.setRobotPositionManual(x, y);
    map.overallDist = 0;
  }
}

void mouseMoved(){      
}

void mouseDragged(){
  //mouseDragged = true;
}

void mouseReleased(){
  mouseDragged = false;
}


  public void saveEEPROM() {
    println("saveEEPROM size="+eeprom.size() );
    try{
      FileOutputStream fout= new FileOutputStream (EEPROM_FILENAME);
      ObjectOutputStream oos = new ObjectOutputStream(fout);      
      oos.writeObject(eeprom);      
      fout.close();
    } catch (Exception e){
      //e.printStackTrace();
      println("ERROR saving EEPROM file");
    }    
  }
  
  public void loadEEPROM() {    
    println("loadEEPROM");    
    try{
      FileInputStream fin= new FileInputStream (EEPROM_FILENAME);
      ObjectInputStream ois = new ObjectInputStream(fin);
      eeprom = (HashMap)ois.readObject();      
      fin.close();
      println("EEPROM file size="+ eeprom.size());
      for (Integer key : eeprom.keySet()) {
         Integer value = eeprom.get(key);
         print(key + "=" + value + ", ");
      }
      println();
      /*for (Integer key : eeprom.keySet()) {
        Integer value = eeprom.get(key);        
        mySerial.write("?76,"+ key + "," + value + "\n");
        delay(20); // avoids Arduino serial buffer overflow
      }*/
    } catch (Exception e){
      //e.printStackTrace();
      println("ERROR loading EEPROM file");
    }       
  }

  
}
  

  