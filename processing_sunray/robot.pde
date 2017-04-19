import java.io.*;


class Robot {
  
// motor
static final float robotMass = 1500;
static final float rpmMax = 25;
static final float reverseSpeedPerc = 0.3;
static final float rotationSpeedPerc = 0.3;
static final float trackSpeedPerc = 0.3;
static final float trackRotationSpeedPerc = 0.1;
static final float motorFrictionMin = 0.2;
static final float motorFrictionMax = 1700.0;
static final float mowSenseMax = 4.0;
static final float imuPID_Kp = 0.7;
static final float imuPID_Ki = 0.1;
static final float imuPID_Kd = 3.0;
static final float motorPID_Kp = 2.0;
static final float motorPID_Ki = 0.03;
static final float motorPID_Kd = 0.03;
static final float stuckMaxDiffOdometryIMU = 0.2;
static final float stuckMaxIMUerror = 5.0;
// perimeter
static final int timedOutIfBelowSmag = 10;
static final int timeOutSecIfNotInside = 15;
static final byte swapCoilPolarity = 0;
// IMU
static final byte useGyro = 1;
static final float gyroBiasDpsMax = 0.01;
static final float imuMode = 0;


boolean verboseOutput = false;
  
  
static final String EEPROM_FILENAME = "eeprom.bin";

String[] states = {"IDLE", "GYRO", "TRAK", "MAP", "MOW", "R/C", "CHG"};
static final int STAT_IDLE       = 0;
static final int STAT_CAL_GYRO   = 1;
static final int STAT_TRACK      = 2;
static final int STAT_CREATE_MAP = 3;
static final int STAT_MOW        = 4;
static final int STAT_RC         = 5;
static final int STAT_CHG        = 6;

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

String[] logStates = {"OFF", "PLAY", "REC"};
static final int LOG_OFF      = 0;
static final int LOG_PLAY     = 1;
static final int LOG_REC      = 2;



//Variables
int mapVerbose = 1;
int imuVerbose = 0;
int motorVerbose = 0;
Map map = null;
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
int logState = LOG_OFF;
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
boolean playPaused = true;
Button btnMapping,btnStop,btnResetParticles,btnTrack,btnMowRand,btnMowLane,btnADCcal,btnMPUselftest;
Button btnIMUstartCal,btnIMUstopCal,btnMow50,btnMowON,btnMowOFF,btnLine,btnLineRev,btnLine90,btnRotate90;
Button btnPlayPaused = null;
Tabsheet tabPlot, tabMenu;
Sheet sheetPlotMain,sheetPlotMisc,sheetMenuMain,sheetMenuMisc;
Plot plotSpeedL, plotSpeedR, plotYaw, plotComYaw, plotCom, plotPeriL, plotPeriR, plotComZ, plotComX;
Plot plotComY, plotComMag, plotSenL, plotSenR, plotSenMow, plotFrictionL, plotFrictionR;
Plot plotPIDimuError,  plotDiffOdoIMU,   plotPIDleftError,   plotPIDrightError;
Plot plotAccY, plotAccZ, plotAccX, plotProb, plotParticlesDist; 
PrintWriter logOutput;
BufferedReader logInput;
PImage satImg;
int nextPlayTime = 0;
  
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

void sendStartRandomMowing(){
   sendPort("?14\n");
}

void sendVerbose(){
  sendPort("?73," + str(imuVerbose) + "," + str(motorVerbose) + "," + str(mapVerbose) + "\n");
}


void sendPort(String s){
  if (logState == LOG_PLAY) return;
  if (demo) return;
  if (useTcp) myTcp.write(s);
    else mySerial.write(s);
}


void setup(PApplet parent){
  println("-----setup-----");
  map = new Map();
  if (useSatMap) satImg = loadSatImage(centerLat, centerLon);

  // List all the available serial ports
  print("Available serial ports: ");
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  tabPlot = new Tabsheet(null, 750,0,475,680);   
  sheetPlotMain = new Sheet(tabPlot, "1");
  sheetPlotMisc = new Sheet(tabPlot, "2");
  tabPlot.activeSheet = sheetPlotMain;
  
  tabMenu = new Tabsheet(null, 320,200,220,400);   
  sheetMenuMain = new Sheet(tabMenu, "1");
  sheetMenuMisc = new Sheet(tabMenu, "2");
  tabMenu.activeSheet = sheetMenuMain;  
  logging(null);
  createPlots();
  createMenu();

  background(255);      // set inital background:
  
  pf = createFont("Arial Bold",14,true);
  textFont(pf,14);
  textAlign(LEFT);
  stroke(0, 0, 0);  
  //frameRate(30);
  
  if ((!demo) && (logState != LOG_PLAY))  {    
    if (useTcp) myTcp =  new Client(parent, tcpHost, tcpPort);  
      else mySerial = new Serial(parent, comPort, 115200, 'N', 8, 1);          
    // A serialEvent() is generated when a newline character is received :    
    if (mySerial != null) {      
      mySerial.bufferUntil('\n');      
      delay(1000);      
      mySerial.clear();      
      delay(1000);
    }
    //loadEEPROM();
    // motor settings
    sendPort("?83," + float2String(rpmMax) + "," + float2String(reverseSpeedPerc) + "," + float2String(rotationSpeedPerc) + "," 
       + float2String(trackSpeedPerc) + "," + float2String(trackRotationSpeedPerc) + "," + float2String(robotMass) + "," 
       + float2String(motorFrictionMin) + "," + float2String(motorFrictionMax) + "," 
       + float2String(mowSenseMax) + "," + float2String(imuPID_Kp) + "," + float2String(imuPID_Ki) + "," + float2String(imuPID_Kd) + ","
       + float2String(motorPID_Kp) + "," + float2String(motorPID_Ki) + "," + float2String(motorPID_Kd) + "," 
       + float2String(stuckMaxDiffOdometryIMU) + "," + float2String(stuckMaxIMUerror) + "\n");
    // perimeter settings
    sendPort("?84," + str(timedOutIfBelowSmag) + "," + str(timeOutSecIfNotInside) + "," + str(swapCoilPolarity) + "\n"); 
    // IMU settings
    sendPort("?82," + str(useGyro) +  "," + float2String(gyroBiasDpsMax) + "," + str(imuMode) + "\n");
    sendVerbose();
    
    //if (mySerial != null) mySerial.buffer(32);    
  }    
}


void drawInfo(int px, int py){
  int x = px;
  int y = py;
  int w = 20;
  fill(0,0,0);         
  text("log: "+logFile + " ("+logStates[logState]+")",   x,y+0*w);  
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


void addPlotData(FloatList list, float value){  
  list.append( value );
  if (list.size() > 300) list.remove(0);
}

void createPlots(){
  int x = 0;
  int y = 0;  
  plotSpeedL = new Plot(sheetPlotMain, 0, -25, 25,     "speedL", x, y+0*ploth, plotw, ploth, 255, 0, 0);
  plotSpeedR = new Plot(sheetPlotMain,1, -25, 25,      "speedR", x, y+0*ploth, plotw, ploth, 0, 127, 0);    
  plotYaw = new Plot(sheetPlotMain,0, -181, 181,       "yaw",    x, y+1*ploth, plotw, ploth, 0, 0, 255);
  plotComYaw = new Plot(sheetPlotMain,1, -181, 181,    "comYaw", x, y+1*ploth, plotw, ploth, 120, 120, 0);  
  plotPeriL = new Plot(sheetPlotMain,0, -2800, 2800,    "periL",   x, y+2*ploth, plotw, ploth, 255, 0, 255);  
  plotPeriR = new Plot(sheetPlotMain,1, -2800, 2800,    "periR",   x, y+2*ploth, plotw, ploth, 0, 127, 0);
  plotComZ = new Plot(sheetPlotMain,0, -500, 500,    "comZ",   x, y+3*ploth, plotw, ploth, 0, 0, 255);
  plotComX = new Plot(sheetPlotMain,1, -900, 900,   "comX",   x, y+3*ploth, plotw, ploth, 120, 120, 0);
  plotComY = new Plot(sheetPlotMain,2, -900, 900, "comY",   x, y+3*ploth, plotw, ploth, 0, 127, 0);  
  plotComMag =new Plot(sheetPlotMain,3, -900, 900, "comMag",   x, y+3*ploth, plotw, ploth, 255, 0, 0);
  plotSenL = new Plot(sheetPlotMain,0, -0.15, 2, "senL",   x, y+4*ploth, plotw, ploth, 255, 0, 0);
  plotSenR = new Plot(sheetPlotMain,1, -0.15, 2, "senR",   x, y+4*ploth, plotw, ploth, 0, 127, 0);
  plotSenMow = new Plot(sheetPlotMain,2, -0.3, 5, "senMow",   x, y+4*ploth, plotw, ploth, 0, 0, 255);
  plotFrictionL = new Plot(sheetPlotMain,0, -0.15, 4000, "frictionL",   x, y+5*ploth, plotw, ploth, 255, 0, 0);
  plotFrictionR = new Plot(sheetPlotMain,1, -0.15, 4000, "frictionR",   x, y+5*ploth, plotw, ploth, 0, 127, 0);     
  /*plot(0, -0.2, 5, dataRanging1,   "rang1",   x, y+7*ploth, 255, 0, 0);
  plot(1, -0.2, 5, dataRanging2,   "rang2",   x, y+7*ploth, 0, 127 , 0);
  plot(2, -0.2, 5, dataRanging3,   "rang3",   x, y+7*ploth, 0, 0, 255);*/
  plotPIDimuError = new Plot(sheetPlotMain,0, -5, 5, "imuErr",   x, y+6*ploth, plotw, ploth, 255, 0, 0);  
  plotDiffOdoIMU = new Plot(sheetPlotMain,1, -0.3, 0.3,  "diffOdoIMU",   x, y+6*ploth, plotw, ploth, 0, 127, 0);  
  plotPIDleftError = new Plot(sheetPlotMain,2, -40, 40, "leftErr",   x, y+6*ploth, plotw, ploth, 0, 0, 255);   
  plotPIDrightError = new Plot(sheetPlotMain,3, -40, 40, "rightErr",   x, y+6*ploth, plotw, ploth, 120, 120, 0);
  plotAccY = new Plot(sheetPlotMain,0, -1, 1, "accY",   x, y+7*ploth, plotw, ploth, 120, 120, 0);
  plotAccZ = new Plot(sheetPlotMain,1, -1, 1, "accZ",   x, y+7*ploth, plotw, ploth, 0, 0, 127);
  plotAccX = new Plot(sheetPlotMain,2, -1, 1, "accX",   x, y+7*ploth, plotw, ploth, 255, 0, 0);
  plotProb = new Plot(sheetPlotMisc, 0, -0.1, 1.1, "prob",   x, y+0*ploth, plotw, ploth, 255, 0, 0);  
  plotParticlesDist = new Plot(sheetPlotMisc, 0, -30, 30, "partDist",   x, y+1*ploth, plotw, ploth, 255, 0, 0);  
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
  if ((mouseDragged) && (abs(dx) < 1.1) && (abs(dy) < 1.1)){ 
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

void createMenu(){
  String s; 
  int h = 26;
  btnStop  = new Button(null, 0, 0, 100, 26, "STOP");
  if (logState == LOG_PLAY){
    s = "Playing";
    if (playPaused) s += " paused";
    btnPlayPaused  = new Button(null, 100, 0, 150, 26, s);
  }
  s = "OFF";
  if (map.stateMapping) s = "ON";
  btnMapping  = new Button(sheetMenuMain, 0, h*0, 220, 26, "mapping is "+s);
  btnResetParticles  = new Button(sheetMenuMain, 0, h*1, 220, 26, "reset particles");
  btnTrack  = new Button(sheetMenuMain, 0, h*2, 220, 26, "track");   
  btnMowRand  = new Button(sheetMenuMain, 0, h*3, 220, 26, "mow rand");
  btnMowLane  = new Button(sheetMenuMain, 0, h*4, 220, 26, "mow lane");
   
  btnADCcal  = new Button(sheetMenuMisc, 0, h*0, 220, 26, "ADC cal");
  btnMPUselftest  = new Button(sheetMenuMisc, 0, h*1, 220, 26, "MPU selftest");
  btnIMUstartCal  = new Button(sheetMenuMisc, 0, h*2, 220, 26, "IMU start cal");
  btnIMUstopCal  = new Button(sheetMenuMisc, 0, h*3, 220, 26, "IMU stop cal");  
  btnMow50  = new Button(sheetMenuMisc, 0, h*4, 220, 26, "Mow 50% ON");
  btnMowON  = new Button(sheetMenuMisc, 0, h*5, 220, 26, "Mow ON");
  btnMowOFF  = new Button(sheetMenuMisc, 0, h*6, 220, 26, "Mow OFF");
  btnLine  = new Button(sheetMenuMisc, 0, h*7, 220, 26, "line");
  btnLineRev  = new Button(sheetMenuMisc, 0, h*8, 220, 26, "line rev");
  btnLine90  = new Button(sheetMenuMisc, 0, h*9, 220, 26, "line angle +90deg");
  btnRotate90  = new Button(sheetMenuMisc, 0, h*10, 220, 26, "rotate +90deg");     
}

void processMenu(){
  /*String menu = "{.main menu|mn0~stop|mn1~track|";
  menu += "mn2~mapping is ";
  if (map.stateMapping) menu += "ON";
    else menu += "OFF";    
  menu += "|mn21~rand mow|mn3~lane mow|mn19~line|mn22~line rev|mn20~line angle +90deg";
  //mn4~request map|mn5~request outline|mn12~request particles";
  menu += "|mn13~reset particles";
  menu += "|mn6~ADC cal|mn16~MPU selftest|mn17~IMU start cal|mn18~IMU stop cal|mn24~map verbose|mn23~motor verbose|mn8~IMU verbose|mn9~mow 50% ON|mn11~mow ON|mn10~mow OFF|mn15~rotate +90deg}";
  drawMenu(330, 200, menu);*/
    
  if (btnStop.clicked) {
    // stop    
    map.stateMapping = false;
    map.stateLocalize = true;
    map.stateLocalizeOutline = false;
    map.stateMowing = false;
    sendPort("?00\n");
  }
  if ((btnPlayPaused != null) && (btnPlayPaused.clicked)){
    // pause/play
    playPaused = !playPaused;
    String s = "Playing";
    if (playPaused) s += " paused";
    btnPlayPaused.label = s; 
  }
  if (btnTrack.clicked) {
    // track
    //map.overallDist = 0;
    map.distributeParticlesOutline();    
    map.stateMapping = false;
    map.stateLocalize=true;
    map.stateLocalizeOutline=true;
    map.stateMowing = false;
    sendPort("?11\n");    
  }
  if (btnMapping.clicked) {
    // mapping
    if (map.stateMapping){
      map.stopMapping();
      sendPort("?00\n");
    } else {
      map.startMapping();       
      //sendPort("?12\n");    
      sendPort("?11\n");
    }  
    String s = "OFF";
    if (map.stateMapping) s = "ON";
    btnMapping.label = "mapping is "+s;
  }
  if (btnResetParticles.clicked) {
    // reset particles
    map.distributeParticlesOutline();
    //map.overallDist = 0;
    //mySerial.write("?16\n");
  }
  if (btnMowRand.clicked) {
    map.stateMapping=false;
    map.stateLocalize=true;
    map.stateLocalizeOutline=false;
    map.stateMowing = true;
    sendStartRandomMowing();
  }
  if (btnMowLane.clicked) {
    // lane mow
    map.stateMapping=false;
    map.stateLocalize=true;
    map.stateLocalizeOutline=false;
    map.stateMowing = true;
    sendPort("?13\n");
  }
  //if (menuResponse.equals("mn4")) sendPort("?03\n");
  //if (menuResponse.equals("mn5")) sendPort("?05\n");
  if (btnADCcal.clicked) sendPort("?71\n");  
  //if (menuResponse.equals("mn8")) { imuVerbose = (1-imuVerbose); sendVerbose(); }
  //if (menuResponse.equals("mn23")) { motorVerbose = (1-motorVerbose); sendVerbose(); }
  //if (menuResponse.equals("mn24")) { mapVerbose = (1-mapVerbose); sendVerbose(); }  
  if (btnMow50.clicked) sendPort("?74,0.5\n");  
  if (btnMowOFF.clicked) sendPort("?74,0\n");
  if (btnMowON.clicked) sendPort("?74,1.0\n");  
  //if (menuResponse.equals("mn14")) sendPort("?75\n");
  //if (menuResponse.equals("mn15")) sendPort("?08," + scalePI(PI/180.0*20.0) + ",0.1\n");  
  if (btnRotate90.clicked) sendRotateAngle(scalePI(PI/180.0*90.0), 0.2);  
  if (btnMPUselftest.clicked) sendPort("?79\n");
  if (btnIMUstartCal.clicked) sendPort("?80\n");
  if (btnIMUstopCal.clicked) sendPort("?81\n");
  //if (menuResponse.equals("mn19")) sendPort("?06,10000," + scalePI(yaw+PI/180.0*90.0) + ",1.0\n");
  if (btnLine.clicked) sendTravelLineDistance(10000, scalePI(yaw), 1.0);  
  if (btnLineRev.clicked) sendTravelLineDistance(20, scalePI(yaw), -1.0);  
  if (btnLine90.clicked) sendPort("?85,10000," + scalePI(PI/180.0*90.0) + ",1.0\n");
      
    
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
    plotSpeedL.addPlotData(speedL );
    plotSpeedR.addPlotData(speedR );    
    //addPlotData(dataPeriL, random(-100, 100) );
    //addPlotData(dataPeriR, random(-100, 100) );
    plotYaw.addPlotData(yaw/PI*180.0);
    //addPlotData(dataComYaw,  comYaw/PI*180.0);
    /*addPlotData(dataComX, random(1000, 2000) );
    addPlotData(dataComY, random(0, 500) );
    addPlotData(dataComZ, random(-1000, -500) );
    addPlotData(dataEffL, random(-1, 1) );    
    addPlotData(dataEffR, random(-1, 1) );*/
    //plotProb.addPlotData(map.overallProb );
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
    plotPeriL.addPlotData(periLeft);
    plotPeriR.addPlotData(periRight);       
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
  drawCompass(580,100,yaw,comYaw);   
  drawRobot(130,420, pitch, roll);
  btnStop.update();
  if (btnPlayPaused != null) btnPlayPaused.update();
  tabPlot.update();
  tabMenu.update();  
  fill(0,0,0);
  text("console", 10,600);
  if (inString != null){           
    text(inString,10,630);    
  }
  text("map", 10,40);
  map.draw();
          
  if (!focused){
    //text("click window for keyboard input!", 10,550);    
  }
  processMenu();
  
  tint(255, 127);  // Display at half opacity
  if (satImg != null) image(satImg, 0, 0);
  //image(mov, 0, 0);    
  
  if (logState == LOG_PLAY) {
    if (!playPaused){
      //if (millis() >= nextPlayTime){
        nextPlayTime = millis() + 10;
        logging(null);
      //}
    }
  }
}

void processDataReceived(String data) {
  if (logState == LOG_REC) logging(data); 
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
        plotProb.addPlotData(map.overallProb);
        plotParticlesDist.addPlotData(map.particlesDistance);
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
        plotPeriL.addPlotData(periLeft);
        plotPeriR.addPlotData(periRight);
        plotSpeedL.addPlotData(speedL);      
        plotSpeedR.addPlotData(speedR);
        plotYaw.addPlotData(yaw/PI*180.0);
        plotComYaw.addPlotData(comYaw/PI*180.0);        
        /*addPlotData(dataSenseL, Float.parseFloat(list[22]));
        addPlotData(dataSenseR, Float.parseFloat(list[23]));
        addPlotData(dataSenseMow, Float.parseFloat(list[24]));
        addPlotData(dataFrictionL, Float.parseFloat(list[25]));        
        addPlotData(dataFrictionR, Float.parseFloat(list[26]));*/        
        //addPlotData(dataProb, Float.parseFloat(list[27]));        
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
        /*if (addr == 1) addPlotData(dataRanging1, distance);
        if (addr == 2) addPlotData(dataRanging2, distance);
        if (addr == 3) addPlotData(dataRanging3, distance);*/
      }
    }
    if (data.startsWith("!86")){
      // motor controller data
      String[] list = splitTokens(data, ",");
      //println(data);      
      if (list.length >= 10){
        plotSenL.addPlotData(Float.parseFloat(list[1]));
        plotSenR.addPlotData(Float.parseFloat(list[2]));
        plotSenMow.addPlotData(Float.parseFloat(list[3]));
        plotFrictionL.addPlotData(Float.parseFloat(list[4]));        
        plotFrictionR.addPlotData(Float.parseFloat(list[5]));
        plotDiffOdoIMU.addPlotData(Float.parseFloat(list[6]));
        plotPIDimuError.addPlotData(Float.parseFloat(list[7]));                                             
        plotPIDleftError.addPlotData(Float.parseFloat(list[8]));
        plotPIDrightError.addPlotData(Float.parseFloat(list[9]));
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
      if (list.length >= 18){        
        String line =   millis() + "," + list[1] + "," + list[2] + "," + list[3] + ","    
                      + list[4] + "," + list[5] + "," + list[6] + ","
                      + list[7] + "," + list[8] + "," + list[9] + "," + list[10] + ","
                      + list[11] + "," + list[12] + "," 
                      + list[13] + "," + list[14] + "," + list[15] + "," 
                      + list[16] + "," + list[17];
        //saveMagData(line);
        float accX = Float.parseFloat(list[13]);
        float accY = Float.parseFloat(list[14]);
        float accZ = Float.parseFloat(list[15]);
        float accXmin = Float.parseFloat(list[16]);
        float accXmax = Float.parseFloat(list[17]);
        comX = 0.99 * comX + 0.01 * Float.parseFloat(list[1]);
        comY = 0.99 * comY + 0.01 * Float.parseFloat(list[2]);
        comZ = 0.99 * comZ + 0.01 * Float.parseFloat(list[3]);
        comMag = 0.99 * comMag + 0.01 * sqrt(sq(comX) + sq(comY) + sq(comZ)); 
        if (millis() >= nextComPlotTime){
          //nextComPlotTime = millis() + 1000;
          plotComX.addPlotData(comX);
          plotComY.addPlotData(comY);
          plotComZ.addPlotData(comZ);
          /*if (abs(accXmax) > abs(accXmin))
            addPlotData(dataAccX, accXmax);
          else 
            addPlotData(dataAccX, accXmin);*/
          plotAccX.addPlotData(accX);
          plotAccY.addPlotData(accY);
          plotAccZ.addPlotData(accZ);
          plotComMag.addPlotData(comMag);
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
      FileInputStream fin= new FileInputStream (sketchPath() + "\\data\\" + EEPROM_FILENAME);
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
  

  
  public void logging(String data) {    
    if (logState == LOG_OFF){    
      if ((logFile != "") && (logFile != null)){
        File afile = new File(sketchPath() + "\\data\\" + logFile);        
        if (afile.exists()){
          println("playing log from "+afile.getAbsolutePath());
          logState = LOG_PLAY;
          logInput = createReader(afile);
        } else {
          println("recording log to "+afile.getAbsolutePath());
          logState = LOG_REC;
          logOutput = createWriter(afile);
        }
        return;
      }      
    }    
    if (logState == LOG_PLAY){
       if (logInput == null) return;       
       String line = "";
       try {
         line = logInput.readLine();
         processDataReceived(line);
       } catch (IOException e) {
         e.printStackTrace();
         line = null;
       }
       if (line == null) {
         println("logging: EOF");
         // Stop reading because of an error or file is empty  
         try {
           logInput.close();
         } catch (IOException e){
           e.printStackTrace();
         }
         logInput = null;
       }
    }     
    if (logState == LOG_REC){
      if (data == null) return;
      logOutput.print(data);
      logOutput.flush(); // Writes the remaining data to the file
      //logOutput.close(); 
    }
  }

}

    
    