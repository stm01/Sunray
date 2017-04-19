import java.io.*;


class Map  {
  
  public static final String MAP_FILENAME = "outdoor_map.bin";
  public static final int MAP_SIZE_X = 50;
  public static final int MAP_SIZE_Y = 50;
  
  public static final int OUTLINE_SIZE = 500;
  public static final int PARTICLES = 5000;
  
  public static final int MAP_DATA_SIGNAL_MAX = 31;
  
  public static final int DRAW_WIDTH = 300;
  public static final int DRAW_HEIGHT = 300;
  
  public static final int DRAW_X = 10; 
  public static final int DRAW_Y = 50;
  
  public static final int perimeterWireLengthMeter = 15;
  public static final float steeringNoise = 0.5;
  public static final float distanceNoise = 0.15;
  //public static final float measurementNoise = 0.01;
  
  public class RobotState   {
    public float x;
    public float y;
    public float orientation;
  }
    
  public float mapScaleX = 0;
  public float mapScaleY = 0;
  public float overallProb = 0;
  public float smoothOverallProb = 0;
  public float currMapX = 0;
  public float currMapY = 0;
  public float particlesDistanceX = 0;
  public float particlesDistanceY = 0;
  public float particlesDistance = 0;
  public boolean hasLocalized = false;
  public float overallDist = 0;  
  
  public boolean stateMapping = false;
  public boolean stateLocalize = true;  
  public boolean stateLocalizeOutline = false;
  public boolean stateMowing = false;
  
          
  public MapData[][] mapData = new MapData[MAP_SIZE_Y][MAP_SIZE_X];  
  public ArrayList<PVector> outlineList = new ArrayList<PVector>();
  public RobotState[] particles = new RobotState[PARTICLES]; 
  public RobotState robotState = new RobotState();
  public RobotState particlesState = new RobotState();
  protected float distanceSum = 0;
    
  
  Map(){
    println("memory: map="+MAP_SIZE_X*MAP_SIZE_Y+"  outline="+OUTLINE_SIZE*6+ "  particles="+PARTICLES*6);
    
    for (int i=0; i < PARTICLES; i++) particles[i] = new RobotState();
    for (int y=0; y < MAP_SIZE_Y; y++){
      for (int x=0; x < MAP_SIZE_X; x++){                    
        mapData[y][x]=new MapData();;
      }
    }
    clearOutline();    
    
    /*exampleOutline();
    correctOutline();
    transferOutlineToMap();
    save();*/
    
    load();            
    transferOutlineToMap();    
    
    distributeParticlesOutline();   
  }
  
  public void startMapping(){
    println("startMapping");
    mapScaleX = 2;
    mapScaleY = 2;
    clearOutline();
    stateMapping=true;
    stateLocalize=false;
  }  
    
  public void stopMapping(){
    println("stopMapping");
    correctOutline();
    transferOutlineToMap();
    save();
    stateMapping = false;
  }
  
  public void run(float course, float distance, float leftMag, float rightMag){    
    distanceSum += distance;
    overallDist += distance;       
    //if (abs(distanceSum) < 0.001) return;
  
    robotMotion(course, distanceSum);    
    if (stateMapping){
      float delta = sqrt( sq(currMapX-robotState.x) + sq(currMapY-robotState.y) ) ;
      if ( delta >= perimeterWireLengthMeter/OUTLINE_SIZE ){     
        float startDist = distanceToStart(robotState.x, robotState.y);
        //Console.println(startDist);
        if (startDist < 0.3) {
          stopMapping();          
        } else { 
          PVector pt = new PVector();
          pt.x = robotState.x;
          pt.y = robotState.y;
          outlineList.add(pt);
        }
        currMapX = robotState.x;
        currMapY = robotState.y;                
      }
    } 
    if (stateLocalize) {    
      particlesMotion(course, distanceSum);
      sense(leftMag, rightMag);
      computeParticlesState();     
      if (particlesDistance<6) hasLocalized = true;            
      if (hasLocalized){
        if ( smoothOverallProb > 0.5 ){ 
          robotState.x = particlesState.x;
          robotState.y = particlesState.y;
        } else{
          //if (stateLocalizeOutline) distributeParticlesOutline();
          if ((leftMag > 0) || (rightMag > 0)) distributeParticlesOutline();
          hasLocalized = false;
        }
      }
      if (stateMowing){
        MapData md = getMapDataMeter(robotState.x, robotState.y);
        md.mowed = true;        
      }
    }
    distanceSum=0;    
  }
  
  public void robotMotion(float course, float distance){
    robotState.orientation = course;
    robotState.x += distance * cos(course);
    robotState.y += distance * sin(course);    
  }
  
  public void particlesMotion(float course, float distance){  
    for (int i=0; i < PARTICLES; i++){
      float particleCourse = gauss(course, steeringNoise); // steering noise
      float particleDistance = gauss(distance, distanceNoise); // distance noise
      particles[i].orientation = particleCourse;
      particles[i].x += particleDistance * cos(particleCourse);
      particles[i].y += particleDistance * sin(particleCourse);
    }
  }  
  
  public void draw(){
    int w = DRAW_WIDTH;
    int h = DRAW_HEIGHT;
    int px = DRAW_X;
    int py = DRAW_Y;
    stroke(255, 0, 0);
    strokeWeight(0);
    fill(200, 200, 200);
    rect(px, py, w, h); 
    drawOutline();
    drawMap();
    drawParticles();    
    drawRobotPos();
  }
  
  public void drawParticles(){
    int px = DRAW_X;
    int py = DRAW_Y;
    float cx;
    float cy;
    fill(200,200,0);
    strokeWeight(0);    
    int stepx = Math.round(  ((float)DRAW_WIDTH) / ((float)MAP_SIZE_X) );
    int stepy = Math.round( ((float)DRAW_HEIGHT) / ((float)MAP_SIZE_Y) );  
    for (int i=0; i < PARTICLES; i++){
      cx = particles[i].x;
      cy = particles[i].y;
      //println(cx+","+cy);
      ellipse(px + Math.round(cx*mapScaleX*stepx), py + DRAW_HEIGHT-Math.round(cy*mapScaleY*stepy), 10, 10);
    }
  }
  
  public void drawMap(){
    int px = DRAW_X;
    int py = DRAW_Y;    
    int stepx = Math.round(  ((float)DRAW_WIDTH) / ((float)MAP_SIZE_X) );
    int stepy = Math.round( ((float)DRAW_HEIGHT) / ((float)MAP_SIZE_Y) );
    stroke(255,255,255);
    strokeWeight(0);
    int r,g,b,col;
    for (int y=0; y < MAP_SIZE_X; y++){
      for (int x=0; x < MAP_SIZE_Y; x++){      
        MapData d = mapData[y][x];            
        if (d.mowed){
           r=0; g=255; b=0;      
        } else {
          col = Math.round( 255.0*1.0/((float)(32-d.signal)) );         
          if (!d.inside) {
            r=255; g=255-col; b=255-col;
            //r=255; g=0; b=0;
          } else {
            r=255-col; g=255-col; b=255;
            //r=0; g=0; b=255;
          }
        }    
        fill(r, g, b);
        rect(px+ x*stepx, py+ y*stepy+1, stepx, stepy);      
        //set(x, y, v);      
      }
    }    
  }
  
  public void drawOutline(){    
    int px = DRAW_X;
    int py = DRAW_Y;
    int dh = DRAW_HEIGHT;
    int dw = DRAW_WIDTH;
    int mw = MAP_SIZE_X;
    int mh = MAP_SIZE_Y;
    if (stateMapping){
      px = 600;
      py = 0;
    }        
    stroke(200,80,0);
    strokeWeight(3);
    if (outlineList.size() < 3) return;    
    int stepx = Math.round(  ((float)dw) / ((float)mw) );
    int stepy = Math.round( ((float)dh) / ((float)mh) );      
    PVector ptLast = null;    
    for (int i=0; i < outlineList.size(); i++){
      PVector pt = outlineList.get(i);
      if (i > 0){                       
        line(px + ptLast.x * mapScaleX*stepx, py + dh - ptLast.y * mapScaleY*stepy, 
             px + pt.x     * mapScaleX*stepx, py + dh - pt.y     * mapScaleY*stepy);
      }
      ptLast = pt;
    }
  }
  
  public boolean isInsideWindow(int px, int py){
    return (    (px >= DRAW_X) && (px < DRAW_X+DRAW_WIDTH) 
             && (py >= DRAW_Y) && (py < DRAW_Y+DRAW_HEIGHT)  );       
  }
  
  public void setRobotPositionManual(int px, int py){
    px -= DRAW_X;
    py -= DRAW_Y;
    int stepx = Math.round( ((float)DRAW_WIDTH) / ((float)MAP_SIZE_X) );
    int stepy = Math.round( ((float)DRAW_HEIGHT) / ((float)MAP_SIZE_Y) );
    //int cx = px + Math.round(robotState.x*mapScaleX*stepx); 
    //int cy = py + DRAW_HEIGHT-Math.round(robotState.y*mapScaleY*stepy);
    float x = px/(mapScaleX*stepx);
    float y = (DRAW_HEIGHT-py)/(mapScaleY*stepy);
    for (int i=0; i < PARTICLES; i++){
      particles[i].x=x;
      particles[i].y=y;
    }
    robotState.x = x;
    robotState.y = y;
  }
 
  
  public void drawRobotPos(){  
    int px = DRAW_X;
    int py = DRAW_Y;
    int dh = DRAW_HEIGHT;
    int dw = DRAW_WIDTH;
    int mw = MAP_SIZE_X;
    int mh = MAP_SIZE_Y;
    if (stateMapping){
      px = 600;
      py = 0;
    }        
    pushMatrix();
    translate(0,0,1);  
    strokeWeight(2);
    stroke(255,255,255);
    if (hasLocalized)
      fill(0,150,0);
    else
      fill(255,0,0);
    int stepx = Math.round( ((float)dw) / ((float)mw) );
    int stepy = Math.round( ((float)dh) / ((float)mh) );  
    int cx = px + Math.round(robotState.x*mapScaleX*stepx); 
    int cy = py + dh-Math.round(robotState.y*mapScaleY*stepy);
    ellipse(cx, cy, 20, 20);    
    stroke(0,0,0);        
    line(cx, cy, cx + Math.round(15*cos(robot.angleRadSet)), cy - Math.round(15*sin(robot.angleRadSet)) );    
    stroke(255,255,255);    
    line(cx, cy, cx + Math.round(10*cos(robotState.orientation)), cy - Math.round(10*sin(robotState.orientation)) );    
    popMatrix();
  }  
  
  public float distanceToParticles(float x, float y){  
    float res = 99999;
    for (int i=0; i < PARTICLES; i++){
      float dist = sqrt( sq(x-particles[i].x) + sq(y-particles[i].y) );
      res = min(res, dist);
    }
    return res;
  }

  public float distanceToStart(float x, float y){  
    float res = 99999;
    if (outlineList.size() < OUTLINE_SIZE*0.2) return res;
    for (int i=0; i < 10; i++){
     float dist = sqrt( sq(x-outlineList.get(i).x) + sq(y-outlineList.get(i).y) );
      res = min(res, dist);
    }
    return res;
  }  
  
  public void clearOutline(){
    println("clearOutline");
    outlineList.clear();
    clearMap();
    robotState.x=0;
    robotState.y=0;
    currMapX = 0;
    currMapY = 0;
  }
  
  public void exampleOutline(){
    for (int i=0; i < OUTLINE_SIZE; i++){
      PVector pt1 = new PVector();
      pt1.x = cos(((float)i)/(OUTLINE_SIZE)*1.6*PI)*20.0;
      pt1.y = sin(((float)i)/(OUTLINE_SIZE)*1.6*PI)*20.0;
      outlineList.add(pt1);
    }
  }
  
  public void save() {
    File afile = new File(sketchPath() + "\\data\\" + MAP_FILENAME);
    println("saving map to " + afile.getAbsolutePath());
    try{
      FileOutputStream fout= new FileOutputStream (afile);
      ObjectOutputStream oos = new ObjectOutputStream(fout);      
      oos.writeObject(outlineList);
      oos.writeObject(mapData);
      oos.writeObject(mapScaleX);
      oos.writeObject(mapScaleY);
      fout.close();
    } catch (Exception e){
      //e.printStackTrace();
      println("ERROR saving map file");
    }    
  }
  
  public void load() {    
    File afile = new File(sketchPath() + "\\data\\" + MAP_FILENAME);
    println("loading map from "+afile.getAbsolutePath());
    try{
      FileInputStream fin= new FileInputStream (afile);
      ObjectInputStream ois = new ObjectInputStream(fin);
      outlineList = (ArrayList)ois.readObject();      
      mapData = (MapData[][])ois.readObject();
      mapScaleX = (float)ois.readObject();
      mapScaleY = (float)ois.readObject();
      fin.close();
    } catch (Exception e){
      //e.printStackTrace();
      println("ERROR loading map file");
    }
  }
  
  public void correctOutline(){
    println("correctOutline");
    if (outlineList.size() == 0) return;    
    for (int i=outlineList.size()-1; i > 0; i--){
      // determine error
      float errx=outlineList.get(outlineList.size()-1).x-outlineList.get(0).x;
      float erry=outlineList.get(outlineList.size()-1).y-outlineList.get(0).y;
      float dist = sqrt( sq(errx) + sq(erry) );
      if (dist < 0.01) break;
      // determine piece distance
      float diffX = outlineList.get(i).x-outlineList.get(i-1).x;
      float diffY = outlineList.get(i).y-outlineList.get(i-1).y;
      if (  (Math.signum(errx) == Math.signum(diffX)) && (Math.signum(erry) == Math.signum(diffY))
          && (abs(diffX) < abs(errx))
          && (abs(diffY) < abs(erry))     )
      {
        // outline piece helps us to reduce error => remove piece
        for (int j=i; j < outlineList.size(); j++){
          outlineList.get(j).x-=diffX;
          outlineList.get(j).y-=diffY;
        }      
      }
    }
    // connect end and start
    PVector pt = new PVector();
    pt.x = outlineList.get(0).x;
    pt.y = outlineList.get(0).y;
    outlineList.add(pt);    
  }
    

 
  void setMapData(int xp, int yp, MapData value){
    if ((xp >= MAP_SIZE_X) || (xp < 0)) return;
    if ((yp >= MAP_SIZE_Y) || (yp < 0)) return;
    mapData[MAP_SIZE_Y-1-yp][xp]=value;
  }

  boolean isXYOnMap(int x, int y){
    if ((x >= MAP_SIZE_X) || (x < 0)) return false;
    if ((y >= MAP_SIZE_Y) || (y < 0)) return false;
    return true;
  }


  void setMapDataMeter(float x, float y, MapData value, int thickness){
    int xp = ((int)(x*mapScaleX));
    int yp = ((int)(y*mapScaleY));
    setMapData(xp, yp, value);
    if (thickness > 1) {
      setMapData(xp+1, yp, value);
      setMapData(xp, yp+1, value);
    }
  }


  boolean isXYOnMapMeter(float x, float y){
    int xp = ((int)(x*mapScaleX));
    int yp = ((int)(y*mapScaleY));
    if ((xp >= MAP_SIZE_X) || (xp < 0)) return false;
    if ((yp >= MAP_SIZE_Y) || (yp < 0)) return false;
    return true;
  }
      
  MapData getMapDataMeter(float x, float y){
    MapData md = new MapData();
    md.inside = false;
    int xp = ((int)(x*mapScaleX));
    int yp = ((int)(y*mapScaleY));
    if ((xp >= MAP_SIZE_X) || (xp < 0)) return md;
    if ((yp >= MAP_SIZE_Y) || (yp < 0)) return md;
    return mapData[MAP_SIZE_Y-1-yp][xp];
  }
  
  void clearMap(){
    println("clearMap");
    for (int y=0; y < MAP_SIZE_Y; y++){
      for (int x=0; x < MAP_SIZE_X; x++){                   
        mapData[y][x].signal=0;
        mapData[y][x].mowed=false;
        mapData[y][x].inside=false;        
      }
    }
  }

 
  void transferOutlineToMap(){
    println("transferOutlineToMap");
    clearMap();       
    float minX = 9999;
    float maxX =  -9999;
    float minY = 9999;
    float maxY =  -9999;
    for (int i=0; i < outlineList.size(); i++){
      float x = outlineList.get(i).x;
      float y = outlineList.get(i).y;
      minX = min(minX, x);
      maxX = max(maxX, x);
      minY = min(minY, y);
      maxY = max(maxY, y);
    }
    for (int i=0; i < outlineList.size(); i++){
      outlineList.get(i).x-=minX;
      outlineList.get(i).y-=minY;
    }
    float deltaX = abs(maxX - minX);
    float deltaY = abs(maxY - minY);
    float delta = max(deltaX, deltaY);
    mapScaleX = ((float)MAP_SIZE_X-1) / delta;
    mapScaleY = ((float)MAP_SIZE_Y-1) / delta;
    println("mapScale="+mapScaleX+", "+mapScaleY);
    robotState.x -= minX;
    robotState.y -= minY;
    float meterPerPixel = min(1/mapScaleX, 1/mapScaleY);
    for (int i=1; i < outlineList.size(); i++){
      float wx = (outlineList.get(i).x - outlineList.get(i-1).x);
      float wy = (outlineList.get(i).y - outlineList.get(i-1).y);
      int steps = max(1, max(((int)(abs(wx) / meterPerPixel)), ((int)(abs(wy) / meterPerPixel))))*2;
      float stepx = wx / ((float)steps);
      float stepy = wy / ((float)steps);
      float x = outlineList.get(i-1).x;
      float y = outlineList.get(i-1).y;
      for (int j=0; j < steps; j++){
        MapData md = getMapDataMeter(x, y);
        md.signal = MAP_DATA_SIGNAL_MAX;
        md.inside = false;
        setMapDataMeter(x, y, md, 1);
        x+=stepx;
        y+=stepy;
      }
    }  
    println("resetMapData");  
    //resetMapDataOutside(0, 0);  
    resetMapDataInside(Math.round(MAP_SIZE_X*0.5), Math.round(MAP_SIZE_Y*0.5)); // set inside state, remaining is outside (FIXME: will not work if perimeter or outside at that pixel)
    for (int y=0; y < MAP_SIZE_Y; y++){
      for (int x=0; x < MAP_SIZE_X; x++){
        MapData md = mapData[y][x];
        if (md.signal == MAP_DATA_SIGNAL_MAX){ // for each perimeter pixel (inside/outside)
          md.signal = 0;
          mapData[y][x] = md;
          resetMapDataSignal(x,y,MAP_DATA_SIGNAL_MAX);  // start "painting" from here
        }
      }
    }
  }

  void resetMapDataOutside(int x, int y){
    if (!isXYOnMap(x,y)) return;
    MapData md = mapData[y][x];
    if (md.signal == MAP_DATA_SIGNAL_MAX) return; // hit perimeter
    if (!md.inside) return; // already outside
    md.inside = false;
    mapData[y][x] = md;
    resetMapDataOutside(x-1,y);
    resetMapDataOutside(x+1,y);
    resetMapDataOutside(x,y-1);
    resetMapDataOutside(x,y+1);
  }

  void resetMapDataInside(int x, int y){
    if (!isXYOnMap(x,y)) return;
    MapData md = mapData[y][x];
    if (md.signal == MAP_DATA_SIGNAL_MAX) return; // hit perimeter
    if (md.inside) return; // already visited
    md.inside = true;
    md.mowed = false;
    mapData[y][x] = md;
    resetMapDataInside(x-1,y);
    resetMapDataInside(x+1,y);
    resetMapDataInside(x,y-1);
    resetMapDataInside(x,y+1);
  }

  void resetMapDataSignal(int x, int y, int signalStrength){
    if (!isXYOnMap(x,y)) return;
    MapData md = mapData[y][x];
    if (md.signal >= signalStrength) return; // signal already higher or equal (visited)
    md.signal =  signalStrength;
    mapData[y][x] = md;
    int strength = max(0,signalStrength-1);
    resetMapDataSignal(x-1,y,strength);
    resetMapDataSignal(x+1,y,strength);
    resetMapDataSignal(x,y-1,strength);
    resetMapDataSignal(x,y+1,strength);
  }
  
  /*
   * Returns random number in normal distribution centering on 0.
   * ~95% of numbers returned should fall between -2 and 2
   */
  float gaussRandom() {
    //return 2.0*random(1.0)-1.0;
    float u = 2*random(1.0)-1;
    float v = 2*random(1.0)-1;
    float r = u*u + v*v;
    // if outside interval [0,1] start over
    if ((r == 0) || (r > 1)) return gaussRandom();

    float c = sqrt(-2*log(r)/r);
    return u*c;    
  }
  

  /*
   * Returns member of set with a given mean and standard deviation
   * mean: mean
   * standard deviation: std_dev
   */
  float gauss(float mean, float std_dev){
    return mean + (gaussRandom()*std_dev);
  }

 
  void computeParticlesState(){
    float maxx = -9999;
    float minx = 9999;
    float maxy = -9999;
    float miny = 9999;
    particlesState.x=0;
    particlesState.y=0;
    //particlesState.orientation=0;  
    for (int i=0; i < PARTICLES; i++){
      particlesState.x += particles[i].x;
      particlesState.y += particles[i].y;
      // orientation is tricky because it is cyclic. By normalizing
      // around the first particle we are somewhat more robust to
      // the 0=2pi problem
      /*particlesState.orientation += ( fmod((particles[i].orientation
                    - particles[0].orientation + M_PI) , (2.0 * M_PI))
                    + particles[0].orientation - M_PI);*/
      maxx = max(maxx, particles[i].x);
      maxy = max(maxy, particles[i].y);
      minx = min(minx, particles[i].x);
      miny = min(miny, particles[i].y);
    }
    particlesState.x/=((float)PARTICLES);
    particlesState.y/=((float)PARTICLES);
    //particlesState.orientation/=((float)PARTICLES);
    particlesDistanceX = maxx-minx;
    particlesDistanceY = maxy-miny;
    particlesDistance = sqrt(sq(particlesDistanceX)+sq(particlesDistanceY));
  }
  
  void distributeParticles(){
    float x;
    float y;
    println("distributeParticles");
    //if (perimeterOutlineSize == 0) return;
    //float minDist = ((float)perimeterWireLengthMeter)/((float)PARTICLES)*0.7;
    for (int i=0; i < PARTICLES; i++){
      x= random(1.0) * ((float)MAP_SIZE_X)/mapScaleX;
      y= random(1.0) * ((float)MAP_SIZE_Y)/mapScaleY;        
      particles[i].x = x;
      particles[i].y = y;
      //particles[i].orientation = robotState.orientation;
      //particles[i].orientation += gauss(0.0, STEERING_NOISE);
    }
  }
 
  void distributeParticlesOutline(){
    float x;
    float y;
    println("distributeParticlesOutline");
    if (outlineList.size() == 0) return;
    //float minDist = ((float)perimeterWireLengthMeter)/((float)PARTICLES)*0.7;
    for (int i=0; i < PARTICLES; i++){
      while (true){
        x= random(1.0) * ((float)MAP_SIZE_X)/mapScaleX;
        y= random(1.0) * ((float)MAP_SIZE_Y)/mapScaleY;
        MapData md = getMapDataMeter(x,y);
        if ( (md.signal == MAP_DATA_SIGNAL_MAX) 
         //&& (distanceToParticles(x,y) > minDist) 
        ) break;
      }
      particles[i].x = x;
      particles[i].y = y;
      //particles[i].orientation = robotState.orientation;
      //particles[i].orientation += gauss(0.0, STEERING_NOISE);
    }
  }
  
  //  computes the probability of a particle
  float measurementProb(int particleIdx, float leftMag, float rightMag){
    // calculate Gaussian
    // gaussian(mu, sigma, x)
    //prob = gaussian(sim.world.getBfield(x, y), measurement_noise, measurement);
    float prob = 1.0;
    RobotState particle = particles[particleIdx];
    if (!isXYOnMapMeter(particle.x, particle.y)) return 0;
    MapData md = getMapDataMeter(particle.x, particle.y);
    //float strength = 1.0 / ((float)(32-md.s.signal));
    if (stateLocalizeOutline){
      // tracking perimeter
      //prob = gaussian(strength, measurementNoise, 1.0);    
      if (md.signal < MAP_DATA_SIGNAL_MAX-1) return 0;
      //return ((float)md.s.signal) / ((float)MAP_DATA_SIGNAL_MAX);
    } else if (stateLocalize) {
      // mowing
      if (!md.inside) {
        if ((leftMag < 0) && (rightMag < 0)) return 0;        
      } else {
        if ((leftMag > 0) || (rightMag >0)) return 0;
        //  prob = gaussian(strength*700, measurementNoise, measurement);
      }
    }
    return prob;  
  }

  // sensing and resampling
  // http://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/resampling_schemes/
  void sense(float leftMag, float rightMag){  
    overallProb = 0;
    for (int i=0; i < PARTICLES; i++){
      float measurement_prob1 = measurementProb(i, leftMag, rightMag);
      overallProb += measurement_prob1 / ((float)PARTICLES);
      int idx = int(random(PARTICLES));
      float measurement_prob2 = measurementProb(idx, leftMag, rightMag);    
      //float rnd = ((float)rand())/((float)RAND_MAX);
      if (measurement_prob1 > measurement_prob2){
        //if (rnd > 0.5) 
        particles[idx].x = particles[i].x;
        particles[idx].y = particles[i].y;
      } 
      else if (measurement_prob2 >=  measurement_prob1) {      
        //if (rnd > 0.5) 
        particles[i].x = particles[idx].x;
        particles[i].y = particles[idx].y;
      }
    }
    smoothOverallProb = 0.9 * smoothOverallProb + 0.1 * overallProb;
  }



/*void drawParticlesFromString(int px, int py, String data){
  float cx;
  float cy;
  fill(200,200,0);
  strokeWeight(0);
  String[] list = split(data, ',');
  int stepx = Math.round(  ((float)bitmapw) / ((float)bitmapDataWidth) );
  int stepy = Math.round( ((float)bitmaph) / ((float)bitmapDataHeight) );  
  for (int i=1; i < list.length-1; i+=2){
    cx = Float.parseFloat(list[i]);
    cy = Float.parseFloat(list[i+1]);
    ellipse(px + Math.round(cx*mapScaleX*stepx), py + bitmaph-Math.round(cy*mapScaleY*stepy), 10, 10);
  }
}

void drawBitmapFromString(int px, int py, String data){  
  String[] a = split(data, ',');  
  mapScaleX = Float.parseFloat(a[1]);
  mapScaleY = Float.parseFloat(a[2]);
  bitmapDataWidth = Integer.parseInt(a[3]); 
  bitmapDataHeight = Integer.parseInt(a[4]);
  int stepx = Math.round(  ((float)bitmapw) / ((float)bitmapDataWidth) );
  int stepy = Math.round( ((float)bitmaph) / ((float)bitmapDataHeight) );
  //println("bitmap "+w+","+h);
  int idx = 5;
  if (a.length < 5 + bitmapDataWidth*bitmapDataHeight*3){
    println("corrupt bitmap");
    return;
  }
  stroke(255,255,255);
  strokeWeight(0);
  for (int y=0; y < bitmapDataHeight; y++){
    for (int x=0; x < bitmapDataWidth; x++){      
      int r = int(a[idx]); idx++;
      int g = int(a[idx]); idx++;
      int b = int(a[idx]); idx++;         
      fill(r, g, b);
      rect(px+ x*stepx, py+ y*stepy+1, stepx, stepy);      
      //set(x, y, v);      
    }
  }  
}

void drawOutlineFromString(int px, int py, String data){
  stroke(0,255,0);
  strokeWeight(2);
  String[] list = split(data, ',');
  if (list.length < 3) return;
  float lastX =0;
  float lastY =0;
  for (int i=1; i < list.length-1; i+=2){
    float x = px+ Float.parseFloat(list[i])  *50;
    float y = py- Float.parseFloat(list[i+1])  *50;
    if (i > 1){
      line(lastX, lastY, x, y);
    }
    lastX = x;
    lastY = y;
  }
}*/




 
  
}
  

  