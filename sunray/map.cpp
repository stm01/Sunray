#include "map.h"
#include "robot.h"
#include "perimeter.h"
#include "buzzer.h"
#include "motor.h"
#include "helper.h"
#include "config.h"
#include "flashmem.h"
#include "imu.h"

MapClass Map;

#define ADDR 100
#define MAGIC 1


void MapClass::begin()
{
  /*for (int i=0; i < 500; i++){
    robot_state_t particle;
    particles.push_back(particle);
    }*/

  lastMotorLeftTicks = 0;
  lastMotorRightTicks = 0;

  perimeterWireLengthMeter = 15;
  steeringNoise = 0.005;
  distanceNoise = 0.01;
  measurementNoise = 0.01;

  mapValid = false;
  robotState.x = 0;
  robotState.y = 0;
  //robotState.orientation = 0;
  currMapX = 0;
  currMapY = 0;
  mapScaleX = 0;
  mapScaleY = 0;
  nextSampleTime = 0;
  perimeterOutlineSize = 0;
  distAvgSum = 0;
  overallProb = 0;
	verboseOutput = false;

  clearOutline();
  //exampleOutline();
  //correctOutline();
  //transferOutlineToMap();
  loadMap();
  //setParticlesState(robotState.x,robotState.y,robotState.orientation);

  distributeParticlesOutline();
}


void MapClass::setParticlesState(float x, float y, float orientation) {
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    outlineParticles[i].x = x;
    outlineParticles[i].y = y;
    //particles[i].orientation=orientation;
  }
}

void MapClass::robotMotion(float course, float distance) {
  //robotState.orientation = course;
  robotState.x += distance * cos(course);
  robotState.y += distance * sin(course);

  if (Robot.state == STAT_MOW) {
    map_data_t md = getMapDataMeter(robotState.x, robotState.y);
    if (md.s.state == MAP_DATA_STATE_UNMOWED) {
      md.s.state = MAP_DATA_STATE_MOWED;
      setMapDataMeter(robotState.x, robotState.y, md);
    }
  }
}

void MapClass::particlesMotion(float course, float distance) {
  // particles motion
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    float particleCourse = gauss(course, steeringNoise); // steering noise
    float particleDistance = gauss(distance, distanceNoise); // distance noise
    //particles[i].orientation = particleCourse;
    outlineParticles[i].x += particleDistance * cos(particleCourse);
    outlineParticles[i].y += particleDistance * sin(particleCourse);
  }
}

void MapClass::computeParticlesState() {
  float maxx = -9999;
  float minx = 9999;
  float maxy = -9999;
  float miny = 9999;
  particlesState.x = 0;
  particlesState.y = 0;
  //particlesState.orientation=0;
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    particlesState.x += outlineParticles[i].x;
    particlesState.y += outlineParticles[i].y;
    // orientation is tricky because it is cyclic. By normalizing
    // around the first particle we are somewhat more robust to
    // the 0=2pi problem
    /*particlesState.orientation += ( fmod((particles[i].orientation
                    - particles[0].orientation + M_PI) , (2.0 * M_PI))
                    + particles[0].orientation - M_PI);*/
    maxx = max(maxx, outlineParticles[i].x);
    maxy = max(maxy, outlineParticles[i].y);
    minx = min(minx, outlineParticles[i].x);
    miny = min(miny, outlineParticles[i].y);
  }
  particlesState.x /= ((float)OUTLINE_PARTICLES);
  particlesState.y /= ((float)OUTLINE_PARTICLES);
  //particlesState.orientation/=((float)PARTICLES);
  particlesDistanceX = maxx - minx;
  particlesDistanceY = maxy - miny;
}


void MapClass::run() {
  if (fabs(Motor.distanceCmAvg) < 0.01) return;
  distAvgSum += Motor.distanceCmAvg / 100.0;
  //if (fabs(distAvgSum) < 0.001) return;
  float yaw = IMU.getYaw();
  if (verboseOutput){
	  ROBOTMSG.print(F("!17,"));
    ROBOTMSG.print(distAvgSum, 4);
    ROBOTMSG.print(F(","));
		ROBOTMSG.println(yaw, 4);	
	}
	
  /*
    robotMotion(yaw, distAvgSum);

    if ((Robot.state == STAT_CREATE_MAP) && (Robot.trackState == TRK_RUN)) {
    float dist = sqrt( sq(currMapX-robotState.x) + sq(currMapY-robotState.y) ) ;
    if ( dist >= ((float)perimeterWireLengthMeter)/((float)OUTLINE_PARTICLES) ){
      if (perimeterOutlineSize < OUTLINE_PARTICLES-1){
        robot_state_t pt;
        pt.x = robotState.x;
        pt.y = robotState.y;
        outlineParticles[perimeterOutlineSize]=pt;
      	perimeterOutlineSize++;
        //updateMap();
        currMapX = robotState.x;
        currMapY = robotState.y;
      }
    }
    }

    if ((Robot.state != STAT_CREATE_MAP) && (Robot.state != STAT_CAL_GYRO)) {
      particlesMotion(yaw, distAvgSum);
      float leftMag = Perimeter.getMagnitude(IDX_LEFT);
      float rightMag = Perimeter.getMagnitude(IDX_RIGHT);
      sense(leftMag, rightMag);
      if (millis() >= nextSampleTime){
        nextSampleTime = millis() + 1000;
        computeParticlesState();
        robotState.x = particlesState.x;
        robotState.y = particlesState.y;
      }
    }
  */
  distAvgSum = 0;
}


void MapClass::clearOutline() {
  DEBUGLN(F("clearOutline"));
  perimeterOutlineSize = 0;
  currMapX = 0;
  currMapY = 0;
  robotState.x = 0;
  robotState.y = 0;
  //robotState.orientation=0;
}

void MapClass::exampleOutline() {
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    robot_state_t pt1;
    pt1.x = cos(((float)i) / (OUTLINE_PARTICLES) * 1.6 * PI) * 0.5;
    pt1.y = sin(((float)i) / (OUTLINE_PARTICLES) * 1.6 * PI) * 0.5;
    outlineParticles[i] = pt1;
  }
}

float MapClass::distanceToStart(float x, float y) {
  float res = 99999;
  if (perimeterOutlineSize < OUTLINE_PARTICLES * 0.2) return res;
  for (int i = 0; i < 10; i++) {
    float dist = sqrt( sq(x - outlineParticles[i].x) + sq(y - outlineParticles[i].y) );
    res = min(res, dist);
  }
  return res;
}


void MapClass::correctOutline() {
  DEBUGLN(F("correctOutline"));
  int sz = perimeterOutlineSize;
  if (sz == 0) return;
  for (int i = perimeterOutlineSize - 1; i > 0; i--) {
    // determine error
    float errx = outlineParticles[perimeterOutlineSize - 1].x - outlineParticles[0].x;
    float erry = outlineParticles[perimeterOutlineSize - 1].y - outlineParticles[0].y;
    float dist = sqrt( sq(errx) + sq(erry) );
    if (dist < 0.01) break;
    // determine piece distance
    float diffX = outlineParticles[i].x - outlineParticles[i - 1].x;
    float diffY = outlineParticles[i].y - outlineParticles[i - 1].y;
    if (  (sign(errx) == sign(diffX)) && (sign(erry) == sign(diffY))
          && (fabs(diffX) < fabs(errx))
          && (fabs(diffY) < fabs(erry))     )
    {
      // outline piece helps us to reduce error => remove piece
      for (int j = i; j < perimeterOutlineSize; j++) {
        outlineParticles[j].x -= diffX;
        outlineParticles[j].y -= diffY;
      }
    }
  }
  // connect end and start
  robot_state_t pt;
  pt.x = outlineParticles[0].x;
  pt.y = outlineParticles[0].y;
  outlineParticles[perimeterOutlineSize] = pt;
  perimeterOutlineSize++;
}

void MapClass::transferOutlineToMap() {
  DEBUGLN(F("transferOutlineToMap"));
  map_data_t md;
  md.s.signal = 0;
  md.s.state = MAP_DATA_STATE_UNMOWED;
  md.s.side = MAP_DATA_SIDE_OUT;
  memset(mapData, md.v, sizeof mapData);
  float minX = 9999;
  float maxX =  -9999;
  float minY = 9999;
  float maxY =  -9999;

  for (int i = 0; i < perimeterOutlineSize; i++) {
    float x = outlineParticles[i].x;
    float y = outlineParticles[i].y;
    minX = min(minX, x);
    maxX = max(maxX, x);
    minY = min(minY, y);
    maxY = max(maxY, y);
  }
  for (int i = 0; i < perimeterOutlineSize; i++) {
    outlineParticles[i].x -= minX;
    outlineParticles[i].y -= minY;
  }
  float deltaX = fabs(maxX - minX);
  float deltaY = fabs(maxY - minY);
  float delta = max(deltaX, deltaY);
  mapScaleX = ((float)MAP_SIZE_X - 1) / delta;
  mapScaleY = ((float)MAP_SIZE_Y - 1) / delta;
  robotState.x -= minX;
  robotState.y -= minY;
  float meterPerPixel = min(1 / mapScaleX, 1 / mapScaleY);
  for (int i = 1; i < perimeterOutlineSize; i++) {
    float wx = (outlineParticles[i].x - outlineParticles[i - 1].x);
    float wy = (outlineParticles[i].y - outlineParticles[i - 1].y);
    int steps = max(1, max(((int)(fabs(wx) / meterPerPixel)), ((int)(fabs(wy) / meterPerPixel)))) * 2;
    float stepx = wx / ((float)steps);
    float stepy = wy / ((float)steps);
    float x = outlineParticles[i - 1].x;
    float y = outlineParticles[i - 1].y;
    for (int j = 0; j < steps; j++) {
      map_data_t md = getMapDataMeter(x, y);
      md.s.signal = MAP_DATA_SIGNAL_MAX;
      md.s.side == MAP_DATA_SIDE_IN;
      setMapDataMeter(x, y, md, 1);
      x += stepx;
      y += stepy;
    }
  }
  DEBUGLN(F("resetMapData"));
  //resetMapDataOutside(0, 0);
  resetMapDataInside(MAP_SIZE_X * 0.3, MAP_SIZE_Y * 0.3); // set inside state, remaining is outside (FIXME: will not work if perimeter or outside at that pixel)
  for (int y = 0; y < MAP_SIZE_Y; y++) {
    for (int x = 0; x < MAP_SIZE_X; x++) {
      map_data_t md = mapData[y][x];
      if (md.s.signal == MAP_DATA_SIGNAL_MAX) { // for each perimeter pixel (inside/outside)
        md.s.signal = 0;
        mapData[y][x] = md;
        resetMapDataSignal(x, y, MAP_DATA_SIGNAL_MAX); // start "painting" from here
      }
    }
  }
}

void MapClass::resetMapDataOutside(int x, int y) {
  if (!isXYOnMap(x, y)) return;
  map_data_t md = mapData[y][x];
  if (md.s.signal == MAP_DATA_SIGNAL_MAX) return; // hit perimeter
  if (md.s.side == MAP_DATA_SIDE_OUT) return; // already outside
  md.s.side = MAP_DATA_SIDE_OUT;
  mapData[y][x] = md;
  resetMapDataOutside(x - 1, y);
  resetMapDataOutside(x + 1, y);
  resetMapDataOutside(x, y - 1);
  resetMapDataOutside(x, y + 1);
}

void MapClass::resetMapDataInside(int x, int y) {
  if (!isXYOnMap(x, y)) return;
  map_data_t md = mapData[y][x];
  if (md.s.signal == MAP_DATA_SIGNAL_MAX) return; // hit perimeter
  if (md.s.side == MAP_DATA_SIDE_IN) return; // already visited
  md.s.side = MAP_DATA_SIDE_IN;
  md.s.state = MAP_DATA_STATE_UNMOWED;
  mapData[y][x] = md;
  resetMapDataInside(x - 1, y);
  resetMapDataInside(x + 1, y);
  resetMapDataInside(x, y - 1);
  resetMapDataInside(x, y + 1);
}

void MapClass::resetMapDataSignal(int x, int y, int signalStrength) {
  if (!isXYOnMap(x, y)) return;
  map_data_t md = mapData[y][x];
  if (md.s.signal >= signalStrength) return; // signal already higher or equal (visited)
  md.s.signal =  signalStrength;
  mapData[y][x] = md;
  int strength = max(0, signalStrength - 1);
  resetMapDataSignal(x - 1, y, strength);
  resetMapDataSignal(x + 1, y, strength);
  resetMapDataSignal(x, y - 1, strength);
  resetMapDataSignal(x, y + 1, strength);
}


void MapClass::setMapData(int xp, int yp, map_data_t value) {
  if ((xp >= MAP_SIZE_X) || (xp < 0)) return;
  if ((yp >= MAP_SIZE_Y) || (yp < 0)) return;
  mapData[MAP_SIZE_Y - 1 - yp][xp] = value;
}

bool MapClass::isXYOnMap(int x, int y) {
  if ((x >= MAP_SIZE_X) || (x < 0)) return false;
  if ((y >= MAP_SIZE_Y) || (y < 0)) return false;
  return true;
}


void MapClass::setMapDataMeter(float x, float y, map_data_t value, int thickness) {
  int xp = ((int)(x * mapScaleX));
  int yp = ((int)(y * mapScaleY));
  setMapData(xp, yp, value);
  if (thickness > 1) {
    setMapData(xp + 1, yp, value);
    setMapData(xp, yp + 1, value);
  }
}

float MapClass::distanceToParticles(float x, float y) {
  float res = 99999;
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    float dist = sqrt( sq(x - outlineParticles[i].x) + sq(y - outlineParticles[i].y) );
    res = min(res, dist);
  }
  return res;
}

void MapClass::distributeParticlesOutline() {
  float x;
  float y;
  DEBUGLN(F("distributeParticlesOutline"));
  if (!mapValid) return;
  //if (perimeterOutlineSize == 0) return;
  float minDist = ((float)perimeterWireLengthMeter) / ((float)OUTLINE_PARTICLES) * 0.7;
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    while (true) {
      x = ((float)rand()) / ((float)RAND_MAX) * ((float)MAP_SIZE_X) / mapScaleX;
      y = ((float)rand()) / ((float)RAND_MAX) * ((float)MAP_SIZE_Y) / mapScaleY;
      map_data_t md = getMapDataMeter(x, y);
      /*DEBUG(x);
        DEBUG(F(","));
        DEBUG(y);
        DEBUG(F(","));
        DEBUGLN(md.s.signal);      */
      if ( (md.s.signal == MAP_DATA_SIGNAL_MAX)
           //&& (distanceToParticles(x,y) > minDist)
         ) break;
    }
    outlineParticles[i].x = x;
    outlineParticles[i].y = y;
    //particles[i].orientation = robotState.orientation;
    //particles[i].orientation += gauss(0.0, STEERING_NOISE);
  }
}

bool MapClass::isXYOnMapMeter(float x, float y) {
  int xp = ((int)(x * mapScaleX));
  int yp = ((int)(y * mapScaleY));
  if ((xp >= MAP_SIZE_X) || (xp < 0)) return false;
  if ((yp >= MAP_SIZE_Y) || (yp < 0)) return false;
  return true;
}

map_data_t MapClass::getMapDataMeter(float x, float y) {
  map_data_t md;
  md.s.side = MAP_DATA_SIDE_OUT;
  int xp = ((int)(x * mapScaleX));
  int yp = ((int)(y * mapScaleY));
  if ((xp >= MAP_SIZE_X) || (xp < 0)) return md;
  if ((yp >= MAP_SIZE_Y) || (yp < 0)) return md;
  return mapData[MAP_SIZE_Y - 1 - yp][xp];
}

//  computes the probability of a particle
float MapClass::measurementProb(int particleIdx, float leftMag, float rightMag) {
  // calculate Gaussian
  // gaussian(mu, sigma, x)
  //prob = gaussian(sim.world.getBfield(x, y), measurement_noise, measurement);
  float prob = 1.0;
  robot_state_t particle = outlineParticles[particleIdx];
  if (!isXYOnMapMeter(particle.x, particle.y)) return 0;
  map_data_t md = getMapDataMeter(particle.x, particle.y);
  //float strength = 1.0 / ((float)(32-md.s.signal));
  if ((Robot.state == STAT_CREATE_MAP) || (Robot.state == STAT_TRACK)) {
    // tracking perimeter
    //prob = gaussian(strength, measurementNoise, 1.0);
    if (md.s.signal < MAP_DATA_SIGNAL_MAX - 1) return 0;
    //return ((float)md.s.signal) / ((float)MAP_DATA_SIGNAL_MAX);
  } else {
    // mowing
    if (md.s.side == MAP_DATA_SIDE_OUT) {
      if ((leftMag < 0) && (rightMag < 0)) return 0;
    } else {
      if ((leftMag > 0) && (rightMag > 0)) return 0;
      //  prob = gaussian(strength*700, measurementNoise, measurement);
    }
  }
  return prob;
}

// sensing and resampling
// http://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/resampling_schemes/
void MapClass::sense(float leftMag, float rightMag) {
  overallProb = 0;
  for (int i = 0; i < OUTLINE_PARTICLES; i++) {
    float measurement_prob1 = measurementProb(i, leftMag, rightMag);
    overallProb += measurement_prob1 / ((float)OUTLINE_PARTICLES);
    int idx = rand() % OUTLINE_PARTICLES;
    float measurement_prob2 = measurementProb(idx, leftMag, rightMag);
    //float rnd = ((float)rand())/((float)RAND_MAX);
    if (measurement_prob1 > measurement_prob2) {
      //if (rnd > 0.5)
      outlineParticles[idx] = outlineParticles[i];
    }
    else if (measurement_prob2 >  measurement_prob1) {
      //if (rnd > 0.5)
      outlineParticles[i] = outlineParticles[idx];
    }
  }
  /*std::vector<float>w;
    for (int i=0; i < PARTICLES; i++){
    float measurement_prob = measurementProb(i, measurement);
    w.push_back(measurement_prob);
    }
    // resampling
    std::vector<robot_state_t>newParticles;
    int index = floor( ((float)rand())/((float)RAND_MAX) * PARTICLES);
    float beta = 0.0;
    float mw = *std::max_element(w.begin(), w.end());
    for (int i=0; i < PARTICLES; i++){
    //beta += random() * 2.0 * mw;
    beta += 1.0 * mw;
    while (beta > w[index]){
      beta -= w[index];
      index = (index + 1) % PARTICLES;
    }
    newParticles.push_back(particles[index]);
    }
    particles = newParticles;*/
}


void MapClass::loadSaveMap(boolean readflag) {
  int addr = ADDR;
  short magic = MAGIC;
  DEBUG(F("Map EEPROM addr="));
  DEBUG(ADDR);
  eereadwrite(readflag, addr, magic); // magic
  eereadwrite(readflag, addr, mapScaleX);
  eereadwrite(readflag, addr, mapScaleY);
  for (int y = 0; y < MAP_SIZE_Y; y++) {
    for (int x = 0; x < MAP_SIZE_X; x++) {
      eereadwrite(readflag, addr, Map.mapData[y][x]);
    }
  }
  DEBUG(F(" endaddr="));
  DEBUGLN(addr);
}

boolean MapClass::loadMap() {
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    DEBUGLN(F("Map error: no map"));
    return false;
  }
  DEBUGLN(F("Map: found map"));
  loadSaveMap(true);
  mapValid = true;
  return true;
}


void MapClass::saveMap() {
  DEBUGLN(F("saveMap"));
  loadSaveMap(false);
  mapValid = true;
}


