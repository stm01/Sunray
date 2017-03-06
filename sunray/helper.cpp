#include <Arduino.h>
#include "helper.h"
#include "config.h"

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

// rescale to -180..+180
float scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180) return (-2*180+d);
  else if (d < -180) return (2*180+d);
  else return d;
}


// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;
  return d;
}

float distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2*180;
  else if (d > 180) d = d - 2*180;
  return d;
}


// weight fusion (w=0..1) of two radiant values (a,b)
float fusionPI(float w, float a, float b)
{
  float c;
  if ((b >= PI/2) && (a <= -PI/2)){
    c = w * a + (1.0-w) * (b-2*PI);
  } else if ((b <= -PI/2) && (a >= PI/2)){
    c = w * (a-2*PI) + (1.0-w) * b;
  } else c = w * a + (1.0-w) * b;
  return scalePI(c);
}


// scale setangle, so that both PI angles have the same sign
float scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

float distance(float x1, float y1, float x2, float y2){
  return sqrtf( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

int sign(float x) { 
  return x<0 ? -1 : 1; 
}


void printFloat(float v){
  binaryLongOrFloat b;
  b.floatingPoint = v;  
  ROBOTMSG.write(b.binary[3]);  
  ROBOTMSG.write(b.binary[2]);  
  ROBOTMSG.write(b.binary[1]);  
  ROBOTMSG.write(b.binary[0]);  
}

void printLong(unsigned long v){
  binaryLongOrFloat b;
  b.ulong = v; 
  ROBOTMSG.write(b.binary[3]);  
  ROBOTMSG.write(b.binary[2]);  
  ROBOTMSG.write(b.binary[1]);  
  ROBOTMSG.write(b.binary[0]);  
}

void printInt(unsigned int v){
  binaryInt b;
  b.uint = v; 
  ROBOTMSG.write(b.binary[1]);  
  ROBOTMSG.write(b.binary[0]);    
}

uint32_t serialToLong(HardwareSerial* serial){
  binaryLongOrFloat b;
  b.binary[3] = serial->read();
  b.binary[2] = serial->read();
  b.binary[1] = serial->read();
  b.binary[0] = serial->read();
  return b.ulong;
}

float serialToFloat(HardwareSerial* serial){
  binaryLongOrFloat b;
  b.binary[3] = serial->read();
  b.binary[2] = serial->read();
  b.binary[1] = serial->read();
  b.binary[0] = serial->read();
  return b.floatingPoint;
}

int freeRam () {
#ifdef __AVR__
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
#else
  return 0;
#endif
}



/*
 * Returns random number in normal distribution centering on 0.
 * ~95% of numbers returned should fall between -2 and 2
 */
float gaussRandom() {
  return 2*((float)rand())/((float)RAND_MAX)-1;
  
  /*static unsigned int rnd = 0;
  rnd += micros(); // seeded with changing number
  rnd ^= rnd << 2; rnd ^= rnd >> 7; rnd ^= rnd << 7;
  return ( 2 * (((float)rnd) / 65535.0) - 1.0 );*/
    

    //printf("random=%3.3f\n", random());
    /*float u = 2*random()-1;
    float v = 2*random()-1;
    float r = u*u + v*v;
    // if outside interval [0,1] start over
    if ((r == 0) || (r > 1)) return gaussRandom();

    float c = sqrt(-2*log(r)/r);
    return u*c;*/

    /* todo: optimize this algorithm by caching (v*c)
     * and returning next time gaussRandom() is called.
     * left out for simplicity */
}

/*
 * Returns member of set with a given mean and standard deviation
 * mean: mean
 * standard deviation: std_dev
 */
float gauss(float mean, float std_dev){
  return mean + (gaussRandom()*std_dev);
}

// calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float gaussian(float mu, float sigma, float x)
{
  return exp(-   pow(mu - x, 2) / pow(sigma,  2) / 2.0     )
              / sqrt(2.0 * M_PI * pow(sigma, 2));
}


  
