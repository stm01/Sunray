// utilities

#ifndef HELPER_H
#define HELPER_H


typedef union {
  float floatingPoint;   
  unsigned long ulong;
  byte binary[4];
} binaryLongOrFloat;

typedef union {
  unsigned int uint;
  byte binary[4];  
} binaryInt;


float scalePI(float v);
float scale180(float v);
float distancePI(float x, float w);
float distance180(float x, float w);
float fusionPI(float w, float a, float b);
float scalePIangles(float setAngle, float currAngle);
float distance(float x1, float y1, float x2, float y2);

int sign(float x);

void printFloat(float v);
void printLong(unsigned long v);
void printInt(unsigned int v);

uint32_t serialToLong(HardwareSerial *serial);
float serialToFloat(HardwareSerial *serial);

float parseFloatValue(String s, String key);

int freeRam ();

/*
 * Returns random number in normal distribution centering on 0.
 * ~95% of numbers returned should fall between -2 and 2
 */
float gaussRandom();

/*
 * Returns member of set with a given mean and standard deviation
 * mean: mean
 * standard deviation: std_dev
 */
float gauss(float mean, float std_dev);

// calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float gaussian(float mu, float sigma, float x);


// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float voltageDividerUges(float R1, float R2, float U2);

// ADC-value to voltage
float ADC2voltage(float ADCvalue);



#endif


