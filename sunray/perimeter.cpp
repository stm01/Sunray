#include "perimeter.h"
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "robot.h"
#include "config.h"

//#define pinLED 13

PerimeterClass Perimeter;


// developer test to be activated in mower.cpp: 
#ifdef USE_DEVELOPER_TEST
  // more motor driver friendly signal (receiver)
  int8_t sigcode_norm[]   = { 1,-1,0,0,0,
                              1,-1,0,0,0,
                             -1, 1,0,0,0,
                              1,-1,0,0,0  };
#else
  // http://grauonline.de/alexwww/ardumower/filter/filter.html    
  // "pseudonoise4_pw" signal
  // if using reconstructed sender signal, use this
  int8_t sigcode_norm[]        = { 1,1,-1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,1,1,-1 };   
  // "pseudonoise4_pw" signal (differential)
  // if using the coil differential signal, use this
  int8_t sigcode_diff[]        = { 1,0,-1, 0,1,-1,1,-1, 0,1,-1,1,0,-1, 0,1,-1, 0,1,-1, 0,1,0,-1 };   
#endif


PerimeterClass::PerimeterClass(){    
  enabled = true;
	useDifferentialPerimeterSignal = true;
  swapCoilPolarity = false;
  timedOutIfBelowSmag = 10;
  timeOutSecIfNotInside = 15;
  callCounter = 0;
  mag[0] = mag[1] = 0;
  smoothMag[0] = smoothMag[1] = 0;
  filterQuality[0] = filterQuality[1] = 0;
  signalCounter[0] = signalCounter[1] = 0;  
  lastInsideTime[0] = lastInsideTime[1] = 0;    
}

void PerimeterClass::begin(byte idx0Pin, byte idx1Pin){
  idxPin[0] = idx0Pin;
  idxPin[1] = idx1Pin;  

  switch (ADCMan.sampleRate){
    case SRATE_9615: subSample = 1; break;
    case SRATE_19231: subSample = 2; break;
    case SRATE_38462: subSample = 4; break;
  }
  
  // use max. 255 samples and multiple of signalsize
  int adcSampleCount = sizeof sigcode_norm * subSample;
  pinMode(idx0Pin, INPUT);
  pinMode(idx1Pin, INPUT);
  ADCMan.setupChannel(idx0Pin, ((int)255 / adcSampleCount) * adcSampleCount, true); 
  ADCMan.setupChannel(idx1Pin, ((int)255 / adcSampleCount) * adcSampleCount, true); 
 // ADCMan.setCapture(idx0Pin, adcSampleCount*2, true); 
 // ADCMan.setCapture(idx1Pin, adcSampleCount*2, true); 
  
  DEBUG(F("matchSignal size="));
  DEBUGLN(sizeof sigcode_norm);  
  DEBUG(F("subSample="));  
  DEBUGLN((int)subSample);    
  DEBUG(F("capture size="));
  DEBUGLN(ADCMan.getSampleCount(idx0Pin));  
}

void PerimeterClass::speedTest(){
  int loops = 0;
  unsigned long endTime = millis() + 1000;
  while (millis() < endTime){
    matchedFilter(0);
    loops++;
  }
  DEBUG(F("speedTest="));
  DEBUGLN(loops);
}

const int8_t* PerimeterClass::getRawSignalSample(byte idx) {
  //return rawSignalSample[idx];
  return NULL;
}

void PerimeterClass::run(){
  if (!enabled) return;
	for (int idx=0; idx < 2; idx++){
    if (ADCMan.isConvComplete(idxPin[idx])) {
     // Keep a sample of the raw signal
      //memset(rawSignalSample[0], 0, RAW_SIGNAL_SAMPLE_SIZE);
      //memcpy(rawSignalSample[0], ADCMan.getCapture(idxPin[0]), min(ADCMan.getCaptureSize(idxPin[0]), RAW_SIGNAL_SAMPLE_SIZE));
      // Process signal
      matchedFilter(idx);
    }
  }
	if (!isInside(IDX_LEFT)) Robot.sensorTriggered(SEN_PERIMETER_LEFT);
  if (!isInside(IDX_RIGHT)) Robot.sensorTriggered(SEN_PERIMETER_RIGHT);
}

int PerimeterClass::getMagnitude(byte idx){  
  return mag[idx];
}

int PerimeterClass::getSmoothMagnitude(byte idx){  
  return smoothMag[idx];
}

void PerimeterClass::printADCMinMax(int8_t *samples){
  int8_t vmax = SCHAR_MIN;
  int8_t vmin = SCHAR_MAX;
  for (byte i=0; i < ADCMan.getSampleCount(idxPin[0]); i++){
    vmax = max(vmax, samples[i]);
    vmin = min(vmin, samples[i]);
  }
  DEBUG(F("perimter min,max="));
  DEBUG((int)vmin);
  DEBUG(F(","));
  DEBUGLN((int)vmax);  
}

// perimeter V2 uses a digital matched filter
void PerimeterClass::matchedFilter(byte idx){
  int16_t sampleCount = ADCMan.getSampleCount(idxPin[0]);
  int8_t *samples = ADCMan.getSamples(idxPin[idx]);    
  if (callCounter == 100) {
    // statistics only
    callCounter = 0;
    signalMin[idx] = 9999;
    signalMax[idx] = -9999;
    signalAvg[idx] = 0;  
    for (int i=0; i < sampleCount; i++){
      int8_t v = samples[i];
      signalAvg[idx] += v;
      signalMin[idx] = min(signalMin[idx], v);
      signalMax[idx] = max(signalMax[idx], v);
    }
    signalAvg[idx] = ((double)signalAvg[idx]) / ((double)(sampleCount));
  }
  // magnitude for tracking (fast but inaccurate)    
  int16_t sigcode_size = sizeof sigcode_norm;
  int8_t *sigcode = sigcode_norm;  
  if (useDifferentialPerimeterSignal) sigcode = sigcode_diff;
  mag[idx] = corrFilter(sigcode, subSample, sigcode_size, samples, sampleCount-sigcode_size*subSample, filterQuality[idx]);
  if (swapCoilPolarity) mag[idx] *= -1;        
  // smoothed magnitude used for signal-off detection
  smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * ((float)abs(mag[idx]));

  // perimeter inside/outside detection
  if (mag[idx] > 0){
    signalCounter[idx] = min(signalCounter[idx]+1, 5);    
  } else {
    signalCounter[idx] = max(signalCounter[idx]-1, -5);    
  }
  if (mag[idx] < 0){
    lastInsideTime[idx] = millis();
  } 
    
  ADCMan.restartConv(idxPin[idx]);    
  if (idx == 0) callCounter++;
}

void PerimeterClass::resetTimedOut(){
  lastInsideTime[0] = millis();
  lastInsideTime[1] = millis();
}

int16_t PerimeterClass::getSignalMin(byte idx){
  return signalMin[idx];
}

int16_t PerimeterClass::getSignalMax(byte idx){
  return signalMax[idx];
}

int16_t PerimeterClass::getSignalAvg(byte idx){
  return signalAvg[idx];
}


float PerimeterClass::getFilterQuality(byte idx){
  return filterQuality[idx];
}

boolean PerimeterClass::isInside(){
  return (isInside(IDX_LEFT) && isInside(IDX_RIGHT));  
}

boolean PerimeterClass::isInside(byte idx){
  if (abs(mag[idx]) > 600) {
    // Large signal, the in/out detection is reliable.
    // Using mag yields very fast in/out transition reporting.
    return (mag[idx]<0);
  } else {
    // Low signal, use filtered value for increased reliability
    return (signalCounter[idx] < 0);
  }
}

bool PerimeterClass::signalTimedOut(){
  return (signalTimedOut(IDX_LEFT) && signalTimedOut(IDX_RIGHT));  
}


boolean PerimeterClass::signalTimedOut(byte idx){
  if (getSmoothMagnitude(idx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[idx] > timeOutSecIfNotInside * 1000) return true;
  return false;
}


// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat 
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data 

int16_t PerimeterClass::corrFilter(int8_t *H, int8_t subsample, int16_t M, int8_t *ip, int16_t nPts, float &quality){  
  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  int16_t Ms = M * subsample; // number of filter coeffs including subsampling

  // compute sum of absolute filter coeffs
  int16_t Hsum = 0;
  for (int16_t i=0; i<M; i++) Hsum += abs(H[i]); 
  Hsum *= subsample;

  // compute correlation
  // for each input value
  for (int16_t j=0; j<nPts; j++)
  {
      int16_t sum = 0;      
      int8_t *Hi = H;
      int8_t ss = 0;
      int8_t *ipi = ip;      
      // for each filter coeffs
      for (int16_t i=0; i<Ms; i++)
      {        
        sum += ((int16_t)(*Hi)) * ((int16_t)(*ipi));
        ss++;
        if (ss == subsample) {
          ss=0;
          Hi++; // next filter coeffs
        }
        ipi++;
      }      
      if (sum > sumMax) sumMax = sum;
      if (sum < sumMin) sumMin = sum;
      ip++;
  }      
  // normalize to 4095
  sumMin = ((float)sumMin) / ((float)(Hsum*127)) * 4095.0;
  sumMax = ((float)sumMax) / ((float)(Hsum*127)) * 4095.0;
  
  // compute ratio min/max 
  if (sumMax > -sumMin) {
    quality = ((float)sumMax) / ((float)-sumMin);
    return sumMax;
  } else {
    quality = ((float)-sumMin) / ((float)sumMax);
    return sumMin;
  }  
}




