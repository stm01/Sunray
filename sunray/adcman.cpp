/* NOTE: requires Arduino Due
   Continouesly performs ADC conversion for all configured ADC channels one after the other using Arduino Due DMA transfer.
*/

#include <chip.h>
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "config.h"
#include "buzzer.h"
#include "flashmem.h"

#define ADDR 0
#define MAGIC 1

#define ADC_SAMPLE_COUNT_MAX 255
#define INVALID_CHANNEL 99

#define NO_CHANNEL 255

uint16_t dmaData[ADC_SAMPLE_COUNT_MAX];
ADCManager ADCMan;


ADCManager::ADCManager(){
  convCounter = 0;  
  chNext = 0;
  chCurr = INVALID_CHANNEL;    
  for (int i=0; i < ADC_CHANNEL_COUNT_MAX; i++){
    channels[i].sampleCount = 0;
    channels[i].zeroOfs =0;
    channels[i].value =0;
    channels[i].convComplete = false;
  }  
  sampleRate = SRATE_38462;   // sampling frequency 38462 Hz
}


void ADCManager::begin(){    
  /*pinMode(A0, INPUT);
    while(true){
      DEBUGLN(analogRead(A0));
    }
  */
  // free running ADC mode, f = ( adclock / 21 cycles per conversion )
  // example f = 19231  Hz:  using ADCCLK=405797 will result in a adcclock=403846 (due to adc_init internal conversion)
  uint32_t adcclk;
  switch (sampleRate){
    case SRATE_38462: adcclk = 811595; break;
    case SRATE_19231: adcclk = 405797; break;
    case SRATE_9615 : adcclk = 202898; break;
  }  
  pmc_enable_periph_clk (ID_ADC); // To use peripheral, we must enable clock distributon to it
  adc_init(ADC, SystemCoreClock, adcclk, ADC_STARTUP_FAST); // startup=768 clocks
  adc_disable_interrupt(ADC, 0xFFFFFFFF);
  adc_set_resolution (ADC, ADC_12_BITS);  
  adc_configure_power_save (ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF); // Disable sleep
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);  // tracking=0, settling=17, transfer=1      
  adc_set_bias_current (ADC, 1); // Bias current - maximum performance over current consumption
  adc_disable_tag (ADC);  // it has to do with sequencer, not using it 
  adc_disable_ts (ADC);   // disable temperature sensor 
  adc_stop_sequencer (ADC);  // not using it
  adc_disable_all_channel (ADC);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 1); // triggering from software, freerunning mode      
  adc_start( ADC );  
  
 /* // test conversion
  setupChannel(A0, 1, false);  
  setupChannel(A1, 3, false);    
  while(true){    
    DEBUG("test A0=");
    DEBUG(getVoltage(A0));
    DEBUG("  A1=");
    DEBUGLN(getVoltage(A1));    
    DEBUG("  cnvs=");
    DEBUGLN(getConvCounter());    
    run();
    delay(500);        
  }*/  
}

void ADCManager::printInfo(){
  DEBUGLN(F("---ADC---"));  
  DEBUG(F("conversions="));
  DEBUGLN(convCounter);
  DEBUG(F("sampleRate="));
  switch (sampleRate){
    case SRATE_38462: DEBUGLN(F("38462")); break;
    case SRATE_19231: DEBUGLN(F("19231")); break;
    case SRATE_9615 : DEBUGLN(F("9615")); break;
  }    
  for (int ch=0; ch < ADC_CHANNEL_COUNT_MAX; ch++){
    if (channels[ch].sampleCount != 0){
      DEBUG(F("AD"));
      DEBUG(ch);
      DEBUG(F("\t"));    
      DEBUG(F("sampleCount="));    
      DEBUGLN(channels[ch].sampleCount);      
      DEBUG(F("\t"));    
      DEBUG(F("autoCalibrate="));    
      DEBUGLN(channels[ch].autoCalibrate);      
    }
  }
}

int ADCManager::getConvCounter(){
  int res = convCounter;
  convCounter = 0;
  return res;
}

void ADCManager::setupChannel(byte pin, int samplecount, bool autocalibrate){
  byte ch = pin-A0;
  pinMode(pin, INPUT);
  channels[ch].pin = pin; 
  channels[ch].autoCalibrate = autocalibrate;  
  channels[ch].convComplete = false;
  channels[ch].maxValue = 0;
  channels[ch].minValue = 0;
  setSampleCount(ch, samplecount);
}

void ADCManager::setSampleCount(byte ch, int samplecount){
  samplecount = min(samplecount, ADC_SAMPLE_COUNT_MAX);
  channels[ch].samples = (int8_t *)realloc(channels[ch].samples, samplecount);
  channels[ch].sampleCount = samplecount;  
}

bool ADCManager::isConvComplete(byte pin){
  byte ch = pin-A0;
  return channels[ch].convComplete;
}


void ADCManager::restartConv(byte pin){
  byte ch = pin-A0;
  channels[ch].convComplete = false;
}

int8_t* ADCManager::getSamples(byte pin){
  byte ch = pin-A0;
  return channels[ch].samples;
}

int ADCManager::getSampleCount(byte pin){
  byte ch = pin-A0;
  return channels[ch].sampleCount;
}

uint16_t ADCManager::getValue(byte pin){
  byte ch = pin-A0;  
  channels[ch].convComplete = false;
  return channels[ch].value;  
}

float ADCManager::getVoltage(byte pin){
  uint16_t v = getValue(pin);
  return ((float)v) / ((float) ((1 << ADC_BITS)-1)) * ADC_REF;   
}

void ADCManager::init(byte ch){
  //adc_disable_channel_differential_input(ADC, (adc_channel_num_t)g_APinDescription[ channels[ch].pin ].ulADCChannelNumber );
  // configure Peripheral DMA  
  adc_enable_channel( ADC, (adc_channel_num_t)g_APinDescription[ channels[ch].pin ].ulADCChannelNumber  );   
  delayMicroseconds(100);  
  PDC_ADC->PERIPH_RPR = (uint32_t) dmaData; // address of buffer
  PDC_ADC->PERIPH_RCR = channels[ch].sampleCount;
  PDC_ADC->PERIPH_PTCR = PERIPH_PTCR_RXTEN; // enable receive      
}

// start another conversion
void ADCManager::run(){
  if ((adc_get_status(ADC) & ADC_ISR_ENDRX) == 0) return; // conversion busy
  // post-process sampling data
  if (chCurr != INVALID_CHANNEL){
    adc_disable_channel( ADC, (adc_channel_num_t)g_APinDescription[ channels[chCurr].pin ].ulADCChannelNumber  );   
    postProcess(chCurr);
    channels[chCurr].convComplete = true;
    chCurr = INVALID_CHANNEL;
    convCounter++;
  } 
  // start next channel sampling    
  for (int i=0; i < ADC_CHANNEL_COUNT_MAX; i++){
    chNext++;
    if (chNext == ADC_CHANNEL_COUNT_MAX) chNext = 0;
    if (channels[chNext].sampleCount != 0){
      if (!channels[chNext].convComplete){
        chCurr = chNext;
        init(chCurr);               
        break;
      }
    }
  }
}


void ADCManager::postProcess(byte ch){  
  //DEBUG("post ch");
  //DEBUG(ch);
  //DEBUG("=");
  //DEBUG(dmaData[0]);
  uint16_t vmax = 0;
  uint16_t vmin = 9999;   
  if (channels[ch].autoCalibrate) {  
    // determine zero point    
    for (int i=0; i < channels[ch].sampleCount; i++){
      uint16_t value = dmaData[i];
      vmax = max(vmax, value);
      vmin = min(vmin, value);
    }
    // determine gliding min,max,ofs
    channels[ch].maxValue = 0.9 * ((double)channels[ch].maxValue) + 0.1 * ((double)vmax);
    channels[ch].minValue = 0.9 * ((double)channels[ch].minValue) + 0.1 * ((double)vmin);
    channels[ch].zeroOfs = channels[ch].minValue + (channels[ch].maxValue - channels[ch].minValue)/2.0;
  }  
  // ------determine average value-------
  int32_t res = 0;
  int i;
  for (i=0; i < channels[ch].sampleCount; i++){
    uint16_t value = dmaData[i];    
    //DEBUG(" v1=");
    //DEBUG(value);    
    value -= channels[ch].zeroOfs;
    res += value;
  }
  //DEBUG(" res=");
  //DEBUG(res);    
  channels[ch].value = ((float)res) / ((float)channels[ch].sampleCount);      
  // --------transfer DMA samples----------
  for (int i=0; i < channels[ch].sampleCount; i++){
    uint16_t value = dmaData[i];
    value -= channels[ch].zeroOfs;
    channels[ch].samples[i] = min(SCHAR_MAX,  max(SCHAR_MIN, ((int8_t) (value >> (ADC_BITS-8))) )); // convert to 8 bits
  }
  //DEBUG(" val");
  //DEBUGLN(channels[ch].value);
}



