/*
License
Copyright (c) 2013-2017 by Alexander Grau

Private-use only! (you need to ask for a commercial-use)
 
The code is open: you can modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

The code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
  
 */

#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
#ifndef __AVR__
  #include "DueTimer.h"
#endif

BuzzerClass Buzzer;


#ifndef __AVR__
static boolean tone_pin_state = false;

void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state);  
}
#endif 



void BuzzerClass::sound(SoundSelect idx, bool async){
  soundIdx = idx;
  toneIdx = 0;
  nextToneTime = millis();
  if (!async){
    while (nextToneTime != 0){
      run();
    }
  }
}

void BuzzerClass::run(){  
  if (nextToneTime == 0) return;
  unsigned long m = millis();
  if (m < nextToneTime) return;
  switch (soundIdx){
    case SND_READY:
      switch (toneIdx){
        case 0: tone(1200); nextToneTime = m + 300; break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(2200); nextToneTime = m + 300; break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4: tone(3200); nextToneTime = m + 500; break;        
        case 5: noTone();  nextToneTime = m + 200; break;
        case 6:            nextToneTime = 0;       break;
      }
      break;
    case SND_PROGRESS:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 20;  break;
        case 1: noTone();  nextToneTime = m + 20;  break;
        case 2:         	 nextToneTime = 0;      break;
      }
      break;
    case SND_OVERCURRENT:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 50;  break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(4200); nextToneTime = m + 50;  break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4:         	 nextToneTime = 0;       break;
      }
      break;
    case SND_STUCK:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 50;  break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(3200); nextToneTime = m + 50;  break;
        case 3: noTone();  nextToneTime = m + 200; break;
				case 4: tone(2200); nextToneTime = m + 50;  break;
				case 5: noTone();  nextToneTime = m + 200; break;
        case 6:            nextToneTime = 0;       break;
      }
      break;			
    case SND_TILT:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 100; break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(1200); nextToneTime = m + 100; break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4:         	 nextToneTime = 0;       break;
      }
      break;
    case SND_PERIMETER_TIMEOUT:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 500; break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(5200); nextToneTime = m + 500; break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4: tone(6200); nextToneTime = m + 500; break;        
        case 5: noTone();  nextToneTime = m + 200; break;
        case 6:            nextToneTime = 0;       break;
      }
      break;      
  }
  toneIdx++;
}

void BuzzerClass::begin()
{
   pinMode(pinBuzzer, OUTPUT);                
   digitalWrite(pinBuzzer, LOW);
   toneIdx=0;
   nextToneTime=0;   
}


void BuzzerClass::tone( uint16_t  freq )
{
#ifdef __AVR__   
   ::tone(pinBuzzer, freq);  
#else  
  pinMode(pinBuzzer, OUTPUT);
  Timer1.attachInterrupt(toneHandler).setFrequency(freq).start(); 
#endif     
}


void BuzzerClass::noTone(){
#ifdef __AVR__   
   ::noTone(pinBuzzer);
#else
  Timer1.stop();
  //pinMode(pinBuzzer, INPUT);  
  digitalWrite(pinBuzzer, LOW);
#endif     
}



