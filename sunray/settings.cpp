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

#include "settings.h"
#include "config.h"
#include "i2c.h"
#include <Arduino.h>

SettingsClass Settings;

#define DS1307_ADDRESS B1101000


/*
// DS1307 real time driver
boolean readDS1307(datetime_t &dt){
  byte buf[8];  
  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 8, buf, 3) != 8) {
    Console.println("DS1307 comm error");    
    //addErrorCounter(ERR_RTC_COMM);
    return false;
  }      
  if (   ((buf[0] >> 7) != 0) || ((buf[1] >> 7) != 0) || ((buf[2] >> 7) != 0) || ((buf[3] >> 3) != 0) 
      || ((buf[4] >> 6) != 0) || ((buf[5] >> 5) != 0) || ((buf[7] & B01101100) != 0) ) {    
    Console.println("DS1307 data1 error");    
    //addErrorCounter(ERR_RTC_DATA);
    return false;
  }
  datetime_t r;
  r.time.minute    = 10*((buf[1] >>4) & B00000111) + (buf[1] & B00001111);
  r.time.hour      = 10*((buf[2] >>4) & B00000111) + (buf[2] & B00001111);
  r.date.dayOfWeek = (buf[3] & B00000111) - 1;
  r.date.day       = 10*((buf[4] >>4) & B00000011) + (buf[4] & B00001111);
  r.date.month     = 10*((buf[5] >>4) & B00000001) + (buf[5] & B00001111);
  r.date.year      = 10*((buf[6] >>4) & B00001111) + (buf[6] & B00001111);
  if (    (r.time.minute > 59) || (r.time.hour > 23) || (r.date.dayOfWeek > 6)  
       || (r.date.month > 12)  || (r.date.day > 31)  || (r.date.day < 1)         
       || (r.date.month < 1)   || (r.date.year > 99) ){
    Console.println("DS1307 data2 error");    
    //addErrorCounter(ERR_RTC_DATA);
    return false;
  }  
  r.date.year      += 2000;
  dt = r;
  return true;
}

boolean setDS1307(datetime_t &dt){
  byte buf[7];
  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 7, buf, 3) != 7){
    Console.println("DS1307 comm error");    
    //addErrorCounter(ERR_RTC_COMM);
    return false;
  }
  buf[0] = buf[0] & B01111111; // enable clock
  buf[1] = ((dt.time.minute / 10) << 4) | (dt.time.minute % 10);
  buf[2] = ((dt.time.hour   / 10) << 4) | (dt.time.hour   % 10);
  buf[3] = dt.date.dayOfWeek + 1;
  buf[4] = ((dt.date.day    / 10) << 4) | (dt.date.day    % 10);
  buf[5] = ((dt.date.month  / 10) << 4) | (dt.date.month  % 10);
  buf[6] = ((dt.date.year % 100  / 10) << 4) | (dt.date.year % 10);
  I2CwriteToBuf(DS1307_ADDRESS, 0x00, 7, buf);
  return true;
}
*/

/*
void Robot::loadSaveUserSettings(boolean readflag){
  int addr = ADDR_USER_SETTINGS;
  short magic = 0;
  if (!readflag) magic = MAGIC;  
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    DEBUGLN(F("EEPROM USERDATA: NO EEPROM USER DATA"));
    DEBUGLN(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, developerActive);            
  DEBUG(F("loadSaveUserSettings addrstop="));
  DEBUGLN(addr);
}

void Robot::loadUserSettings(){  
  DEBUGLN(F("loadUserSettings"));  
  loadSaveUserSettings(true);
}


void SettingsClass::save(){
  DEBUGLN(F("USER SETTINGS ARE SAVED"));	
  loadSaveUserSettings(false);
}

*/


void SettingsClass::run(){  
}

void SettingsClass::begin(){
	DEBUG(F("testing DS1307..."));
	write(0, 42);
	byte data = read(0);
	if (data != 42) DEBUGLN(F("failed - battery connected to PCB?"));
	  else DEBUGLN(F("success"));
}


byte SettingsClass::read(int address){
  uint8_t data = 0;
  I2C_readFrom(DS1307_ADDRESS, 8 + address, 1, &data, 1);  
	return data;
}

void SettingsClass::write(int address, byte data){
	I2C_writeToValue(DS1307_ADDRESS, 8 + address, data);
}


