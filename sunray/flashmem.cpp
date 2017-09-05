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

#include "flashmem.h"
#include "config.h"

#ifdef __AVR__
  #include <EEPROM.h>
#else
  #include "flash_efc.h"
  #include <chip.h>

  // 1Kb of data
  #define DATA_LENGTH   ((IFLASH1_PAGE_SIZE/sizeof(byte))*4)

  // choose a start address that's offset to show that it doesn't have to be on a page boundary
  #define  FLASH_START  ((byte *)IFLASH1_ADDR)

  //  FLASH_DEBUG can be enabled to get debugging information displayed.
  #define FLASH_DEBUG 1

  #ifdef FLASH_DEBUG
  #define _FLASH_DEBUG(x) DEBUG(x);
  #else
  #define _FLASH_DEBUG(x)
  #endif
#endif  


FlashClass Flash;


int eereadwriteString(boolean readflag, int &ee, String& value)
{
  unsigned int i;
  if (readflag) {
    value = "";
    char ch = Flash.read(ee++);
    while (ch) {
      value += ch;
      ch = Flash.read(ee++);
    }
  } else {
    for(i=0; i<value.length(); i++) {
      Flash.write(ee++, value.charAt(i));
    }
    Flash.write(ee++, 0);
  }
}

FlashClass::FlashClass() {
  verboseOutput = false;
#ifdef __AVR__  
#else
  uint32_t retCode;

  /* Initialize flash: 6 wait states for flash writing. */
  retCode = flash_init(FLASH_ACCESS_MODE_128, 6);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Flash init failed\n"));
  }
#endif
}


byte FlashClass::read(uint32_t address) {
#ifdef __AVR__  
  return EEPROM.read(address);
#else
  return FLASH_START[address];
#endif
}


byte* FlashClass::readAddress(uint32_t address) {
#ifdef __AVR__  
  byte d = EEPROM.read(address);
  return &d;
#else
  return FLASH_START+address;
#endif
}

void FlashClass::dump(){
  DEBUGLN(F("EEPROM dump"));
  for (int i=0; i < 1024; i++){
    byte v = read(i);
    if (v != 0){
      DEBUG(i);
      DEBUG(F("="));
      DEBUG(v);
      DEBUG(F(", "));
    }
  }
  DEBUGLN();
}

boolean FlashClass::write(uint32_t address, byte value) {
  if (verboseOutput){
    ROBOTMSG.print(F("!76,"));
    ROBOTMSG.print(address);
    ROBOTMSG.print(F(","));
    ROBOTMSG.print(value);  
    ROBOTMSG.println();
  }
#ifdef __AVR__  
  EEPROM.write(address, value);
  return true;
#else
  uint32_t retCode;
  uint32_t byteLength = 1;  
  byte *data;

  retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + byteLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Failed to unlock flash for write\n"));
    return false;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, &value, byteLength, 1);
  //retCode = flash_write((uint32_t)FLASH_START, data, byteLength, 1);

  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Flash write failed\n"));
    return false;
  }

  // Lock page
  retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + byteLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Failed to lock flash page\n"));
    return false;
  }
  return true;
#endif
}


boolean FlashClass::write(uint32_t address, byte *data, uint32_t dataLength) {
#ifdef __AVR__  
  for (int i=0; i < dataLength; i++){
    EEPROM.write(address+i, data[i]);    
  }
  return true;
#else
  uint32_t retCode;

  if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
    _FLASH_DEBUG(F("Flash write address too low\n"));
    return false;
  }

  if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    _FLASH_DEBUG(F("Flash write address too high\n"));
    return false;
  }

  if (((uint32_t)FLASH_START+address & 3) != 0) {
    _FLASH_DEBUG(F("Flash start address must be on four byte boundary\n"));
    return false;
  }

  // Unlock page
  retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Failed to unlock flash for write\n"));
    return false;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, data, dataLength, 1);

  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Flash write failed\n"));
    return false;
  }

  // Lock page
    retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG(F("Failed to lock flash page\n"));
    return false;
  }
  return true;
#endif
}




