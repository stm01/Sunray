/* stores settings in DS1307 realtime clock module
 */

#ifndef SETTINGS_H
#define SETTINGS_H


#include <Arduino.h>
#include <inttypes.h>


class SettingsClass {
    public:
      void begin();            
      void run();								
    protected:   
			byte read(int address);
			void write(int address, byte data);		      
};

extern SettingsClass Settings;


/*

template <class T> int eewrite(int &ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          Flash.write(ee++, *p++);
    return i;
}

template <class T> int eeread(int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = Flash.read(ee++);
    return i;
}

template <class T> int eereadwrite(boolean readflag, int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    { 
       if (readflag) *p++ = Flash.read(ee++);
         else Flash.write(ee++, *p++);
    }
    return i;
}


int eereadwriteString(boolean readflag, int &ee, String& value);

*/


#endif

