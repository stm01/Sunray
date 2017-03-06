
// Android remote control (ArduRemote / pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/

// example usage:
//   RemoteControl remote;
//   remote.initSerial(19200);
//   while (true){
//     remote.readSerial();
//     remote.run();
//  }


#ifndef REMOTECTL_H
#define REMOTECTL_H

#include <Arduino.h>
#include "pid.h"
#include "perimeter.h"


// pfodApp state
enum { PFOD_OFF, PFOD_MENU, PFOD_PLOT, PFOD_MAP };


class RemoteControl
{
  public:
    RemoteControl();
    void begin();
    void run();    	
  protected: 
    byte pfodState;
    unsigned long nextPlotTime;
    bool pfodCmdComplete;
    String pfodCmd;    
    void sendMainMenu(boolean update);
	void processMainMenu(String pfodCmd);
              
};


extern RemoteControl RemoteCtl;

#endif

