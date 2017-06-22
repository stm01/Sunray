// ROBOTMSG handling

#ifndef ROBOTMSG_H
#define ROBOTMSG_H

#include <Arduino.h>


class RobotMsgClass {
    public:      
			unsigned long nextInfoTime;
			void begin();
			void run();			
    protected:   
			void readRobotMessages();
			void printSensorData();
			void sendMap();
			void sendParticles();
			void sendPerimeterOutline();
};

extern RobotMsgClass RobotMsg;

#endif

