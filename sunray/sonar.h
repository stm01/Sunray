// HC-SR04 ultrasonic sensor driver (2cm - 400cm)
// for 3 sensors, optimized for speed: based on hardware interrupts (no polling)
// up to 100 Hz measurements tested

#ifndef SONAR_H
#define SONAR_H



class SonarClass {
    public:      
			bool enabled;
			int triggerBelow;
			void begin();            
      void run();
	    bool obstacle();	    
			unsigned int distanceLeft; // cm
			unsigned int distanceRight;
			unsigned int distanceCenter;  		
			bool verboseOutput; 
    protected:                 
			unsigned int convertCm(unsigned int echoTime);
};

extern SonarClass Sonar;

#endif

