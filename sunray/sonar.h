// HC-SR04 ultrasonic sensor driver (2cm - 400cm)

#ifndef SONAR_H
#define SONAR_H



class SonarClass {
    public:
      void begin();            
      void run();
	    bool obstacle();	    
			unsigned int distanceLeft;
			unsigned int distanceRight;
			unsigned int distanceCenter;  		
			bool verboseOutput; 
    protected:                 
};

extern SonarClass Sonar;

#endif

