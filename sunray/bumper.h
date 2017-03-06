#ifndef BUMPER_H
#define BUMPER_H



class BumperClass {
    public:
      void begin();            
      void run();
	    bool pressed();
	    bool leftPressed;
	    bool rightPressed;      
    protected:           
      unsigned long nextCheckTime;
};

extern BumperClass Bumper;

#endif

