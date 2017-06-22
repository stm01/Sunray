#ifndef BUMPER_H
#define BUMPER_H



class BumperClass {
    public:
		  bool enabled;
      void begin();                  
			void run();			
	    bool pressed();
			bool changed();
	    bool leftPressed;
	    bool rightPressed;      			
			bool leftChanged;
			bool rightChanged;
    protected:           		
      unsigned long nextCheckTime;
};

extern BumperClass Bumper;

#endif

