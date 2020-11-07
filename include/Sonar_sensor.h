#ifndef SONAR_SENSOR
#define SONAR_SENSOR

#include <Romi32U4.h>

class SonarSensor{
    private:
        const int pin_TRIG = 0;
        const int pin_ECHO = 1;
        volatile unsigned int pulseTime;
        volatile unsigned long startTime;
        void pulseHandler();

        static void ultrasonicISR();

    public:
        void Init(void); 
        float ReadData(void); 
        void PrintData(void);
        void loop(void);
};

#endif