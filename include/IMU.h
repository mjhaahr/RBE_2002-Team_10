#ifndef IMU
#define IMU

#include <Romi32U4.h>

class IMU_sensor{
    private:
        int data[3] = {0};
        char report[120];
        const int threshold = 300; //collision threshold for 50-100mm/s (may need retesting)
        const int crossover = -30; //ramp crossover point in y term gyro, as it hits the ground at the end of the ramp
        //float pitch = 0;
        //const float deltaT = 0.025; //s (25 ms)
        //unsigned long targetTime = 0; //next update time
        
    public:
        struct acceleration_data {
            int X;
            int Y;
            int Z;
        };
        void Init(void);
        void PrintAcceleration(void);
        void PrintGyro(void);
        acceleration_data ReadAcceleration(void);
        acceleration_data ReadGyro(void);
        //void gyroPositionUpdate(void);
        boolean DetectCollision(void);
        boolean EndOfRamp(void);
};

#endif
