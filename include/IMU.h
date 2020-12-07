#ifndef IMU
#define IMU

#include <Romi32U4.h>

class IMU_sensor{
    private:
        int data[3] = {0};
        char report[120];
        const int threshold = 300; //collision threshold for 50-100mm/s (may need retesting)
        const int crossover = -150; //ramp crossover point (from on incline to flat) (may need retesting) (Z is not very sensitive for angular changes, probably should look at using the gyro)
        enum RAMP_STATE {BEFORE, ON, OFF};
        RAMP_STATE ramp_state = BEFORE; //initial state: BEFORE
        
    public:
        struct acceleration_data {
            int X;
            int Y;
            int Z;
        };
        void Init(void);
        void PrintAcceleration(void);
        acceleration_data ReadAcceleration(void);
        boolean DetectCollision(void);
        boolean EndOfRamp(void);
};

#endif
