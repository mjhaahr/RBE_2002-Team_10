#ifndef POSITION_ESTIMATION
#define POSITION_ESTIMATION

#include <Romi32U4.h>

class Position{
    private:
        float x, y, theta;  // internal stored vals
		float vL, vR; //Left and Right velocities in m/s (input is m/s)
        float R, w, V;  // Intermediate Variables (Dist to ICC, Angular Vel, Abs Vel)
		float rPlusL, rMinusL;  // Velocities added and subtracted
		float xDelta, yDelta, thetaDelta; //Change in position
        const float eqThreshold = 0.0002; //0.2 mm/s tolerance
        unsigned long time_prev, time_now;
        const float l = 0.142875; //wheel track in meters (updated to match actual device)
        const float l2 = l / 2.0; //half wheel track
        const float deltaT = 50; //time interval
        
    public:
        struct pose_data {
            float X;
            float Y;
            float THETA;
        };
        void Init(void);
        void UpdatePose(float,float);
        pose_data ReadPose(void);
        void PrintPose(void);
        void Stop(void);
};

#endif
