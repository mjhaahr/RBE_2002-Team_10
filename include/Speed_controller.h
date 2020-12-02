#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>
#include "Position_estimation.h"

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1; 
        float E_left = 0; 
        float E_right = 0;
        int counts = 1450; //number of counts for a 180 degree turn; you will likely have to change this
		const float maxAccel = 200.0; //mm/2^2
		const float deltaV = maxAccel * 0.060; //change in velocity from maxAccel and encoder update rate (should be 50 ms, but due to how slow some code is, is longer in actual fact)

        Position::pose_data currentPos;
    public:
        void Init(void);
        void Run(float, float); 
        boolean Turn(float,int); //degrees, direction of rotation: 0->left, 1->right
        void constrainAccel(int); //acceleration constrain
        void Stop(void);
};

#endif
