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
        float error_distance = 0;
        float error_theta = 0;
		const float maxAccel = 20.0; //mm/2^2
		const float deltaV = maxAccel * 0.050; //change in velocity from maxAccel and encoder update rate
		float constrainAccel(float); //acceleration constrain
        const float distanceTolerance = 0.02; //10 mm error tolerance

        const float KpD = 0.5;
        const float KpT = 11;
        const float KiD = 0.01; 
        const float KiT = 0.000; 
        const float KdD = 0.0;
        const float KdT = 3;
        float E_dist = 0;
        float E_theta = 0;
        float theta_last = 0;
        float T_diff = 0;

        int totalAngle = 0;

        Position::pose_data currentPos;

    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        void Init(void);
        void Run(float, float); 
        boolean Turn(float,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean MoveToPosition(float,float); //target_x, target_y
        void Stop(void);
};

#endif
