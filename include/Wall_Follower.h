#ifndef WALL_FOLLOWER
#define WALL_FOLLOWER

#include <Romi32U4.h>

class WallFollower {
    private:
        const float Kp = 6; //Adapt parameters Kp and Kd until your robot consistently drives along a wall
        const float Kd = 3.5;
        float E_left = 0;
        float E_right = 0;
        float E_distance = 0;
        float prev_e_distance = 0;
        const float angleCompensation = cos(PI/4.0); // Pi/4 is 45˚, the angle of the IR sensor on the robot.

    public:
        void Init(void);
        float Process(float);
};

#endif