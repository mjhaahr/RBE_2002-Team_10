#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        enum ROBOT_STATE {IDLE, DRIVE};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        float positions[5][2] = {
            {0.610, 0.610},
            {1.220, 0.915},
            {1.525, 0},
            {0.610, -0.305},
            {0, 0} //back at origin position
        }; //positions array (in mm), based on position deltas, assume starts in first position, allows for easy access and loop behavior
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
};
#endif