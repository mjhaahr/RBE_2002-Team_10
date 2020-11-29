#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        enum ROBOT_STATE {IDLE, DRIVE};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        float positions[5][2] = {
            {200, 200},
            {200, 100},
            {100, -300},
            {-300, -100},
            {-200, 100} //back at origin position
        }; //positions array (in mm), based on position deltas, assume starts in first position, allows for easy access and loop behavior
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
};
#endif