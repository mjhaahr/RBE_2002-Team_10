#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = 400;
        int threshold_pick_up = 1600;
        int data[6] = {0}; //first 3 are for filtered, last 3 are for raw, but converted
        enum ROBOT_STATE {IDLE, DRIVE, REVERSE, TURN, DATA};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        
        char report[120]; //data acquisition string buffer
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
};

#endif