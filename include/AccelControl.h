#ifndef ACCELCONTROL
#define ACCELCONTROL

#include <Romi32U4.h>

class AccelControl{
    public:
        void Init(void);
        void Stop(void);
        void Run(void);

    private:
        enum ROBOT_STATE {IDLE, DRIVE};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
};

#endif