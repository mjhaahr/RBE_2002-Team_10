#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
    	enum ROBOT_STATE {IDLE, DRIVETOWALL, TURN90, WALLFOLLOW, WALLFOLLOW10CM}
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
        
    public:
    	void Init(void);
        void Stop(void);
        void Run(void);
};
#endif
