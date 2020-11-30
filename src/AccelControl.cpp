#include <Romi32U4.h>
#include "AccelControl.h"
#include "Speed_controller.h"

//sensors
Romi32U4ButtonA pbA;
Romi32U4ButtonB pbB;
Romi32U4ButtonC pbC;

//motor-speed controller
SpeedController chassis;

void AccelControl::Init(void)
{
    chassis.Init();
}

void AccelControl::Stop(void)
{
    chassis.Stop();
}

void AccelControl::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(pbA.getSingleDebouncedRelease()){ 
            robot_state = DRIVE; 
            chassis.Stop();             
        } 
        else { 
            robot_state = IDLE;
            chassis.Stop(); 
        }   
        break;
    
    case DRIVE:
        if(pbA.getSingleDebouncedRelease()){ 
            robot_state = IDLE; 
            chassis.Stop();             
        } else if (pbB.getSingleDebouncedRelease()){
            Serial.println("Begin");
            chassis.StraightConstrained(200, 7);
            Serial.println("End");
            robot_state = DRIVE;
        } else if (pbC.getSingleDebouncedRelease()){
            Serial.println("Begin");
            chassis.Straight(200, 5);
            Serial.println("End");
            robot_state = DRIVE;
        }
        break;
    };
}