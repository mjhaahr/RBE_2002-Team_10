#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h" //will need some remaking

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

void Behaviors::Init(void)
{
    robot.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = DRIVETOWALL; 
            robot.Stop();
            delay(1000);             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    
    case DRIVETOWALL:
        if(buttonA.getSingleDebouncedRelease()){
            robot_state = IDLE; 
            robot.Stop();             
        } else if(){
        	robot_state = IDLE; 
            robot.Stop();
        } else {
        	robot.StraightConstrained(200); //drives straight and constrained
        }
        break;
    case TURN90:
    	
    };
}