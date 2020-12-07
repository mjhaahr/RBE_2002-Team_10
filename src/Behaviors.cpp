#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"

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
        } else if(true){ //Collision
        	robot_state = WAITFORBUTTON; 
            robot.Stop();
        } else {
        	robot.constrainAccel(100); //drives straight and constrained
        }
        break;

    case WAITFORBUTTON:
        if(buttonA.getSingleDebouncedRelease()){
            robot_state = WALLFOLLOW; 
            robot.Stop();
            delay(500);
            robot.Turn(PI/2, 1);            
        } else{
            robot_state = WAITFORBUTTON; 
            robot.Stop();
        }
    	break;
    case WALLFOLLOW:
        if (true){ //Ramp
            robot_state = WALLFOLLOW10CM; 
            robot.Stop();
        } else {
            robot_state = WALLFOLLOW; 
            robot.WallFollow(20); //distance
        }
        break;

    case WALLFOLLOW10CM:
        boolean done;
        if (done){ //10CM travel
            robot_state = IDLE; 
            robot.Stop();
        } else {
            robot_state = WALLFOLLOW10CM; 
            done = robot.WallFollow10CM(20); //distance
        }
        break;
    default:
        robot_state = IDLE; 
        robot.Stop();  
        break;
    };
}