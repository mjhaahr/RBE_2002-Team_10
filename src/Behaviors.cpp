#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "IMU.h"

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

//IMU
IMU_sensor accel;

void Behaviors::Init(void)
{
    robot.Init();
    accel.Init();
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
        } else if(accel.DetectCollision()){ //Collision
            robot.Stop();

            unsigned long targetTime = millis() + 1500; //back away from wall to make turning easier
            while (targetTime > millis()){
                robot.Run(-50, -50);
            }

        	robot_state = WAITFORBUTTON; 
            robot.Stop();
        } else {
        	robot.constrainAccel(150); //drives straight and constrained
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
        if (accel.EndOfRamp()){ //Ramp
            robot_state = WALLFOLLOW10CM; 
            robot.Stop();
        } else {
            robot_state = WALLFOLLOW;
            robot.WallFollow(20); //distance
        }
        break;

    case WALLFOLLOW10CM:
        robot.WallFollow10CM(20);
        robot_state = IDLE; 
        break;
    default:
        robot_state = IDLE; 
        robot.Stop();  
        break;
    };
}