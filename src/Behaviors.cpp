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
            robot_state = DRIVE; 
            robot.Stop();             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    
    case DRIVE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = IDLE; 
            robot.Stop();             
        } 
        else {
            /*Serial.println("start");
            robot.MoveToPosition(0.610,0.610); //approx 1 ft
            Serial.println("end");*/
            Serial.println("start");
            
            for(int i = 0; i < 5; i++){
                Serial.print("Pos: ");
                Serial.println(i);

                robot.MoveToPosition(positions[i][0],positions[i][1]);
                delay(500);

                Serial.print("Pos: ");
                Serial.println(i + 1);
            }
            
            Serial.println("end");
            robot_state = IDLE;
        }
        break;
    };
}