#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//motor-speed controller
SpeedController PIcontroller;

void Behaviors::Init(void)
{
    Serial.begin(9600);
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    if(abs(data[2]) > threshold_pick_up) return 1;
    else return 0;
    return false;
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
        case IDLE:
            if(buttonA.getSingleDebouncedRelease()){ //transition condition
                robot_state = DRIVE; 
                PIcontroller.Stop(); //action
            } else { //transition condition
                robot_state = IDLE; 
                PIcontroller.Stop(); //action 
            }   
            break;
            
        case DRIVE:
            if (DetectCollision()) { //transition condition
                robot_state = REVERSE; 
                PIcontroller.Stop(); //action
            } else if(buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp()){ //transition condition
                robot_state = IDLE; 
                PIcontroller.Stop(); //action
            } else {
                robot_state = DRIVE; 
                PIcontroller.Run(80,80); //action 
            }   
            break;
        case REVERSE:
            if (PIcontroller.Reverse(50, 10)){//action
                robot_state = TURN; 
                PIcontroller.Stop(); //action
            }
            break;
        case TURN:
            if (PIcontroller.Turn(90, 0)){//action
                robot_state = DRIVE; 
                PIcontroller.Stop(); //action
            }
            break;
            break;
    }
    Serial.println(robot_state);
}