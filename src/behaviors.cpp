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
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)){ //manual setting as the const int is being redifined, no clue why
        Serial.print("Collision: \t");
        Serial.print(data[0]);
        Serial.print('\t');
        Serial.println(data[1]);
        return 1;
    } 
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    if(abs(data[2]) > threshold_pick_up) {
        Serial.print("Picked up: \t");
        Serial.println(data[2]);
        return 1; //manual setting as the const int is being redifined, no clue why
    } else return 0;
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
                PIcontroller.Run(50,50); //action 
            }   
            break;
        case REVERSE:
            if (PIcontroller.Reverse(50, 10)){//action
                robot_state = TURN; //for collision testing
                PIcontroller.Stop(); //action
            }
            break;
        case TURN:
            if (PIcontroller.Turn(90, 0)){//action
                robot_state = DRIVE; 
                PIcontroller.Stop(); //action
            }
            break;
        case DATA:
            if(buttonA.getSingleDebouncedRelease()){ //transition condition
                robot_state = IDLE; 
                PIcontroller.Stop(); //action
            } else {
                robot_state = DATA; 
                PIcontroller.Run(50,50); //action 

                auto data_acc = LSM6.ReadAcceleration(); //IMU Read and data processing
                data[3] = data_acc.X * 0.061;
                data[4] = data_acc.Y * 0.061;
                data[5] = data_acc.Z * 0.061;
                data[0] = med_x.Filter(data[3]);
                data[1] = med_y.Filter(data[4]);
                data[2] = med_z.Filter(data[5]);

                sprintf(report, "%6d %6d %6d %6d %6d %6d", data[3], data[4], data[5], data[0], data[1], data[2]); //data sprintf (scaled raw X, scaled raw Y, scaled raw Z, filtered X, filtered Y, filtered Z)
                Serial.println(report); 
                delay(5); //5 ms delay for known time interval
            }  
            break;
        default:
            robot_state = IDLE;
            PIcontroller.Stop(); //action
            break;
    }
    Serial.println(robot_state);
}