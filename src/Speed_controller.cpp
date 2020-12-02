#include <Romi32U4.h>
#include "Encoders.h"
#include "Speed_controller.h"
#include "Position_estimation.h"


Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

float time_track = 0;

void SpeedController::Init(void){
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right){
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();
        
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right);
    }
}

boolean SpeedController::Turn(float radians, int direction){ //turn now operatres using Odometry, rather than internally checking counts
    motors.setEfforts(0, 0);
    currentPos = odometry.ReadPose(); 
    float target;

    if(direction){
        target =  currentPos.THETA + radians;
    } else{
        target =  currentPos.THETA - radians;
    }

    if(direction){
        do {
            Run(-50,50);
            currentPos = odometry.ReadPose(); 
        } while(target > currentPos.THETA);
    } else{
        do {
            Run(50,-50);
            currentPos = odometry.ReadPose(); 
        } while(target < currentPos.THETA);
    }

    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop(){
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}

void SpeedController::constrainAccel(int targetSpeed){
	int currVel = MagneticEncoder.ReadVelocityLeft(); //+ MagneticEncoder.ReadVelocityRight()) / 2; //can just read one, they should basically be the same value
    int decVel = currVel - deltaV; //how much it can increase in speed by
	int incVel = currVel + deltaV; //how much it can decrease in speed by
	int vel = (targetSpeed > incVel) ? (incVel) : ((targetSpeed < decVel) ? (decVel) : (targetSpeed));
    motors.setEfforts(vel, vel);
}
