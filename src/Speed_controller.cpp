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

boolean SpeedController::MoveToPosition(float target_x, float target_y){
    E_dist = 0;
    E_theta = 0;
    theta_last = 0;
    //initial turn to position
    currentPos = odometry.ReadPose(); 
    float yError = target_y - currentPos.Y;
    float xError = target_x - currentPos.X;
    float turnAngle = atan((yError) / (xError)) - currentPos.THETA;
    int turnDeg = (int) (turnAngle * 180 / PI);
    if (xError < 0){
        turnDeg += 180;
    }
    Turn(turnDeg, 1);

    do {
        currentPos = odometry.ReadPose();
        error_distance = sqrt(pow((target_x - currentPos.X), 2) + pow((target_y - currentPos.Y), 2));
        error_theta = atan((target_y - currentPos.Y) / (target_x - currentPos.X)) - currentPos.THETA;
        E_dist += error_distance;
        E_theta += error_theta;
        Serial.print(error_distance);
        Serial.print('\t');
        Serial.print(error_theta);
        Serial.print('\t');
        Serial.print('\t');

        T_diff = error_theta - theta_last;

        float speedRight = KpD * error_distance + KiD * E_dist + KpT * error_theta + KiT * E_theta + KdT * T_diff;
        float speedLeft = KpD * error_distance  + KiD * E_dist - KpT * error_theta - KiT * E_theta - KdT * T_diff;

        theta_last = error_theta;
        
        Serial.print(speedLeft);
        Serial.print('\t');/*
        Serial.print(speedLeft);
        Serial.print('\t');
        Serial.print('\t');
        
        speedRight = constrainAccel(speedRight);
        speedLeft = constrainAccel(speedLeft);

        Serial.print(speedRight);
        Serial.print('\t');
        */
       Serial.println(speedRight);
       Run(speedLeft, speedRight);
    
    } while (error_distance >= distanceTolerance); //define a distance criteria that lets the robot know that it reached the waypoint.
    motors.setEfforts(0, 0);
    Turn(turnDeg, 0);
    return 1;
}

boolean SpeedController::Turn(int degree, int direction){
    motors.setEfforts(0, 0);
    int turns = counts*(degree/180.0);
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns){
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time){
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time){ //in mm/s and s
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop(){
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}

float SpeedController::constrainAccel(float targetSpeed){
	float currVel = (MagneticEncoder.ReadVelocityLeft() + MagneticEncoder.ReadVelocityRight()) / 2; //can just read one, they should basically be the same value
    float decVel = currVel + deltaV; //how much it can increase in speed by
	float incVel = currVel - deltaV; //how much it can decrease in speed by
	float vOut = (currVel > incVel) ? (incVel) : ((currVel < decVel) ? (decVel) : (currVel));
	return vOut;
}
