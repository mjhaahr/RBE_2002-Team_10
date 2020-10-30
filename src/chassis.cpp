#include <Romi32U4.h>
#include "chassis.h"

float RomiChassis::SpeedLeft(void) {
    // !!! ATTENTION !!!
    // Assignment 1
    int num_count = count_left - prev_count_left;
    float ang_vel = ((float) num_count) / interval; //angular velocity in counts per millisecond
    left_speed = (ang_vel * C_wheel) / N_wheel * 1000; //tangential velocity in mm/s
    return left_speed; //[mm/s]
}

float RomiChassis::SpeedRight(void) {
    // !!! ATTENTION !!!
    // Assignment 1
    int num_count = count_right - prev_count_right;
    float ang_vel = ((float) num_count) / interval; //angular velocity in counts per millisecond
    right_speed = (ang_vel * C_wheel) / N_wheel * 1000; //tangential velocity in mm/s
    return right_speed; //[mm/s]
}

void RomiChassis::UpdateEffortDriveWheels(int left, int right) { 
    motors.setEfforts(left,right);
}

void RomiChassis::UpdateEffortDriveWheelsPI(int target_speed_left, int target_speed_right) {
    // !!! ATTENTION !!!
    // Assignment 2
    error_left = target_speed_left - SpeedLeft();
    error_right = target_speed_right - SpeedRight();
    E_left += error_left;
    E_right += error_right;

    u_left = error_left * Kp + E_left * Ki;
    u_right = error_right * Kp + E_right * Ki;
    motors.setEfforts(u_left,u_right);
}

void RomiChassis::SerialPlotter() {
    // !!! ATTENTION !!!
    // USE this function for assignment 3!
    Serial.print(left_speed);
    Serial.print(" |\t");
    Serial.print(right_speed);
    Serial.print(" |\t");
    Serial.print(error_left);
    Serial.print(" |\t");
    Serial.print(error_right);
    Serial.print(" |\t");
    Serial.print(u_left);
    Serial.print(" |\t");
    Serial.print(u_right);
    Serial.println();
}

void RomiChassis::MotorControl(void) {
    uint32_t now = millis();
    if(now - last_update >= interval) {    
        prev_count_left = count_left;
        prev_count_right = count_right;
        count_left = encoders.getCountsLeft();
        count_right = encoders.getCountsRight();
        previous_time = millis();
        UpdateEffortDriveWheelsPI(target_left, target_right);
        last_update = now;
    }
}

void RomiChassis::StartDriving(float left, float right, uint32_t duration) {
  target_left = left; target_right = right;
  start_time = millis();
  last_update = start_time;
  end_time = start_time + duration; //fails at rollover
  E_left = 0;
  E_right = 0;
}

bool RomiChassis::CheckDriveComplete(void) {
  return millis() >= end_time;
}

void RomiChassis::Stop(void) {
  target_left = target_right = 0;
  motors.setEfforts(0, 0);
}