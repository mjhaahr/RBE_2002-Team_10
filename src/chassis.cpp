#include <Romi32U4.h>
#include "chassis.h"

float RomiChassis::SpeedLeft(void) {
    // !!! ATTENTION !!!
    // Assignment 1
    int num_count = count_left - prev_count_left;
    float ang_vel = ((float) num_count) / interval; //angular velocity in counts per millisecond
    float tan_vel = (ang_vel * C_wheel) / N_wheel * 1000; //tangnetial velocity in mm/s
    return tan_vel; //[mm/s]
}

float RomiChassis::SpeedRight(void) {
    // !!! ATTENTION !!!
    // Assignment 1
    int num_count = count_right - prev_count_right;
    float ang_vel = ((float) num_count) / interval; //angular velocity in counts per millisecond
    float tan_vel = (ang_vel * C_wheel) / N_wheel * 1000; //tangnetial velocity in mm/s
    return tan_vel; //[mm/s]
}

void RomiChassis::UpdateEffortDriveWheels(int left, int right) { 
    motors.setEfforts(left,right);
}

void RomiChassis::UpdateEffortDriveWheelsPI(int target_speed_left, int target_speed_right) {
    // !!! ATTENTION !!!
    // Assignment 2
    {
        float u_left = 0;
        float u_right = 0;
        motors.setEfforts(u_left,u_right);
    }
}

void RomiChassis::SerialPlotter(float a, float b, float c, float d) {
    // !!! ATTENTION !!!
    // USE this function for assignment 3!
    Serial.print(a);
    Serial.print('\t');
    Serial.print(b);
    Serial.print('\t');
    Serial.print(c);
    Serial.print('\t');
    Serial.print(d);
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
        UpdateEffortDriveWheels(target_left, target_right);
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