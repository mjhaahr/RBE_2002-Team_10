#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_Follower.h"
#include "IR_sensor.h"

IRsensor SharpIR;

void WallFollower::Init(void) {
    SharpIR.Init();
}

float WallFollower::Process(float target_distance) {
    float target_adjusted = target_distance / angleCompensation; // prevent repeated floating point math
    prev_e_distance = E_distance; //reset previous error
    E_distance = target_adjusted - SharpIR.ReadData(); //might need to flip terms, not sure how that will manifest

    float diff = E_distance - prev_e_distance;
    float speed = constrain(E_distance * Kd + diff * Kp, -50, 50); // Constrain speed within +- 50mm/s
    return speed;
}