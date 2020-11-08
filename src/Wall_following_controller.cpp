#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

IRsensor SharpIR;
SonarSensor HCSR04;
void WallFollowingController::Init(void) {
    SharpIR.Init();
    HCSR04.Init();
}

float WallFollowingController::Process(float target_distance) {
    //assignment 2: write a PD controller that outputs speed as a function of distance error
    prev_e_distance = E_distance; //reset previous error
    E_distance = target_distance - SharpIR.ReadData(); //might need to flip terms, not sure how that will manifest
    float diff = E_distance - prev_e_distance;
    float speed = constrain(E_distance * Kd + diff * Kp, -50, 50);
    return speed;
}