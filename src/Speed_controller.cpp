#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
}

void SpeedController::Process(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        
        Serial.print(MagneticEncoder.ReadVelocityLeft());
        Serial.print('\t');
        Serial.println(MagneticEncoder.ReadVelocityRight());
    }
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
}