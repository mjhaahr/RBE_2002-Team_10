#include  "Position_estimation.h"
#include "Encoders.h"

Encoder RomiEncoders;

float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    x = 0;
    y = 0;
    theta = 0;
    time_prev = millis();
}

void Position::Stop(void)
{
    x = 0; 
    y = 0;
    theta = 0;
    time_prev = millis();
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right) //better odometry code
{
    time_now = millis();
    if(time_now - time_prev >= deltaT) //update every 50ms for practical reasons
    {
    	//Unit conversion (mm/s to m/s)
		vL = RomiEncoders.ReadVelocityLeft() / 1000;
		vR = RomiEncoders.ReadVelocityRight() / 1000;
    	
        //Velocities added
		rPlusL = vR + vL;
		rMinusL = vR - vL;

        R = l2 * rPlusL / rMinusL;
        w = rMinusL / l;
        V = rPlusL / 2;

		if ((target_speed_left == target_speed_right) || (abs(rMinusL) <= eqThreshold)){ //Drive Straight with tolerance
			xDelta = V * cos(theta) * deltaT / 1000;
			yDelta = V * sin(theta) * deltaT / 1000;
			thetaDelta = 0; //No update to theta
		
		} else if ((target_speed_left == -target_speed_right) || (abs(rPlusL) <= eqThreshold)){ //Point turn, Easy enough to seperate
            xDelta = 0;
			yDelta = 0;
			thetaDelta = w * deltaT / 1000; //duration in seconds
		} else { //Curved path
			thetaDelta = w * deltaT / 1000;
			xDelta = (R * sin(theta + thetaDelta)) -(R * sin(theta));
			yDelta = (R * cos(theta)) - (R * cos(theta + thetaDelta));
		}

        x += xDelta;
        y += yDelta;
		theta += thetaDelta;
    }
}