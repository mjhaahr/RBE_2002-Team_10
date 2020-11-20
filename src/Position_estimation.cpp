#include  "Position_estimation.h"
#include "Encoders.h"
#include <math.h>

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
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

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if(time_now - time_prev >= deltaT) //update every 50ms for practical reasons
    {
    	//Unit conversion (mm/s to m/s)
		vL = target_speed_left * 1000;
		vR = target_speed_right * 1000;
    	
        //Velocities added
		rPlusL = vR + vL;
		rMinusL = vR - vL;

        R = l2 * rPlusL / rMinusL;
        w = rMinusL / l;
        V = rPlusL / 2;

		if (target_speed_left == target_speed_right){ //Drive Straight
			xDelta = V * cos(theta) * deltaT / 1000;
			yDelta = V * sin(theta) * deltaT / 1000;
			thetaDelta = 0; //No update to theta
		} else if (target_speed_left == -target_speed_right){ //Point turn, Easy enough to seperate
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
		PrintPose(); //print after update (every 50 ms)
    }
}

