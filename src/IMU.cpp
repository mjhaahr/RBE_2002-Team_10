#include "IMU.h" 
#include <Romi32U4.h>
#include <LSM6.h>
#include "Median_filter.h"

LSM6 imu;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

void IMU_sensor::Init(void)
{
    Wire.begin();
    if (!imu.init())
    {
        while(1)
        {
            Serial.println("Failed to detect the LSM6.");
            delay(100);
        }
    }
    imu.setFullScaleAcc(imu.ACC_FS2);
    imu.enableDefault();
    
    med_x.Init();
    med_y.Init();
    med_z.Init();
}

IMU_sensor::acceleration_data IMU_sensor::ReadAcceleration(void)
{
    imu.read();
    return {(int) (med_x.Filter(imu.a.x) * 0.061), (int) (med_y.Filter(imu.a.y) * 0.061), (int) (med_z.Filter(imu.a.y) * 0.061)};
}

void IMU_sensor::PrintAcceleration(void)
{
    acceleration_data data = ReadAcceleration();
    snprintf_P(report, sizeof(report),
    PSTR("A: %10d %10d %10d"),
    data.X, data.Y, data.Z);
    //imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report); 
}

boolean IMU_sensor::DetectCollision(void){
	acceleration_data data = ReadAcceleration();
	if(abs(data.X) > threshold){ //front on collision only, only need to test X direction
        return 1;
    } else {
    	return 0;
    }
}

boolean IMU_sensor::EndOfRamp(void){
	acceleration_data data = ReadAcceleration();
	switch (ramp_state) {
		case BEFORE:
			if (data.Z < crossover){ //started going up, accel in Z direction decreases
				ramp_state = UP;
				return 0;
			} else {
				ramp_state = BEFORE;
				return 0;
			}
			break;
		case UP:
			if (data.Z > crossover){ //flattened off, accel in Z direction increases
				ramp_state = ON;
				return 0;
			} else {
				ramp_state = UP;
				return 0;
			}
			break;
		case ON:
			if (data.Z < crossover){ //started going down, accel in Z direction decreases
				ramp_state = DOWN;
				return 0;
			} else {
				ramp_state = ON;
				return 0;
			}
			break;
		case DOWN:
			if (data.Z > crossover){ //off ramp, accel in Z direction increases
				ramp_state = OFF;
				return 1;
			} else {
				ramp_state = DOWN;
				return 0;
			}
			break;
		case OFF:
			return 1;
			break;
		default:
			ramp_state = BEFORE;
			return 0;
			break;
	}
}