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
    return {(med_x.Filter((int) (imu.a.x * 0.061))), (int) (med_y.Filter((int) (imu.a.y * 0.061))), (med_z.Filter((int) (imu.a.z * 0.061)))};
}

void IMU_sensor::PrintAcceleration(void)
{
    IMU_sensor::acceleration_data data = ReadAcceleration();
    snprintf_P(report, sizeof(report),
    PSTR("A: %10d %10d %10d"),
    data.X, data.Y, data.Z);
    //imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report); 
}

boolean IMU_sensor::DetectCollision(void){
	IMU_sensor::acceleration_data data = ReadAcceleration();
	if(abs(data.X) > threshold){ //front on collision only, only need to test X direction
        return 1;
    } else {
    	return 0;
    }
}

boolean IMU_sensor::EndOfRamp(void){
	IMU_sensor::acceleration_data data = ReadAcceleration();
	return 0; //currently not implemented
}