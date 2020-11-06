#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

void IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
    //assignment 1.1
    //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
    return 0;
}