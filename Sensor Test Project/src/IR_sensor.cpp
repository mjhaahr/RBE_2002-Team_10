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
    unsigned int adc = analogRead(pin_IR);
    float volts = ((float) adc) * conversionFactor;
    return volts;
}