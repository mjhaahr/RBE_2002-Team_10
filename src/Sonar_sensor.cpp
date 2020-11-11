#include <Romi32U4.h>
#include "Sonar_sensor.h"

static SonarSensor *isr_rangefinder; //singeton object for the ISR to access becuase it is static (pointer)


void SonarSensor::Init(void)
{
    pinMode(pin_TRIG,OUTPUT);
    pinMode(pin_ECHO, INPUT);   
    attachInterrupt(digitalPinToInterrupt(pin_ECHO), SonarSensor::ultrasonicISR, CHANGE); //attachs the ISR to the interupt
    isr_rangefinder = this; //declares the ISR rangefinder object as pointing to the initialized object (this)

}

void SonarSensor::PrintData(void)
{
    Serial.println(ReadData());
}

float SonarSensor::ReadData(void)
{
    float distance = (pulseTime + 34.88) / 56.274;
    return distance;  //Matthew Equation
}

void SonarSensor::loop(void) {
    digitalWrite(pin_TRIG, LOW); 
    delayMicroseconds(2); 
    digitalWrite(pin_TRIG, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(pin_TRIG, LOW);
}

void SonarSensor::pulseHandler() {
    if (digitalRead(pin_ECHO) == 1) { //if value of the echoPin is 1 it sets the start time of the pulse
        startTime = micros();
    } else { //if the value of echoPin is not 1 it ends the pulse and calculates the pulse time
        pulseTime = micros() - startTime;
    }
}

void SonarSensor::ultrasonicISR(){
    isr_rangefinder->pulseHandler(); //calls specific object instance of the pulseHandler because ultrasonicISR is static
}