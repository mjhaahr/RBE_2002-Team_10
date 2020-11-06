#include <Arduino.h>
#include "IR_sensor.h"
#include "Sonar_sensor.h"

SonarSensor US;
IRsensor IR;

void setup() {
    IR.Init();
    US.Init();
    Serial.begin(9600);
}

void loop() {
    US.PrintData();
    delay(10);
}