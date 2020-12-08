#include <Arduino.h>
#include "Behaviors.h"

Behaviors FinalProject;

void setup(){
    Serial.begin(9600);
    FinalProject.Init();
}

void loop(){
    FinalProject.Run();
}
