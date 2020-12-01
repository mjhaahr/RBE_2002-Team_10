#include <Arduino.h>
#include "Behaviors.h"
//#include "AccelControl.h"

Behaviors followWaypoints;
//AccelControl accelControl;

void setup() {
    followWaypoints.Init();
    //accelControl.Init();
}

void loop() {
    followWaypoints.Run();
    //accelControl.Run();
}