#include <wpi-32u4-library.h>
#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;

const float wheelDiameter = 35.0; //mm 
const int CPR = 1440;
const float wheelCircumference = wheelDiameter * M_PI; //mm
const int reqCountsTable = 1450; //counts for a 90 deg turn for a table in Foisie
const int reqCountsFloor = 1500; //counts for a 90 deg turn for the floor in Foisie

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
    buttonA.waitForButton();
    swingTurn();
}

void swingTurn(){ //right is pivot
  encoders.getCountsAndResetLeft(); //resets Left encoder
  
  motors.setEfforts(50, 0);
  
  for(;;) {
    int leftTravel = encoders.getCountsLeft(); //resets Left encoder
    
    if (abs(reqCountsFloor) <= abs(leftTravel)) { 
      break;
    }
  }
  motors.setEfforts(0,0);
}
