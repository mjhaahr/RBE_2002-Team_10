#include <wpi-32u4-library.h>
#include <Romi32U4.h>

enum STATE{
  IDLE,
  DRIVE
};

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
STATE state;

void setup() {
  Serial.begin(9600);
  state = IDLE;
}

void loop() {
  switch(state){
    case IDLE:
      if(buttonA.getSingleDebouncedRelease()){
        state = DRIVE;
        motors.setEfforts(0,0);
      } else {
        state = IDLE;
        motors.setEfforts(0,0);
      }
      break;
      
    case DRIVE:
      if(buttonA.getSingleDebouncedRelease()){
        state = IDLE;
        motors.setEfforts(0,0);
      } else {
        state = DRIVE;
        motors.setEfforts(50,50);
      }
      break;
      
    default:
      state = IDLE;
      motors.setEfforts(0,0);
      break;
  }
  delay(50); //just to prevent very quick loops
}
