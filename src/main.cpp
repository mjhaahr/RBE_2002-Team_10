#include <Arduino.h>
#include <Romi32U4.h>
#include "Behaviors.h"

Behaviors collisionBehavior;

void setup() {
  collisionBehavior.Init();
}

void loop() {
  collisionBehavior.Run();
}