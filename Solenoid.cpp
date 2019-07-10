#include "Solenoid.h"
#include <Arduino.h>

void Solenoid::openValve(){
    digitalWrite(controlPin,HIGH);
}

void Solenoid::closeValve(){
    digitalWrite(controlPin,LOW);
}