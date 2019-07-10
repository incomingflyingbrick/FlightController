#include "StepMotor.h"
#include <AccelStepper.h>
#include <Arduino.h>


StepMotor::StepMotor(int dir, int pulse, int upLimit, int downLimit){
    pulsePin = pulse;
    dirPin = dir;
    upLimitPin = upLimit;
    downLimitPin = downLimit;
    stepper = AccelStepper(1,pulsePin,dirPin);// use mode 1,which is motor driver
    stepper.setMaxSpeed(2000);// no more than 2000 for speed
}


void StepMotor::extend(){
    stepper.setSpeed(1000);
}

void StepMotor::retract(){
    stepper.setSpeed(-1000);
}

void StepMotor::runSpeed(){
    if(digitalRead(upLimitPin)==LOW||digitalRead(downLimitPin)==LOW){
        stepper.stop();
        return;
    }
    stepper.runSpeed();
}


