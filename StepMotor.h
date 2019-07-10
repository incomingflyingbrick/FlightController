#include <AccelStepper.h>

class StepMotor
{
public:
    StepMotor(int dir, int pulse, int upLimitPin, int downLimitPin);
    void extend();
    void retract();
    void runSpeed();

private:
    AccelStepper stepper;
    int dirPin;
    int pulsePin;
    int upLimitPin;
    int downLimitPin;
};