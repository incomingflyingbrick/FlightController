#include <Arduino.h>

class Solenoid
{
public:
    
    Solenoid(int ioPin){
        controlPin = ioPin;
        pinMode(controlPin,OUTPUT);
    }
    void openValve();

    void closeValve();

private:

    int controlPin;
};
