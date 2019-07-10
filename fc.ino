#include "IMUService.h"

IMUService imuService(25);

void setup()
{
    Serial.begin(9600);
    imuService.prepare();
}

void loop()
{
    imuService.loopIMU();
}
