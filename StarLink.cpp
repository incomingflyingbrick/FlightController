#include "StarLink.h"
#include <CmdMessenger.h>

StarLink::StarLink(Stream &serial)
{
    cmdMessenger = new CmdMessenger(serial);
}

// call in a loop
void StarLink::readDataLoop()
{
    cmdMessenger->feedinSerialData();
}

//***********Router Cmd**************
// router use only, for send success ack to coordinator
void StarLink::cmdSuccessAck(String type)
{
    cmdMessenger->sendCmd(kSuccess, "S:" + type);
}

void StarLink::cmdEgineTempData(double tempurature){
    cmdMessenger->sendCmd(kTData,tempurature);
}

void StarLink::cmdEginePressureData(double pressure){
     cmdMessenger->sendCmd(kPData,pressure);
}


//***********Coordinator Cmd**************
// coordinator use only cmd
void StarLink::cmdEngineStart()
{
    cmdMessenger->sendCmd(kEStart);
}

void StarLink::cmdEngineStop()
{
    cmdMessenger->sendCmd(kEStop);
}

//***********Router callback**************
// router use only, for send success ack to coordinator
void StarLink::callBackEStart()
{
    cmdSuccessAck("EStart");
}
void StarLink::callBackEStop()
{
    cmdSuccessAck("EStop");
}

//***********Coordinator callback**************
// coordinator callback when receive succes msg, send data to PC
void StarLink::callBackSuccessAck()
{
    // TODO sending PC to PC
}

void StarLink::callBackETemp(){
    // TODO sending temp data to PC
}
void StarLink::callBackEPressure(){
    // TODO sending pressure data to PC
}
