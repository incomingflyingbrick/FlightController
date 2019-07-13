#include "StarLink.h"
#include <CmdMessenger.h>

StarLink::StarLink(Stream &serial)
{
    dataLink = &serial;
    cmdMessenger = cmdMessenger(dataLink);
}

// call in a loop
void StarLink::readDataLoop()
{
    cmdMessenger->feedinSerialData();
}

// router use only, for send success ack to coordinator
void StarLink::cmdSuccessAck(String type)
{
    cmdMessenger->sendCmd(kSuccess, "S:" + type);
}
// coordinator use only cmd
void StarLink::cmdEngineStart()
{
    cmdMessenger->sendCmd(kEStart);
}

void StarLink::cmdEngineStop()
{
    cmdMessenger->sendCmd(kEStop);
}

// router use only, for send success ack to coordinator
void StarLink::callBackEStart() {
    cmdSuccessAck("EStart");
}
void StarLink::callBackEStop() {
    cmdSuccessAck("EStop");
}