#include <Arduino.h>
#include <CmdMessenger.h>

class StarLink
{

    enum
    {
        kSetLed, // Command to request led to be set in specific state
        kStatus, // Command to report status
        kSuccess,// Command receiv success
        kEStart,// Engine Start
        kEStop,// Shutdown Engine
    };
/**
 * cmd和callBack为对应关系
 */
public:
    StarLink(Stream &serial);
    CmdMessenger *cmdMessenger;
    void readDataLoop();
    //**********router only cmd************
    void cmdSuccessAck();
    //**********coordinator only cmd************
    void cmdEngineStart();
    void cmdEngineStop();
    //**********router only call back************
    void callBackEStart();
    void callBackEStop();


private:
    Stream *dataLink;
    
};