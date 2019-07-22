#include <Arduino.h>
#include <CmdMessenger.h>

class StarLink
{
    enum
    {
        kSuccess,// Command receiv success
        kEStart,// Engine Start
        kEStop,// Shutdown Engine
        kPData,// Engine chamber pressure
        kTData,// Engine temp
    };
/**
 * cmd和callBack为对应关系
 */
public:
    StarLink(Stream &serial);
    CmdMessenger *cmdMessenger;
    void readDataLoop();
    //********** router only cmd ************
    void cmdSuccessAck();
    void cmdEgineTempData(double tempuratue);
    void cmdEginePressureData(double pressure);
    //********** coordinator only cmd ************
    void cmdEngineStart();
    void cmdEngineStop();
    //********** router only callback ************
    void callBackEStart();
    void callBackEStop();
    //***** coordinator only callback******
    void callBackSuccessAck();
    void callBackETemp();
    void callBackEPressure();

private:
    Stream *dataLink;
    
};