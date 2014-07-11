#ifndef basicControl_H
#define basicControl_H

#include <map>
#include <list>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/RateThread.h>

// Include the Helper for Motor Control
//#include "RobotHelper.h"
#include "MotorControlHelper.h"

class basicControl:  public yarp::os::RateThread,
                     public yarp::dev::DeviceDriver,
                     public yarp::dev::IMultipleWrapper
{
public:
    basicControl();

    // Yarp device driver functions
    bool open(yarp::os::Searchable &conf);
    void run();                                //!  Loop thread function

    MotorControlHelper          *frontMC;
    MotorControlHelper          *rearMC;

    std::list<std::string>::const_iterator partNameListIterator;
    std::list<std::string>      partNameList;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList &device2attach);
    bool detachAll();

private:
    double                              _period;   // seconds
    bool                                _verbose;
};

#endif // basicControl_H
