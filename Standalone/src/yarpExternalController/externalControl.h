#ifndef EXAMPLEMODULE_H
#define EXAMPLEMODULE_H

#include <map>
#include <list>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/ConstString.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/IVelocityControl2.h>

#include <Helper/InterfacesHelper.h>


class ExampleModule: public yarp::os::RFModule,
                     public yarp::dev::DeviceDriver
{
public:
    ExampleModule();

    yarp::os::ConstString robotName_RT;
    yarp::os::ConstString robotName_Other;
    yarp::os::ConstString partName;

    // Yarp RF Module core functions
    bool    setPeriod(double period);
    double  getPeriod();
    bool    updateModule();                                //!  Loop thread function
    bool    configure(yarp::os::ResourceFinder &rf);       //! Config function, called at the module start-up.

    yarp::dev::PolyDriver    polyDriverRT_MC;
    Interfaces               frontRT_MC;

    yarp::dev::PolyDriver    polyDriverOther_MC;
    Interfaces               frontOther_MC;

    int axesNum;

private:
    double                              _period;   // seconds
    bool                                _verbose;
};

#endif // EXAMPLEMODULE_H
