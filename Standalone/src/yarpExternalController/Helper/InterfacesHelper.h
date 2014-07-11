#ifndef _INTERFACE_HELPER_H_
#define _INTERFACE_HELPER_H_


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

class Interfaces
{
public:
    // interfaces
    yarp::dev::IPositionControl         *posCtrl_p;
    yarp::dev::IPositionControl2        *posCtrl2_p;

    yarp::dev::IVelocityControl         *velCtrl_p;
    yarp::dev::IVelocityControl2        *velCtrl2_p;

    yarp::dev::IEncodersTimed           *encoder_p;
    yarp::dev::IAmplifierControl        *amplif_p;

    yarp::dev::IControlLimits2          *controlLim_p;
    yarp::dev::IAxisInfo                *axisInfo_p;

    yarp::dev::IPreciselyTimed          *stamp_p;
    yarp::dev::IControlCalibration2     *calib2_p;
    yarp::dev::ITorqueControl           *trqCtrl_p;
    yarp::dev::IImpedanceControl        *impCtrl2_p;

    yarp::dev::IControlMode             *ctrlMode_p;
    yarp::dev::IOpenLoopControl         *openLoop_p;
    yarp::dev::IPositionDirect          *directCtrl_p;

    bool attach(yarp::dev::PolyDriver *poly);
};

template <class InterfaceType>
bool getInterface(yarp::dev::PolyDriver *driver, InterfaceType *&type, yarp::os::ConstString name)
{
    driver->view(type);

    if(type == NULL)
    {
        std::cout << "view of " << name << " from POLYDRIVER FAILED\n";
        return false;
    }
    else
            std::cout << "view of " << name << " from POLYDRIVER done with SUCCESS\n";
    return true;
}

#endif   // _INTERFACE_HELPER_H_
