#ifndef MotorControlHelper_H
#define MotorControlHelper_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PreciselyTimed.h>

class MotorControlHelper : public yarp::dev::IWrapper
{
public:
    MotorControlHelper();
    ~MotorControlHelper();

    bool open(yarp::os::Searchable *config);

    // wrapper interface
    bool attach(yarp::dev::PolyDriver *poly);
    bool detach();

    bool attachAll(const yarp::dev::PolyDriverList &device2attach);
    bool detachAll();


    int _axes;
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

private:

    bool                                _verbose;
    yarp::dev::PolyDriver               _remoteControlBoard;

    template <class InterfaceType> bool getInterface(yarp::dev::PolyDriver *driver, InterfaceType *&type, yarp::os::ConstString name);
};

#endif // MotorControlHelper_H
