#include <InterfacesHelper.h>

bool Interfaces::attach(yarp::dev::PolyDriver *poly)
{
    bool ret = true;
    ret = ret && getInterface(poly, posCtrl_p,   "IPositionControl");
    ret = ret && getInterface(poly, posCtrl2_p,  "IPositionControl2");

    ret = ret && getInterface(poly, velCtrl_p,   "IVelocityControl");
    ret = ret && getInterface(poly, velCtrl2_p,  "IVelocityControl2");

    ret = ret && getInterface(poly, encoder_p,   "IEncodersTimed");
    ret = ret && getInterface(poly, amplif_p,    "IAmplifierControl");

    ret = ret && getInterface(poly, controlLim_p,"IControlLimits2");
    ret = ret && getInterface(poly, axisInfo_p,  "IAxisInfo");

    ret = ret && getInterface(poly, stamp_p,     "IPreciselyTimed");
    ret = ret && getInterface(poly, calib2_p,    "IControlCalibration2");

    ret = ret && getInterface(poly, trqCtrl_p,   "ITorqueControl");
    ret = ret && getInterface(poly, impCtrl2_p,  "IImpedanceControl");

    ret = ret && getInterface(poly, ctrlMode_p,  "IControlMode");
    ret = ret && getInterface(poly, openLoop_p,  "IOpenLoopControl");
    ret = ret && getInterface(poly, directCtrl_p,"IPositionDirect");
    return ret;
}
