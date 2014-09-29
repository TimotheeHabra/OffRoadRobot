#ifdef YARP

#include "MotorControlHelper.h"

using namespace yarp::os;

template <class InterfaceType>
bool MotorControlHelper::getInterface(yarp::dev::PolyDriver *driver, InterfaceType *&type, ConstString name)
{
    driver->view(type);

    if(type == NULL)
    {
        std::cout << "view of " << name << " from POLYDRIVER FAILED\n";
        return false;
    }
    else
        if(_verbose)
            std::cout << "view of " << name << " from POLYDRIVER done with SUCCESS\n";
    return true;
}

MotorControlHelper::MotorControlHelper() : posCtrl_p(NULL),
                                                     posCtrl2_p(NULL),
                                                     velCtrl_p(NULL),
                                                     velCtrl2_p(NULL),
                                                     encoder_p(NULL),
                                                     amplif_p(NULL),
                                                     controlLim_p(NULL),
                                                     axisInfo_p(NULL),
                                                     stamp_p(NULL),
                                                     calib2_p(NULL),
                                                     trqCtrl_p(NULL),
                                                     impCtrl2_p(NULL),
                                                     ctrlMode_p(NULL),
                                                     openLoop_p(NULL),
                                                     directCtrl_p(NULL),
                                                     _verbose(true)
{ }




/**
  * Specify which analog sensor this thread has to read from.
  */

bool MotorControlHelper::attachAll(const yarp::dev::PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        std::cerr<<"MotorControlHelper: cannot attach more than one device\n";
        return false;
    }

    yarp::dev::PolyDriver * device2attach_p=device2attach[0]->poly;

    if (device2attach_p->isValid())
    {
        attach(device2attach_p);
    }

    return true;
}

bool MotorControlHelper::detachAll()
{
    posCtrl_p = NULL;
    posCtrl2_p = NULL;

    velCtrl_p = NULL;
    velCtrl2_p = NULL;
    encoder_p = NULL;
    amplif_p = NULL;

    controlLim_p = NULL;
    axisInfo_p = NULL;

    stamp_p = NULL;
    calib2_p = NULL;
    trqCtrl_p = NULL;
    impCtrl2_p = NULL;

    ctrlMode_p = NULL;
    openLoop_p = NULL;
    directCtrl_p = NULL;
}

bool MotorControlHelper::attach(yarp::dev::PolyDriver *poly)
{
    bool ret = true;
    ret = ret && getInterface(poly, posCtrl_p,   "IPositionControl");
    ret = ret && getInterface(poly, posCtrl2_p,  "IPositionControl2");

    if(posCtrl_p)
        posCtrl_p->getAxes(&_axes);

    ret = ret && getInterface(poly, encoder_p,   "IEncodersTimed");

    ret = ret && getInterface(poly, velCtrl_p,   "IVelocityControl");
    ret = ret && getInterface(poly, velCtrl2_p,  "IVelocityControl2");

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
    return true;
}

bool MotorControlHelper::detach()
{

}

bool MotorControlHelper::open(yarp::os::Searchable *config)
{
    ConstString partName;
    ConstString prefix = "/";
    Value *partName_p;

    if(config->check("verbose"))
        _verbose = true;

    std::cout << config->toString() << std::endl;

    /*  Look for input data */
    // Check for robotName, if not found, fallback to 'coman'
    ConstString robotName = config->check("robot", Value("robotranModel"),"Getting robot name").asString().c_str();

    // part name is mandatory, if not found fire an error and quit
    if( !config->check("part", partName_p))
    {
        std::cout << "Error: Please insert partName in config file or with --part command line options." << std::endl;
        return false;
    }

    // output port name is optional, by default is comanBasic
    ConstString locPortName = config->check("local",Value("robotTranClientExample"),"Output port name").asString();

    // generate port names, will be used hereafter to configure the polyDriver
    partName = partName_p->asString();
    ConstString portName = prefix + robotName + prefix + partName;
    ConstString localPortName = prefix + locPortName + prefix + partName;

    if(!config->check("embedded", Value(false), "am I inside the robotInterface?").asBool())
    {
        // open the device
        yarp::os::Property options;
        options.clear();
        options.put("device", "remote_controlboard"); // aggiungo parametri necessari
        options.put("local", localPortName);
        options.put("remote", portName);

        std::cout << "remoteControlBoard.open(options) " << options.toString() << std::endl;

        if(!_remoteControlBoard.open(options) )
        {
            std::cout << "Error opening remoteControlBoard!!" << std::endl;
            return false;
        }

        attach(&_remoteControlBoard);

        // Initting interfaces
        if(!posCtrl_p->getAxes(&_axes) )
        {
            std::cout << "ERROR: Unable to get axis number, closing!" << std::endl;
            return false;
        }
        else
            std::cout << "GOOD: number of axis is " << _axes << std::endl;
    }
    else
    {
        std::cout << "Example is embedded!!";
    }
    return true;
}


MotorControlHelper::~MotorControlHelper(){}


#endif
