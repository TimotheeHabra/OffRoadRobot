#include "externalControl.h"
#include "Helper/InterfacesHelper.h"

#include <stdexcept>      // std::out_of_range
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

ExampleModule::ExampleModule() : _verbose(true)
{
    _period = 0.5f;  // seconds
}

bool ExampleModule::configure(yarp::os::ResourceFinder &rf)
{
    Property config;
    config.fromString(rf.toString());
    cout << "\n\n$$$ROBOTRAN::configure params are " << rf.toString() << std::endl;


    // search for robotName_RT
    if(rf.check("robot") )
    {
        robotName_RT = rf.find("robot").asString();
    }
    else
        robotName_RT =  "OffRoad";

    // search for robotName_RT
    if(rf.check("part") )
    {
        partName = rf.find("part").asString();
    }
    else
        partName =  "front";

    yarp::os::Property prop;
    prop.fromString(rf.toString());

    prop.put("robot", robotName_RT);
    cout << "robot is " << robotName_RT << endl;

    yarp::os::ConstString localPort, remotePort;
    localPort = "/externalControl/" + robotName_RT + "/" + partName;
    remotePort = "/" + robotName_RT + "/" + partName;
    prop.put("device", "remote_controlboard");
    prop.put("local", localPort);
    prop.put("remote", remotePort);

    if(!polyDriverRT_MC.open(prop) )
    {
        std::cout << "error opening front part" << std::endl;
        return false;
    }

    frontRT_MC.attach(&polyDriverRT_MC);
    if(frontRT_MC.posCtrl_p)
        frontRT_MC.posCtrl_p->getAxes(&axesNum);

    std::cout << "encoders values; frontRT_MC->_axes is " << axesNum << std::endl;

    std::cout << "************************" << std::endl;
    std::cout << "connecting to REAL robot " << std::endl;
    yarp::os::Property prop2;
    prop2.fromString(rf.toString());

    // search for robotName_RT
    if(rf.check("other") )
    {
        robotName_Other = rf.find("other").asString();
    }
    else
        robotName_Other =  "realRobot";


    prop.put("robot", robotName_Other);
    cout << "robot is " << robotName_Other << endl;


    localPort = "/externalControl/" + robotName_Other + "/" + partName;
    remotePort = "/" + robotName_Other + "/" + partName;
    prop.put("device", "remote_controlboard");
    prop.put("local", localPort);
    prop.put("remote", remotePort);

    if(!polyDriverOther_MC.open(prop) )
    {
        std::cout << "error opening front part" << std::endl;
        return false;
    }

    int realAxes;
    frontOther_MC.attach(&polyDriverOther_MC);
    if(frontOther_MC.posCtrl_p)
        frontOther_MC.posCtrl_p->getAxes(&realAxes);

    if(realAxes != axesNum)
        std::cout << "\n\nERROR!! number of axes of simulated robot is different from the real one!!" << std::endl;
    else
        std::cout << "\n\nOK!! number of axes of simulated robot is matches with the real one!!" << std::endl;

    return true;
}


// RFmodule functions
double ExampleModule::getPeriod()
{
    return _period;  // seconds
}

bool ExampleModule::setPeriod(double period)
{
    cout << "\n\n$$$ExampleModule::setPeriod" << endl;

    _period = period;  // seconds
    return true;
}

bool ExampleModule::updateModule()
{
    double *encRT = new double[axesNum];
    double *timeRT = new double[axesNum];


    double *encOther = new double[axesNum];
    double *timeOther = new double[axesNum];


    frontRT_MC.encoder_p->getEncodersTimed(encRT, timeRT);
    frontOther_MC.encoder_p->getEncodersTimed(encOther, timeOther);


    std::cout << "encoders values RT\t";
    for(int i=0; i<axesNum;  i++)
    {
        std::cout << encRT[i] << "\t";
    }
    std::cout << std::endl;

    std::cout << "encoders values other\t";
    for(int i=0; i<axesNum;  i++)
    {
        std::cout << encOther[i] << "\t";
    }
    std::cout << std::endl;
    return true;
}

