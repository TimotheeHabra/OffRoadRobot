#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

	// here should come initialization of yarp
	// - getting configuration files
	// - creating desired Robotran-Yarp drivers
	// - opening ports

void yarp_init()
{

	cout << "initialization of yarp interface" << endl;

	yarp::os::Network               _yarp;
    yarp::dev::PolyDriver           _wrapper;
    yarp::dev::IMultipleWrapper     *_iWrap;
    yarp::dev::PolyDriver           _controlBoard;


//    // init YARP and instantiate yarp device driver
//    if( !_yarp.checkNetwork() ) {
//        std::cerr << "GazeboYarpControlBoard::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
//        // either return something invalid (NULL) and check the value in the main_simulation or directly throw an exit here.
//        exit(0);
//    }

    // Add the robotranControlboard device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::RobotranYarpMotionControl>
                                      ("robotranMotionControl", "controlboardwrapper2", "RobotranYarpMotionControl"));

    yarp::os::Property parameters;
    parameters.put("device", "robotranMotionControl");
    parameters.put("joints", 23);
//    yarp::os::Bottle pidGroup;
    yarp::os::Property &pid_GROUP = parameters.addGroup("PID");

    yarp::os::Bottle kp;
    yarp::os::Bottle &kplist = kp.addList();
    kplist.addString("kp");
    kplist.addDouble(10);
    kplist.addDouble(20);

    yarp::os::Bottle kd;
    yarp::os::Bottle &kdlist = kd.addList();
    kdlist.addString("kd");
    kdlist.addDouble(100);
    kdlist.addDouble(200);

    std::cout << "kp " << kp.toString() << std::endl;
    std::cout << "kd " << kd.toString() << std::endl;

    pid_GROUP.fromString(kp.toString(), false);
    pid_GROUP.fromString(kd.toString(), false);

//    parameters.addGroup();
    _controlBoard.open(parameters);

    if (!_controlBoard.isValid())
        fprintf(stderr, "controlBoard did not open\n");
    else
        printf("controlBoard opened correctly\n");

}

#endif