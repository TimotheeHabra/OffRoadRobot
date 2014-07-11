#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "externalControl.h"

using namespace std;

int main(int argc, char **argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);

    rf.setDefaultConfigFile("config.ini");
    rf.configure("",argc,argv);

    // configure module
    ExampleModule module;

    // instantiate my object and go!!
    return module.runModule(rf);
}

