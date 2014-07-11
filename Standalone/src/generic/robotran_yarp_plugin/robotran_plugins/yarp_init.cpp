#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriverList.h>

#include <iostream>

using namespace std;

// here should come initialization of yarp
// - getting configuration files
// - creating desired Robotran-Yarp drivers
// - opening ports

void* yarp_init(void)
{
	cout << "initialization of yarp interface" << endl;

    yarp::dev::PolyDriverList       *p_controlBoardList = NULL;
    bool verbose = true;


    yarp::os::Network               _yarp;
    yarp::dev::PolyDriver           _wrapper;
    yarp::dev::IMultipleWrapper     *_iWrap = NULL;

    // init YARP and instantiate yarp device driver
    if( !_yarp.checkNetwork() ) {
        std::cerr << "RobotranYarpControlBoard::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        // either return something invalid (NULL) and check the value in the main_simulation or directly throw an exit here.
        exit(0);
    }

    // Add the robotranControlboard device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::RobotranYarpMotionControl>
                                      ("robotranMotionControl", "controlboardwrapper2", "RobotranYarpMotionControl"));

    // Add the robotranControlboard device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::RobotranYarpForceTorqueDriver>
                                      ("robotranForceTorqueSensor", "analogServer", "RobotranYarpForceTorqueDriver"));


    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);

    yarp::os::ConstString fileNameWithPath = rf.findFileByName("RoboTran.ini");
    if(fileNameWithPath == "")
    {
        std::cout << "Default config file ´RoboTran.ini´  was not found" << std::endl;
        return NULL;
    }

    p_controlBoardList = new yarp::dev::PolyDriverList;

    yarp::os::Property p;
    p.fromConfigFile(fileNameWithPath);

    if(verbose)
        std::cout << "\n\n config param are\n " << p.toString() << std::endl;

    bool found = p.check("GENERAL");
    if(!found)
    {
        std::cout << "GENERAL section not found" << std::endl;
        return NULL;
    }

    yarp::os::Bottle &general = p.findGroup("GENERAL");

    if(general.check("verbose"))
        verbose = true;

    yarp::os::ConstString robotName = general.find("robot").asString();
    if(!general.check("types"))
    {
        std::cout << "ERROR: ´types´ list was not found in the GENERAL group" << std::endl;
        return NULL;
    }

    yarp::os::Bottle * types = general.find("types").asList();
    if(verbose)
        std::cout << "\nFound following types (" << types->toString() << ")" << std::endl;

    for(int typeIndex=0; typeIndex < types->size(); typeIndex++)
    {
        yarp::os::ConstString typeName = types->get(typeIndex).asString();
        if(!general.check(typeName))
        {
            std::cout << "ERROR: I was expecting the keyword " << typeName << " followed by a list of entries like" << std::endl;
            std::cout << typeName << " (foo1 foo2 foo3)" << std::endl;
            return NULL;
        }

        if(!general.find(typeName).isList() )
        {
            std::cout << "ERROR: the keyword " << typeName << " is not a list. Correct syntax is like" << std::endl;
            std::cout << typeName << " (foo1 foo2 foo3)  maybe the ´()´ brackets are missing" << std::endl;
            return NULL;
        }

        yarp::os::Bottle * entryList = general.find(typeName).asList();


        if(verbose)
            std::cout << "type ´" << typeName << "´ has the following entries (" << entryList->toString() << ")" << std::endl;

        for(int entryIndex=0; entryIndex < entryList->size(); entryIndex++)
        {
            yarp::os::ConstString entryName = entryList->get(entryIndex).asString();
            if(!p.check(entryName))
            {
                std::cout << "cannot find device ´" << entryName << "´ referenced in the type list ´" << typeName << "´" << std::endl;
                return NULL;
            }

            yarp::os::Bottle &entryParams = p.findGroup(entryName);
            if(verbose)
                std::cout << "Entry ´" << entryName << "´ has the following parameters \n\t" << entryParams.toString() << "\n" << std::endl;

            // Looking for other files referenced by the main one

            yarp::os::Property tmpProp(entryParams.toString().c_str());

            while(tmpProp.check("file"))
            {
                yarp::os::ResourceFinder subRF;
                subRF.setVerbose(false);
                yarp::os::ConstString  subfileName = tmpProp.find("file").asString();
                if(verbose)
                    std::cout << "found subfile " << subfileName << std::endl;

                yarp::os::ConstString fileNameWithPath = subRF.findFileByName(subfileName);
                if(fileNameWithPath == "")
                {
                    std::cout << "sub config file ´" << subfileName << "´  was not found" << std::endl;
                    return NULL;
                }
                yarp::os::Property p2;
                p2.fromConfigFile(fileNameWithPath);

                tmpProp.unput("file");
                tmpProp.fromString(p2.toString(), false);
            }
            yarp::dev::PolyDriver *tmp = new yarp::dev::PolyDriver;
            tmpProp.put("robot", robotName);
            tmp->open(tmpProp);


            if (!tmp->isValid())
            {
                fprintf(stderr, "driver %s did not open\n", entryName.c_str());
            }
            else
            {
                printf("controlBoard opened correctly\n");

                yarp::dev::PolyDriverDescriptor newDev( tmp, entryName.c_str());
                p_controlBoardList->push(newDev);
            }
        }
    }
    return (void*) p_controlBoardList;
}

#endif
