#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

void updateDataFromYarp()
{
	// here should come the update from yarp to the simulator
	// write the new references (torque, position, speed ...) desired by the ControlInterface
	// - get references
	// - feed low leved PID controllers with the ref

	//cout << "update data from yarp " << endl;
}

// here should come the update from simulator to yarp
// write the new sensor state computed by simulation
// - sensor_driver.Update()
void updateDataToYarp(void* RobotranYarp_interface, const MBSdataStruct * MBSdata)
{

	yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)RobotranYarp_interface;  // convert back into object

	yarp::dev::RobotranYarpMotionControl* 		RobotranControlBoardType 	= NULL;
	yarp::dev::RobotranYarpForceTorqueDriver* 	RobotranForceTorqueType 	= NULL;

	for(int i=0; i < controlBoardList->size(); i++)
    {
        (*controlBoardList)[i]->poly->view(RobotranControlBoardType);
        if(RobotranControlBoardType) 
        {
        	//printf("I found a control Board driver at %d \n\n", i);
        	RobotranControlBoardType->updateToYarp(MBSdata);
    	}

    	//(*controlBoardList)[i]->poly->view(RobotranForceTorqueType);
    	//else if(RobotranForceTorqueType)
    	//{
    	//	printf("I found a force torque driver at %d \n\n", i);
    	//}

    }
}

#endif