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

void updateDataToYarp()
{
	// here should come the update from simulator to yarp
	// write the new sensor state computed by simulation
	// - sensor_driver.onUpdate()

	//cout << "update data to yarp " << endl;
}

#endif