#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

void yarp_finish(void* RobotranYarp_interface)
{
	// here should come termination of yarp
	// - closing port
	// - removing Robotran-Yarp drivers

	cout << "termination of yarp" << endl;

	yarp::dev::PolyDriver *p_controlBoard;
	p_controlBoard = (yarp::dev::PolyDriver*)RobotranYarp_interface;  // convert back into object

	delete(p_controlBoard);

}

#endif