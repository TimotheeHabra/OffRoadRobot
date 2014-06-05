#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

void yarp_finish()
{
	// here should come termination of yarp
	// - closing port
	// - removing Robotran-Yarp drivers

	cout << "termination of yarp" << endl;

}

#endif