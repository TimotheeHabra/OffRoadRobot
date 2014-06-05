#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

void yarp_init()
{
	// here should come initialization of yarp
	// - getting configuration files
	// - creating desired Robotran-Yarp drivers
	// - opening ports

	cout << "initialization of yarp interface" << endl;

}

#endif