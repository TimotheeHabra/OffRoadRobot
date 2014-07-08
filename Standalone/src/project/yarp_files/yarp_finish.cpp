#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>
#include <yarp/dev/PolyDriverList.h>

using namespace std;

void yarp_finish(void* RobotranYarp_interface)
{
	// here should come termination of yarp
	// - closing port
	// - removing Robotran-Yarp drivers

	cout << "termination of yarp" << endl;

    yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)RobotranYarp_interface;  // convert back into object

    if(controlBoardList == NULL)
        return;

    std::cout <<" controlBoardList->size() is " <<  controlBoardList->size() << std::endl;
    for(int i=0; i < controlBoardList->size(); i++)
    {
        std::cout <<" closing device " << (*controlBoardList)[i]->key << std::endl;

        (*controlBoardList)[i]->poly->close();
    }

    delete controlBoardList;
}

#endif
