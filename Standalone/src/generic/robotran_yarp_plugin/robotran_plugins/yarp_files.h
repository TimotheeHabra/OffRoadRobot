#ifdef YARP

#ifndef _YARP_FILES_H_
#define _YARP_FILES_H_

/////////////////////////////////////
//function called by C and implemented in C++ (must only use C compatible input/output types)
/////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

	#include <MBSdataStruct.h>

	void* yarp_init(void);  //void* to be compatible with C type. It is actually an pointer to an object
    void updateDataFromYarp(void* RobotranYarp_interface, MBSdataStruct * MBSdata);
	void updateDataToYarp(void* RobotranYarp_interface, const MBSdataStruct * MBSdata);
	void yarp_finish(void* RobotranYarp_interface);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////
//function used by C++ code only
/////////////////////////////////////
#ifdef __cplusplus

	// add necessary yarp libraries here
	#include <yarp/dev/Wrapper.h>
	#include <yarp/dev/PolyDriverList.h>

    #include <ForceTorqueDriver.h>
	#include <RobotranMotionControlBoard.h>

#endif

#endif
#endif

