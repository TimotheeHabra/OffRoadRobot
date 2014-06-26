#ifdef YARP

#ifndef _YARP_FILES_H_
#define _YARP_FILES_H_

/////////////////////////////////////
//function called by C and implemented in C++ (must only use C compatible input/output types)
/////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

	void yarp_init(void);
	void updateDataFromYarp(void);
	void updateDataToYarp(void);
	void yarp_finish(void);

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

	#include <RobotranMotionControlBoard.h>

#endif

#endif
#endif

