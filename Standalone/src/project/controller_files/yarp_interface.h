#ifdef YARP


#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

/////////////////////////////////////
//function called by C and implemented in C++ (must only use C compatible input/output types)
/////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

	void getControllerInput_Yarp(void);
	void writeControllerOutput_Yarp(void);

#ifdef __cplusplus
}
#endif

/////////////////////////////////////
//function used by C++ code only
/////////////////////////////////////
#ifdef __cplusplus

	// add necessary yarp libraries here

#endif

#endif
#endif
