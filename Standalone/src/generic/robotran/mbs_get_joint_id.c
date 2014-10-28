#include <string.h>
#include <stdio.h>

#include "user_all_id.h"

int mbs_get_joint_id(const char* name)
{

	int i;
	char all_joints[COUNT_JOINT][35] = ALL_JOINTS;

	for(i=0;i<COUNT_JOINT;i++)
	{
		if( strcmp(name,all_joints[i]) == 0 ) 
			return i+1;
	}
	
	printf("problem : joint %s not found\n",name);	
	return -1;
}