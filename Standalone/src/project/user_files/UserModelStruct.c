//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Tue Aug 26 16:44:44 2014
//---------------------------




#include "UserModelStruct.h"

// ============================================================ //


UserModelStruct* init_UserModelStruct() 
{
    UserModelStruct* ums;
    ums = (UserModelStruct*)malloc(sizeof(UserModelStruct));
    ums->Motor.motorStates = get_int_vec(12+1);
    ums->Motor.motorStates[0] = 12+1;
 
    return ums;
}

void free_UserModelStruct(UserModelStruct* ums) 
{
    free_int_vec(ums->Motor.motorStates);
    free(ums);
}

 void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums) 
{

    int ind;
    int ind_state_value = 0;

    for(ind=0; ind<gen->user_models->user_model_list[0]->parameter_list[0]->n_value; ind++)
    {
        ums->Motor.motorStates[ind] = ind_state_value;
        ind_state_value++;
    }
 
}

// ============================================================ //
 
