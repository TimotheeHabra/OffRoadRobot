//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : 06-May-2014
//---------------------------

#ifndef ControllerStruct_h
#define ControllerStruct_h


// ---- Structures definitions (typedef) ---- //

// ControllerStruc
typedef struct ControllerStruct
{
    double t;
    double Control[4];

} ControllerStruct;


// ---- Init and free functions: declarations ---- //

ControllerStruct * init_ControllerStruct(void);
void free_ControllerStruct(ControllerStruct *cvs);

/*--------------------*/
#endif

