//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : Fri May 16 15:18:54 2014
//---------------------------

#ifndef ControllerStruct_h
#define ControllerStruct_h


// ---- Structures definitions (typedef) ---- //

// ControllerInputsStruc
typedef struct ControllerInputsStruct
{
    double tsim;

} ControllerInputsStruct;


// ControllerOutputsStruc
typedef struct ControllerOutputsStruct
{
    double q_ref[4];
    double qd_ref[4];
    double qdd_ref[4];

} ControllerOutputsStruct;


// ControllerStruc
typedef struct ControllerStruct
{
    ControllerInputsStruct *Inputs;
    ControllerOutputsStruct *Outputs;

} ControllerStruct;


// ---- Init and free functions: declarations ---- //

ControllerInputsStruct * init_ControllerInputsStruct(void);
void free_ControllerInputsStruct(ControllerInputsStruct *cvs);

ControllerOutputsStruct * init_ControllerOutputsStruct(void);
void free_ControllerOutputsStruct(ControllerOutputsStruct *cvs);

ControllerStruct * init_ControllerStruct(void);
void free_ControllerStruct(ControllerStruct *cvs);

/*--------------------*/
#endif

