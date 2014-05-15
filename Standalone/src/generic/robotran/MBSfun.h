//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#ifndef MBSfun_h
#define MBSfun_h
/*--------------------*/

//#ifndef CMEX
//#include "simstruc.h"
//#else
//#include "mex.h"
//#endif

//------ Custom defines (not in Robotran standard files) -------//
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

//--------------------------------------------------------------//

#include "MBSdataStruct.h"
#include "LocalDataStruct.h"
#include "MBSsensorStruct.h"

#ifndef CMEX
/* S-function */

int sf_set_default_IO_sizes(SimStruct *S);
int sf_set_input_sizes(SimStruct *S, MBSdataStruct *MBSdata);
int sf_set_output_sizes(SimStruct *S, MBSdataStruct *MBSdata);
void sf_set_user_input_sizes(SimStruct *S,MBSdataStruct *MBSdata, int sf_input);
void sf_set_user_output_sizes(SimStruct *S,MBSdataStruct *MBSdata);

void sf_InitCond(SimStruct *S, MBSdataStruct *MBSdata);

void sf_get_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);
void sf_set_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);
void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput);
void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);

void mbs_compute_model(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);
#endif

/* Data management functions */

#ifdef LOAD_XML

#include "loadMBSheader_xml.h"

#else

#ifdef HARDINPUT
	MBSdataStruct *loadMBSsizes_hard();
	MBSdataStruct *loadMBSdata_hard();
#else
	MBSdataStruct *loadMBSsizes(const mxArray *s_ptr);
	MBSdataStruct *loadMBSdata(const mxArray *s_ptr);
#endif

#endif

#ifndef CMEX
UserModelStruct * loadUserModel(mxArray *usm_ptr);
#else
UserModelStruct * loadUserModel();
#endif

#ifndef CMEX
void user_initialization(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);
void user_compute_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds);
#else
int user_initialization(MBSdataStruct *MBSdata, LocalDataStruct *lds);
void user_compute_output(MBSdataStruct *MBSdata, LocalDataStruct *lds);
#endif

void allocate_sensor(MBSsensorStruct *psens, int njoint);
void init_sensor(MBSsensorStruct *psens, int njoint);
void free_sensor(MBSsensorStruct *psens);

//#ifndef CMEX
UserIOStruct * initUserIO(MBSdataStruct *s);
void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s);
//#endif

void checkMBSdata(MBSdataStruct *MBSdata);

void storeMBSdata(MBSdataStruct *s, char *s_name);

void freeMBSdata(MBSdataStruct *s);
void freeUserModel(UserModelStruct *ums);

/* dirdynared */

#ifdef DIRDYNARED
int dirdynared(LocalDataStruct *lds,MBSdataStruct *s);
#endif

/* accelred */

#ifdef ACCELRED
int accelred(double *qddu, MBSdataStruct *s, double tsim);
#endif


/* invdynared */

#ifdef INVDYNARED
int invdynared(LocalDataStruct *lds, MBSdataStruct *s);
#endif

/* LocalDataStruct */

#ifndef CMEX
LocalDataStruct * initLocalDataStruct(SimStruct *S, MBSdataStruct *s);
void freeLocalDataStruct(SimStruct *S, LocalDataStruct *lds, MBSdataStruct *s);
#else
LocalDataStruct * initLocalDataStruct(MBSdataStruct *s);
void freeLocalDataStruct(LocalDataStruct *lds, MBSdataStruct *s);
#endif

/* Computation functions */

int mbs_close_geo(MBSdataStruct *s, LocalDataStruct *lds);

void mbs_close_kin(MBSdataStruct *s, LocalDataStruct *lds);

void mbs_calc_hJ(LocalDataStruct *lds, MBSdataStruct *s, double tsim);

//double* mbs_kine_wheel(double Pw[4],double Rw[4][4],
//						double Vw[4],double OMw[4],
//						MBSdataStruct *s,double tsim,int iwhl);
void mbs_kine_wheel(double Pw[4],double Rw[4][4],
					   double Vw[4],double OMw[4],
					   MBSdataStruct *MBSdata,double tsim,int iwhl,
					   double *rzp, double *anglisp, double *angcambp,
					   double *glissp, double Vct[4], double Rtsol[4][4], double dxF[4]);

void mbs_bakker(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb, double gliss);

void mbs_calspan(double *Fwhl,double *Mwhl,
			 double anglis, double ancamb);

/* Symbolic generated functions */

void cons_hJ(double *h,double **Jac,MBSdataStruct *s, double tsim);

void mbs_calc_hJ(LocalDataStruct *lds, MBSdataStruct *s, double tsim);
void mbs_calc_jdqd(LocalDataStruct *lds, MBSdataStruct *s, double tsim);

void cons_jdqd(double *Jdqd,MBSdataStruct *s, double tsim);

#ifdef CXX
void link1D(double **frc,double **trq,double *Flnk,double *Z,double *Zd,MBSdataStruct *s, double tsim);
#else
void link(double **frc,double **trq,double *Flnk,double *Z,double *Zd,MBSdataStruct *s, double tsim);
#endif

void link3D(double **frc,double **trq, MBSdataStruct *s, double tsim);

void extforces(double **frc, double **trq, MBSdataStruct *s, double tsim);

int accelred(double *qddu,MBSdataStruct *s, double tsim);

void  gensensor(MBSsensorStruct *sens,
              MBSdataStruct *s,
              int isens);

void  sensor(MBSsensorStruct *sens,
              MBSdataStruct *s,
              int isens);

void invdyna(double *Qq,
             MBSdataStruct *s, double tsim);

void dirdyna(double **M,double *c,
			 MBSdataStruct *s, double tsim);

/* User functions */

void user_cons_hJ(double *h, double **Jac, MBSdataStruct *s, double tsim);

void user_cons_jdqd(double *jdqd, MBSdataStruct *s, double tsim);

void user_Derivative(MBSdataStruct *s);

void user_DrivenJoints(MBSdataStruct *s, double tsim);

double* user_ExtForces(double PxF[4], double RxF[4][4],
					   double VxF[4], double OMxF[4],
					   double AxF[4], double OMPxF[4],
					   MBSdataStruct *s, double tsim, int ixF);

double* user_GenExtForces(double PxF[4], double RxF[4][4],
					   double VxF[4], double OMxF[4],
					   double AxF[4], double OMPxF[4],
					   MBSdataStruct *s, double tsim,int iBody);

double user_GroundLevel(double px, double py,
						MBSdataStruct *s, double tsim, int iwhl);

double* user_JointForces(MBSdataStruct *s, double tsim);

double user_LinkForces(double Z, double Zd, MBSdataStruct *s, double tsim, int ilnk);

double* user_Link3DForces(double PxF[4], double RxF[4][4],
					   double VxF[4], double OMxF[4],
					   double AxF[4], double OMPxF[4],
					   MBSdataStruct *s, double tsim,int ixF);

void user_WheelForces(double rz, double anglis, double angcamb,
				      double gliss, double Vctz, double dxF[4],
					  MBSdataStruct *MBSdata, double tsim, int iwhl,
					  double* SWr);
/*--------------------*/
#endif
