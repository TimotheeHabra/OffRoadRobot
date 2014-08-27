//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°37
//
#ifndef MBSdataStruct_h
#define MBSdataStruct_h
/*--------------------*/

//#ifndef CMEX
#include "user_sf_IO.h"
//#endif

#include "stdio.h"
#include "stdlib.h"

#include "UserModelStruct.h"
#include "LocalDataStruct.h"

typedef struct MBSdataStruct
{
	// MATLAB FIELDS //

	// Données géométriques et dynamiques //
    int npt;
    double *dpt[3+1], *l[3+1], *m, *In[9+1];
    double g[3+1];

    int nbody, njoint;

	// Infos partitionnement //
	int nqu, nqc, nqlocked, nqdriven, nqa, nqv, nhu; // JFC : 15/01/2008 : ajout de nhu
    int *qu, *qc, *qlocked, *qdriven, *qa, *qv, *hu; // JFC : 15/01/2008 : ajout de hu

	// Variables articulaires, valeurs initiales et limites //
    double *q, *qd, *qdd;
    double *q0, *qd0, *qdd0;

    double *qmin, *qmax;

	// Frc, Trq, Qq, tsim
    double *frc[3+1], *trq[3+1], *Qq;
	double tsim;

    // Constraints
    double *lrod;
    int Nloopc, Ncons, Nuserc;
	double NRerr;

	// Links
    int Nlink, Nlink3D;
    double *Z, *Zd, *Fl;
	double **l3DWr; // JFC : Attention: la convention des indices est inversée par rapport aux habitudes Robotran

	// Sensors
    int Nsensor;

	// Ext Forces
    int Nxfrc, *xfidpt;
    double **SWr;	// JFC : Attention: la convention des indices est inversée par rapport aux habitudes Robotran

	// Wheel
    int Nwheel;
    double *rnom;

#if !defined SENSORKIN
    // User Model
	int Nuser_model;
    UserModelStruct *user_model;

//#ifndef CMEX
	// User Variable
	UserIOStruct *user_IO;
//#endif

#endif

	// User State
    double *ux, *uxd;
    double *ux0;
    int Nux;

	// OTHER FIELDS //
	double *qddu;

	int DonePart;
	int DoneEquil;

	int process; // 1 = part, 2 = equil, 3 = dirdynared, 

} MBSdataStruct;

/*--------------------*/
#endif
