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
// 01/10/2008 : JFC : Bug n°38
//

#ifndef MBSsensorStruct_h
#define MBSsensorStruct_h
/*--------------------*/

typedef struct MBSsensorStruct_tag
{
		double P[4];
		double R[4][4];
		double V[4];
		double OM[4];
		double A[4];
		double OMP[4];
		double *J[7]; //attention: nécessite allocation dynamique en J[7][njoint+1]
		
/**/
} MBSsensorStruct;

#endif
