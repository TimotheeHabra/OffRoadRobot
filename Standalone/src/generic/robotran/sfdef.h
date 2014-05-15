//-------------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008 by JF Collard
// Last update : 01/10/2008
//-------------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°42/1
//

#ifdef ACCELRED

#define SF_SAMPLE_TIME CONTINUOUS_SAMPLE_TIME
#define SF_NPARAM 2
#define SF_NCSTATES 2*MBSdata->nqu+MBSdata->Nux
#define SF_NDSTATES 0
#define SF_NINPUT 2
#define SF_NOUTPUT 3

#define MDL_INITIALIZE_CONDITIONS
#define MDL_DERIVATIVES 

#elif defined DIRDYNARED

#define SF_SAMPLE_TIME CONTINUOUS_SAMPLE_TIME
#define SF_NPARAM 2
#define SF_NCSTATES 2*MBSdata->nqu+MBSdata->Nux
#define SF_NDSTATES 0
#define SF_NINPUT 2
#define SF_NOUTPUT 3

#define MDL_INITIALIZE_CONDITIONS
#define MDL_DERIVATIVES 

#elif defined INVDYNARED

#define SF_SAMPLE_TIME INHERITED_SAMPLE_TIME
#define SF_NPARAM 2
#define SF_NCSTATES 0
#define SF_NDSTATES 0
#define SF_NINPUT 4
#define SF_NOUTPUT 5

#undef MDL_INITIALIZE_CONDITIONS
#undef MDL_DERIVATIVES

#elif defined SENSORKIN

#define SF_SAMPLE_TIME INHERITED_SAMPLE_TIME
#define SF_NPARAM 7
#define SF_NCSTATES 0
#define SF_NDSTATES 0
#define SF_NINPUT 3
#define SF_NOUTPUT 3

#undef MDL_INITIALIZE_CONDITIONS
#undef MDL_DERIVATIVES

#endif
