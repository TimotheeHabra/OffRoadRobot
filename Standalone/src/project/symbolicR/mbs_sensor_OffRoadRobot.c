//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Tue Jul  8 17:19:30 2014
//
//	==> Project name : OffRoadRobot
//	==> using XML input file 
//
//	==> Number of joints : 14
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 2822
//
//	==> All Parameter Symbols included
//	==> Generation Time :  0.040 seconds
//	==> Post-Processing :  0.050 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void  sensor(MBSsensorStruct *sens, 
              MBSdataStruct *s,
              int isens)
{ 
 
#include "mbs_sensor_OffRoadRobot.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    ROcp0_16 = C5*C6;
    ROcp0_26 = ROcp0_25*C6+C4*S6;
    ROcp0_36 = ROcp0_35*C6+S4*S6;
    ROcp0_46 = -C5*S6;
    ROcp0_56 = -(ROcp0_25*S6-C4*C6);
    ROcp0_66 = -(ROcp0_35*S6-S4*C6);
    OMcp0_25 = qd[5]*C4;
    OMcp0_35 = qd[5]*S4;
    OMcp0_16 = qd[4]+qd[6]*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd[6];
    OMcp0_36 = OMcp0_35+ROcp0_95*qd[6];
    OPcp0_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp0_26 = ROcp0_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp0_35*S5-ROcp0_95*qd[4]);
    OPcp0_36 = ROcp0_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp0_25*S5-ROcp0_85*qd[4]);

// = = Block_1_0_0_1_0_2 = = 
 
// Sensor Kinematics 


    ROcp0_17 = ROcp0_16*C7-S5*S7;
    ROcp0_27 = ROcp0_26*C7-ROcp0_85*S7;
    ROcp0_37 = ROcp0_36*C7-ROcp0_95*S7;
    ROcp0_77 = ROcp0_16*S7+S5*C7;
    ROcp0_87 = ROcp0_26*S7+ROcp0_85*C7;
    ROcp0_97 = ROcp0_36*S7+ROcp0_95*C7;
    ROcp0_18 = ROcp0_17*C8-ROcp0_77*S8;
    ROcp0_28 = ROcp0_27*C8-ROcp0_87*S8;
    ROcp0_38 = ROcp0_37*C8-ROcp0_97*S8;
    ROcp0_78 = ROcp0_17*S8+ROcp0_77*C8;
    ROcp0_88 = ROcp0_27*S8+ROcp0_87*C8;
    ROcp0_98 = ROcp0_37*S8+ROcp0_97*C8;
    RLcp0_17 = ROcp0_16*s->dpt[1][1]+ROcp0_46*s->dpt[2][1]+s->dpt[3][1]*S5;
    RLcp0_27 = ROcp0_26*s->dpt[1][1]+ROcp0_56*s->dpt[2][1]+ROcp0_85*s->dpt[3][1];
    RLcp0_37 = ROcp0_36*s->dpt[1][1]+ROcp0_66*s->dpt[2][1]+ROcp0_95*s->dpt[3][1];
    OMcp0_17 = OMcp0_16+ROcp0_46*qd[7];
    OMcp0_27 = OMcp0_26+ROcp0_56*qd[7];
    OMcp0_37 = OMcp0_36+ROcp0_66*qd[7];
    ORcp0_17 = OMcp0_26*RLcp0_37-OMcp0_36*RLcp0_27;
    ORcp0_27 = -(OMcp0_16*RLcp0_37-OMcp0_36*RLcp0_17);
    ORcp0_37 = OMcp0_16*RLcp0_27-OMcp0_26*RLcp0_17;
    OPcp0_17 = OPcp0_16+ROcp0_46*qdd[7]+qd[7]*(OMcp0_26*ROcp0_66-OMcp0_36*ROcp0_56);
    OPcp0_27 = OPcp0_26+ROcp0_56*qdd[7]-qd[7]*(OMcp0_16*ROcp0_66-OMcp0_36*ROcp0_46);
    OPcp0_37 = OPcp0_36+ROcp0_66*qdd[7]+qd[7]*(OMcp0_16*ROcp0_56-OMcp0_26*ROcp0_46);
    RLcp0_18 = s->dpt[2][5]*ROcp0_46+s->dpt[3][5]*ROcp0_77+ROcp0_17*s->dpt[1][5];
    RLcp0_28 = s->dpt[2][5]*ROcp0_56+s->dpt[3][5]*ROcp0_87+ROcp0_27*s->dpt[1][5];
    RLcp0_38 = s->dpt[2][5]*ROcp0_66+s->dpt[3][5]*ROcp0_97+ROcp0_37*s->dpt[1][5];
    OMcp0_18 = OMcp0_17+ROcp0_46*qd[8];
    OMcp0_28 = OMcp0_27+ROcp0_56*qd[8];
    OMcp0_38 = OMcp0_37+ROcp0_66*qd[8];
    ORcp0_18 = OMcp0_27*RLcp0_38-OMcp0_37*RLcp0_28;
    ORcp0_28 = -(OMcp0_17*RLcp0_38-OMcp0_37*RLcp0_18);
    ORcp0_38 = OMcp0_17*RLcp0_28-OMcp0_27*RLcp0_18;
    OPcp0_18 = OPcp0_17+ROcp0_46*qdd[8]+qd[8]*(OMcp0_27*ROcp0_66-OMcp0_37*ROcp0_56);
    OPcp0_28 = OPcp0_27+ROcp0_56*qdd[8]-qd[8]*(OMcp0_17*ROcp0_66-OMcp0_37*ROcp0_46);
    OPcp0_38 = OPcp0_37+ROcp0_66*qdd[8]+qd[8]*(OMcp0_17*ROcp0_56-OMcp0_27*ROcp0_46);
    RLcp0_115 = s->dpt[1][6]*ROcp0_18+s->dpt[2][6]*ROcp0_46+s->dpt[3][6]*ROcp0_78;
    RLcp0_215 = s->dpt[1][6]*ROcp0_28+s->dpt[2][6]*ROcp0_56+s->dpt[3][6]*ROcp0_88;
    RLcp0_315 = s->dpt[1][6]*ROcp0_38+s->dpt[2][6]*ROcp0_66+s->dpt[3][6]*ROcp0_98;
    POcp0_115 = RLcp0_115+RLcp0_17+RLcp0_18+q[1];
    POcp0_215 = RLcp0_215+RLcp0_27+RLcp0_28+q[2];
    POcp0_315 = RLcp0_315+RLcp0_37+RLcp0_38+q[3];
    JTcp0_215_4 = -(RLcp0_315+RLcp0_37+RLcp0_38);
    JTcp0_315_4 = RLcp0_215+RLcp0_27+RLcp0_28;
    JTcp0_115_5 = C4*(RLcp0_37+RLcp0_38)-S4*(RLcp0_27+RLcp0_28)-RLcp0_215*S4+RLcp0_315*C4;
    JTcp0_215_5 = S4*(RLcp0_115+RLcp0_17+RLcp0_18);
    JTcp0_315_5 = -C4*(RLcp0_115+RLcp0_17+RLcp0_18);
    JTcp0_115_6 = ROcp0_85*(RLcp0_37+RLcp0_38)-ROcp0_95*(RLcp0_27+RLcp0_28)-RLcp0_215*ROcp0_95+RLcp0_315*ROcp0_85;
    JTcp0_215_6 = -(RLcp0_315*S5-ROcp0_95*(RLcp0_115+RLcp0_17+RLcp0_18)+S5*(RLcp0_37+RLcp0_38));
    JTcp0_315_6 = RLcp0_215*S5-ROcp0_85*(RLcp0_115+RLcp0_17+RLcp0_18)+S5*(RLcp0_27+RLcp0_28);
    JTcp0_115_7 = ROcp0_56*(RLcp0_315+RLcp0_38)-ROcp0_66*(RLcp0_215+RLcp0_28);
    JTcp0_215_7 = -(ROcp0_46*(RLcp0_315+RLcp0_38)-ROcp0_66*(RLcp0_115+RLcp0_18));
    JTcp0_315_7 = ROcp0_46*(RLcp0_215+RLcp0_28)-ROcp0_56*(RLcp0_115+RLcp0_18);
    JTcp0_115_8 = -(RLcp0_215*ROcp0_66-RLcp0_315*ROcp0_56);
    JTcp0_215_8 = RLcp0_115*ROcp0_66-RLcp0_315*ROcp0_46;
    JTcp0_315_8 = -(RLcp0_115*ROcp0_56-RLcp0_215*ROcp0_46);
    ORcp0_115 = OMcp0_28*RLcp0_315-OMcp0_38*RLcp0_215;
    ORcp0_215 = -(OMcp0_18*RLcp0_315-OMcp0_38*RLcp0_115);
    ORcp0_315 = OMcp0_18*RLcp0_215-OMcp0_28*RLcp0_115;
    VIcp0_115 = ORcp0_115+ORcp0_17+ORcp0_18+qd[1];
    VIcp0_215 = ORcp0_215+ORcp0_27+ORcp0_28+qd[2];
    VIcp0_315 = ORcp0_315+ORcp0_37+ORcp0_38+qd[3];
    ACcp0_115 = qdd[1]+OMcp0_26*ORcp0_37+OMcp0_27*ORcp0_38+OMcp0_28*ORcp0_315-OMcp0_36*ORcp0_27-OMcp0_37*ORcp0_28-OMcp0_38
 *ORcp0_215+OPcp0_26*RLcp0_37+OPcp0_27*RLcp0_38+OPcp0_28*RLcp0_315-OPcp0_36*RLcp0_27-OPcp0_37*RLcp0_28-OPcp0_38*RLcp0_215;
    ACcp0_215 = qdd[2]-OMcp0_16*ORcp0_37-OMcp0_17*ORcp0_38-OMcp0_18*ORcp0_315+OMcp0_36*ORcp0_17+OMcp0_37*ORcp0_18+OMcp0_38
 *ORcp0_115-OPcp0_16*RLcp0_37-OPcp0_17*RLcp0_38-OPcp0_18*RLcp0_315+OPcp0_36*RLcp0_17+OPcp0_37*RLcp0_18+OPcp0_38*RLcp0_115;
    ACcp0_315 = qdd[3]+OMcp0_16*ORcp0_27+OMcp0_17*ORcp0_28+OMcp0_18*ORcp0_215-OMcp0_26*ORcp0_17-OMcp0_27*ORcp0_18-OMcp0_28
 *ORcp0_115+OPcp0_16*RLcp0_27+OPcp0_17*RLcp0_28+OPcp0_18*RLcp0_215-OPcp0_26*RLcp0_17-OPcp0_27*RLcp0_18-OPcp0_28*RLcp0_115;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_115;
    sens->P[2] = POcp0_215;
    sens->P[3] = POcp0_315;
    sens->R[1][1] = ROcp0_18;
    sens->R[1][2] = ROcp0_28;
    sens->R[1][3] = ROcp0_38;
    sens->R[2][1] = ROcp0_46;
    sens->R[2][2] = ROcp0_56;
    sens->R[2][3] = ROcp0_66;
    sens->R[3][1] = ROcp0_78;
    sens->R[3][2] = ROcp0_88;
    sens->R[3][3] = ROcp0_98;
    sens->V[1] = VIcp0_115;
    sens->V[2] = VIcp0_215;
    sens->V[3] = VIcp0_315;
    sens->OM[1] = OMcp0_18;
    sens->OM[2] = OMcp0_28;
    sens->OM[3] = OMcp0_38;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp0_115_5;
    sens->J[1][6] = JTcp0_115_6;
    sens->J[1][7] = JTcp0_115_7;
    sens->J[1][8] = JTcp0_115_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp0_215_4;
    sens->J[2][5] = JTcp0_215_5;
    sens->J[2][6] = JTcp0_215_6;
    sens->J[2][7] = JTcp0_215_7;
    sens->J[2][8] = JTcp0_215_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp0_315_4;
    sens->J[3][5] = JTcp0_315_5;
    sens->J[3][6] = JTcp0_315_6;
    sens->J[3][7] = JTcp0_315_7;
    sens->J[3][8] = JTcp0_315_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp0_46;
    sens->J[4][8] = ROcp0_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp0_85;
    sens->J[5][7] = ROcp0_56;
    sens->J[5][8] = ROcp0_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp0_95;
    sens->J[6][7] = ROcp0_66;
    sens->J[6][8] = ROcp0_66;
    sens->A[1] = ACcp0_115;
    sens->A[2] = ACcp0_215;
    sens->A[3] = ACcp0_315;
    sens->OMP[1] = OPcp0_18;
    sens->OMP[2] = OPcp0_28;
    sens->OMP[3] = OPcp0_38;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_25 = qd[5]*C4;
    OMcp1_35 = qd[5]*S4;
    OMcp1_16 = qd[4]+qd[6]*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd[6];
    OMcp1_36 = OMcp1_35+ROcp1_95*qd[6];
    OPcp1_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp1_26 = ROcp1_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4]);
    OPcp1_36 = ROcp1_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp1_25*S5-ROcp1_85*qd[4]);

// = = Block_1_0_0_2_0_3 = = 
 
// Sensor Kinematics 


    ROcp1_19 = ROcp1_16*C9-S5*S9;
    ROcp1_29 = ROcp1_26*C9-ROcp1_85*S9;
    ROcp1_39 = ROcp1_36*C9-ROcp1_95*S9;
    ROcp1_79 = ROcp1_16*S9+S5*C9;
    ROcp1_89 = ROcp1_26*S9+ROcp1_85*C9;
    ROcp1_99 = ROcp1_36*S9+ROcp1_95*C9;
    ROcp1_110 = ROcp1_19*C10-ROcp1_79*S10;
    ROcp1_210 = ROcp1_29*C10-ROcp1_89*S10;
    ROcp1_310 = ROcp1_39*C10-ROcp1_99*S10;
    ROcp1_710 = ROcp1_19*S10+ROcp1_79*C10;
    ROcp1_810 = ROcp1_29*S10+ROcp1_89*C10;
    ROcp1_910 = ROcp1_39*S10+ROcp1_99*C10;
    RLcp1_19 = ROcp1_16*s->dpt[1][2]+ROcp1_46*s->dpt[2][2]+s->dpt[3][2]*S5;
    RLcp1_29 = ROcp1_26*s->dpt[1][2]+ROcp1_56*s->dpt[2][2]+ROcp1_85*s->dpt[3][2];
    RLcp1_39 = ROcp1_36*s->dpt[1][2]+ROcp1_66*s->dpt[2][2]+ROcp1_95*s->dpt[3][2];
    OMcp1_19 = OMcp1_16+ROcp1_46*qd[9];
    OMcp1_29 = OMcp1_26+ROcp1_56*qd[9];
    OMcp1_39 = OMcp1_36+ROcp1_66*qd[9];
    ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
    ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
    ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
    OPcp1_19 = OPcp1_16+ROcp1_46*qdd[9]+qd[9]*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56);
    OPcp1_29 = OPcp1_26+ROcp1_56*qdd[9]-qd[9]*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46);
    OPcp1_39 = OPcp1_36+ROcp1_66*qdd[9]+qd[9]*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46);
    RLcp1_110 = s->dpt[2][7]*ROcp1_46+s->dpt[3][7]*ROcp1_79+ROcp1_19*s->dpt[1][7];
    RLcp1_210 = s->dpt[2][7]*ROcp1_56+s->dpt[3][7]*ROcp1_89+ROcp1_29*s->dpt[1][7];
    RLcp1_310 = s->dpt[2][7]*ROcp1_66+s->dpt[3][7]*ROcp1_99+ROcp1_39*s->dpt[1][7];
    OMcp1_110 = OMcp1_19+ROcp1_46*qd[10];
    OMcp1_210 = OMcp1_29+ROcp1_56*qd[10];
    OMcp1_310 = OMcp1_39+ROcp1_66*qd[10];
    ORcp1_110 = OMcp1_29*RLcp1_310-OMcp1_39*RLcp1_210;
    ORcp1_210 = -(OMcp1_19*RLcp1_310-OMcp1_39*RLcp1_110);
    ORcp1_310 = OMcp1_19*RLcp1_210-OMcp1_29*RLcp1_110;
    OPcp1_110 = OPcp1_19+ROcp1_46*qdd[10]+qd[10]*(OMcp1_29*ROcp1_66-OMcp1_39*ROcp1_56);
    OPcp1_210 = OPcp1_29+ROcp1_56*qdd[10]-qd[10]*(OMcp1_19*ROcp1_66-OMcp1_39*ROcp1_46);
    OPcp1_310 = OPcp1_39+ROcp1_66*qdd[10]+qd[10]*(OMcp1_19*ROcp1_56-OMcp1_29*ROcp1_46);
    RLcp1_116 = s->dpt[1][8]*ROcp1_110+s->dpt[2][8]*ROcp1_46+s->dpt[3][8]*ROcp1_710;
    RLcp1_216 = s->dpt[1][8]*ROcp1_210+s->dpt[2][8]*ROcp1_56+s->dpt[3][8]*ROcp1_810;
    RLcp1_316 = s->dpt[1][8]*ROcp1_310+s->dpt[2][8]*ROcp1_66+s->dpt[3][8]*ROcp1_910;
    POcp1_116 = RLcp1_110+RLcp1_116+RLcp1_19+q[1];
    POcp1_216 = RLcp1_210+RLcp1_216+RLcp1_29+q[2];
    POcp1_316 = RLcp1_310+RLcp1_316+RLcp1_39+q[3];
    JTcp1_216_4 = -(RLcp1_310+RLcp1_316+RLcp1_39);
    JTcp1_316_4 = RLcp1_210+RLcp1_216+RLcp1_29;
    JTcp1_116_5 = C4*(RLcp1_310+RLcp1_39)-S4*(RLcp1_210+RLcp1_29)-RLcp1_216*S4+RLcp1_316*C4;
    JTcp1_216_5 = S4*(RLcp1_110+RLcp1_116+RLcp1_19);
    JTcp1_316_5 = -C4*(RLcp1_110+RLcp1_116+RLcp1_19);
    JTcp1_116_6 = ROcp1_85*(RLcp1_310+RLcp1_39)-ROcp1_95*(RLcp1_210+RLcp1_29)-RLcp1_216*ROcp1_95+RLcp1_316*ROcp1_85;
    JTcp1_216_6 = -(RLcp1_316*S5-ROcp1_95*(RLcp1_110+RLcp1_116+RLcp1_19)+S5*(RLcp1_310+RLcp1_39));
    JTcp1_316_6 = RLcp1_216*S5-ROcp1_85*(RLcp1_110+RLcp1_116+RLcp1_19)+S5*(RLcp1_210+RLcp1_29);
    JTcp1_116_7 = ROcp1_56*(RLcp1_310+RLcp1_316)-ROcp1_66*(RLcp1_210+RLcp1_216);
    JTcp1_216_7 = -(ROcp1_46*(RLcp1_310+RLcp1_316)-ROcp1_66*(RLcp1_110+RLcp1_116));
    JTcp1_316_7 = ROcp1_46*(RLcp1_210+RLcp1_216)-ROcp1_56*(RLcp1_110+RLcp1_116);
    JTcp1_116_8 = -(RLcp1_216*ROcp1_66-RLcp1_316*ROcp1_56);
    JTcp1_216_8 = RLcp1_116*ROcp1_66-RLcp1_316*ROcp1_46;
    JTcp1_316_8 = -(RLcp1_116*ROcp1_56-RLcp1_216*ROcp1_46);
    ORcp1_116 = OMcp1_210*RLcp1_316-OMcp1_310*RLcp1_216;
    ORcp1_216 = -(OMcp1_110*RLcp1_316-OMcp1_310*RLcp1_116);
    ORcp1_316 = OMcp1_110*RLcp1_216-OMcp1_210*RLcp1_116;
    VIcp1_116 = ORcp1_110+ORcp1_116+ORcp1_19+qd[1];
    VIcp1_216 = ORcp1_210+ORcp1_216+ORcp1_29+qd[2];
    VIcp1_316 = ORcp1_310+ORcp1_316+ORcp1_39+qd[3];
    ACcp1_116 = qdd[1]+OMcp1_210*ORcp1_316+OMcp1_26*ORcp1_39+OMcp1_29*ORcp1_310-OMcp1_310*ORcp1_216-OMcp1_36*ORcp1_29-
 OMcp1_39*ORcp1_210+OPcp1_210*RLcp1_316+OPcp1_26*RLcp1_39+OPcp1_29*RLcp1_310-OPcp1_310*RLcp1_216-OPcp1_36*RLcp1_29-OPcp1_39*
 RLcp1_210;
    ACcp1_216 = qdd[2]-OMcp1_110*ORcp1_316-OMcp1_16*ORcp1_39-OMcp1_19*ORcp1_310+OMcp1_310*ORcp1_116+OMcp1_36*ORcp1_19+
 OMcp1_39*ORcp1_110-OPcp1_110*RLcp1_316-OPcp1_16*RLcp1_39-OPcp1_19*RLcp1_310+OPcp1_310*RLcp1_116+OPcp1_36*RLcp1_19+OPcp1_39*
 RLcp1_110;
    ACcp1_316 = qdd[3]+OMcp1_110*ORcp1_216+OMcp1_16*ORcp1_29+OMcp1_19*ORcp1_210-OMcp1_210*ORcp1_116-OMcp1_26*ORcp1_19-
 OMcp1_29*ORcp1_110+OPcp1_110*RLcp1_216+OPcp1_16*RLcp1_29+OPcp1_19*RLcp1_210-OPcp1_210*RLcp1_116-OPcp1_26*RLcp1_19-OPcp1_29*
 RLcp1_110;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_116;
    sens->P[2] = POcp1_216;
    sens->P[3] = POcp1_316;
    sens->R[1][1] = ROcp1_110;
    sens->R[1][2] = ROcp1_210;
    sens->R[1][3] = ROcp1_310;
    sens->R[2][1] = ROcp1_46;
    sens->R[2][2] = ROcp1_56;
    sens->R[2][3] = ROcp1_66;
    sens->R[3][1] = ROcp1_710;
    sens->R[3][2] = ROcp1_810;
    sens->R[3][3] = ROcp1_910;
    sens->V[1] = VIcp1_116;
    sens->V[2] = VIcp1_216;
    sens->V[3] = VIcp1_316;
    sens->OM[1] = OMcp1_110;
    sens->OM[2] = OMcp1_210;
    sens->OM[3] = OMcp1_310;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp1_116_5;
    sens->J[1][6] = JTcp1_116_6;
    sens->J[1][9] = JTcp1_116_7;
    sens->J[1][10] = JTcp1_116_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp1_216_4;
    sens->J[2][5] = JTcp1_216_5;
    sens->J[2][6] = JTcp1_216_6;
    sens->J[2][9] = JTcp1_216_7;
    sens->J[2][10] = JTcp1_216_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp1_316_4;
    sens->J[3][5] = JTcp1_316_5;
    sens->J[3][6] = JTcp1_316_6;
    sens->J[3][9] = JTcp1_316_7;
    sens->J[3][10] = JTcp1_316_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][9] = ROcp1_46;
    sens->J[4][10] = ROcp1_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp1_85;
    sens->J[5][9] = ROcp1_56;
    sens->J[5][10] = ROcp1_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp1_95;
    sens->J[6][9] = ROcp1_66;
    sens->J[6][10] = ROcp1_66;
    sens->A[1] = ACcp1_116;
    sens->A[2] = ACcp1_216;
    sens->A[3] = ACcp1_316;
    sens->OMP[1] = OPcp1_110;
    sens->OMP[2] = OPcp1_210;
    sens->OMP[3] = OPcp1_310;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_25 = qd[5]*C4;
    OMcp2_35 = qd[5]*S4;
    OMcp2_16 = qd[4]+qd[6]*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd[6];
    OMcp2_36 = OMcp2_35+ROcp2_95*qd[6];
    OPcp2_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp2_26 = ROcp2_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4]);
    OPcp2_36 = ROcp2_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp2_25*S5-ROcp2_85*qd[4]);

// = = Block_1_0_0_3_0_4 = = 
 
// Sensor Kinematics 


    ROcp2_111 = ROcp2_16*C11-S11*S5;
    ROcp2_211 = ROcp2_26*C11-ROcp2_85*S11;
    ROcp2_311 = ROcp2_36*C11-ROcp2_95*S11;
    ROcp2_711 = ROcp2_16*S11+C11*S5;
    ROcp2_811 = ROcp2_26*S11+ROcp2_85*C11;
    ROcp2_911 = ROcp2_36*S11+ROcp2_95*C11;
    ROcp2_112 = ROcp2_111*C12-ROcp2_711*S12;
    ROcp2_212 = ROcp2_211*C12-ROcp2_811*S12;
    ROcp2_312 = ROcp2_311*C12-ROcp2_911*S12;
    ROcp2_712 = ROcp2_111*S12+ROcp2_711*C12;
    ROcp2_812 = ROcp2_211*S12+ROcp2_811*C12;
    ROcp2_912 = ROcp2_311*S12+ROcp2_911*C12;
    RLcp2_111 = ROcp2_16*s->dpt[1][3]+ROcp2_46*s->dpt[2][3]+s->dpt[3][3]*S5;
    RLcp2_211 = ROcp2_26*s->dpt[1][3]+ROcp2_56*s->dpt[2][3]+ROcp2_85*s->dpt[3][3];
    RLcp2_311 = ROcp2_36*s->dpt[1][3]+ROcp2_66*s->dpt[2][3]+ROcp2_95*s->dpt[3][3];
    OMcp2_111 = OMcp2_16+ROcp2_46*qd[11];
    OMcp2_211 = OMcp2_26+ROcp2_56*qd[11];
    OMcp2_311 = OMcp2_36+ROcp2_66*qd[11];
    ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
    ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
    ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
    OPcp2_111 = OPcp2_16+ROcp2_46*qdd[11]+qd[11]*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56);
    OPcp2_211 = OPcp2_26+ROcp2_56*qdd[11]-qd[11]*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46);
    OPcp2_311 = OPcp2_36+ROcp2_66*qdd[11]+qd[11]*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46);
    RLcp2_112 = s->dpt[2][9]*ROcp2_46+s->dpt[3][9]*ROcp2_711+ROcp2_111*s->dpt[1][9];
    RLcp2_212 = s->dpt[2][9]*ROcp2_56+s->dpt[3][9]*ROcp2_811+ROcp2_211*s->dpt[1][9];
    RLcp2_312 = s->dpt[2][9]*ROcp2_66+s->dpt[3][9]*ROcp2_911+ROcp2_311*s->dpt[1][9];
    OMcp2_112 = OMcp2_111+ROcp2_46*qd[12];
    OMcp2_212 = OMcp2_211+ROcp2_56*qd[12];
    OMcp2_312 = OMcp2_311+ROcp2_66*qd[12];
    ORcp2_112 = OMcp2_211*RLcp2_312-OMcp2_311*RLcp2_212;
    ORcp2_212 = -(OMcp2_111*RLcp2_312-OMcp2_311*RLcp2_112);
    ORcp2_312 = OMcp2_111*RLcp2_212-OMcp2_211*RLcp2_112;
    OPcp2_112 = OPcp2_111+ROcp2_46*qdd[12]+qd[12]*(OMcp2_211*ROcp2_66-OMcp2_311*ROcp2_56);
    OPcp2_212 = OPcp2_211+ROcp2_56*qdd[12]-qd[12]*(OMcp2_111*ROcp2_66-OMcp2_311*ROcp2_46);
    OPcp2_312 = OPcp2_311+ROcp2_66*qdd[12]+qd[12]*(OMcp2_111*ROcp2_56-OMcp2_211*ROcp2_46);
    RLcp2_117 = s->dpt[1][10]*ROcp2_112+s->dpt[2][10]*ROcp2_46+s->dpt[3][10]*ROcp2_712;
    RLcp2_217 = s->dpt[1][10]*ROcp2_212+s->dpt[2][10]*ROcp2_56+s->dpt[3][10]*ROcp2_812;
    RLcp2_317 = s->dpt[1][10]*ROcp2_312+s->dpt[2][10]*ROcp2_66+s->dpt[3][10]*ROcp2_912;
    POcp2_117 = RLcp2_111+RLcp2_112+RLcp2_117+q[1];
    POcp2_217 = RLcp2_211+RLcp2_212+RLcp2_217+q[2];
    POcp2_317 = RLcp2_311+RLcp2_312+RLcp2_317+q[3];
    JTcp2_217_4 = -(RLcp2_311+RLcp2_312+RLcp2_317);
    JTcp2_317_4 = RLcp2_211+RLcp2_212+RLcp2_217;
    JTcp2_117_5 = C4*(RLcp2_311+RLcp2_312)-S4*(RLcp2_211+RLcp2_212)-RLcp2_217*S4+RLcp2_317*C4;
    JTcp2_217_5 = S4*(RLcp2_111+RLcp2_112+RLcp2_117);
    JTcp2_317_5 = -C4*(RLcp2_111+RLcp2_112+RLcp2_117);
    JTcp2_117_6 = ROcp2_85*(RLcp2_311+RLcp2_312)-ROcp2_95*(RLcp2_211+RLcp2_212)-RLcp2_217*ROcp2_95+RLcp2_317*ROcp2_85;
    JTcp2_217_6 = -(RLcp2_317*S5-ROcp2_95*(RLcp2_111+RLcp2_112+RLcp2_117)+S5*(RLcp2_311+RLcp2_312));
    JTcp2_317_6 = RLcp2_217*S5-ROcp2_85*(RLcp2_111+RLcp2_112+RLcp2_117)+S5*(RLcp2_211+RLcp2_212);
    JTcp2_117_7 = ROcp2_56*(RLcp2_312+RLcp2_317)-ROcp2_66*(RLcp2_212+RLcp2_217);
    JTcp2_217_7 = -(ROcp2_46*(RLcp2_312+RLcp2_317)-ROcp2_66*(RLcp2_112+RLcp2_117));
    JTcp2_317_7 = ROcp2_46*(RLcp2_212+RLcp2_217)-ROcp2_56*(RLcp2_112+RLcp2_117);
    JTcp2_117_8 = -(RLcp2_217*ROcp2_66-RLcp2_317*ROcp2_56);
    JTcp2_217_8 = RLcp2_117*ROcp2_66-RLcp2_317*ROcp2_46;
    JTcp2_317_8 = -(RLcp2_117*ROcp2_56-RLcp2_217*ROcp2_46);
    ORcp2_117 = OMcp2_212*RLcp2_317-OMcp2_312*RLcp2_217;
    ORcp2_217 = -(OMcp2_112*RLcp2_317-OMcp2_312*RLcp2_117);
    ORcp2_317 = OMcp2_112*RLcp2_217-OMcp2_212*RLcp2_117;
    VIcp2_117 = ORcp2_111+ORcp2_112+ORcp2_117+qd[1];
    VIcp2_217 = ORcp2_211+ORcp2_212+ORcp2_217+qd[2];
    VIcp2_317 = ORcp2_311+ORcp2_312+ORcp2_317+qd[3];
    ACcp2_117 = qdd[1]+OMcp2_211*ORcp2_312+OMcp2_212*ORcp2_317+OMcp2_26*ORcp2_311-OMcp2_311*ORcp2_212-OMcp2_312*ORcp2_217-
 OMcp2_36*ORcp2_211+OPcp2_211*RLcp2_312+OPcp2_212*RLcp2_317+OPcp2_26*RLcp2_311-OPcp2_311*RLcp2_212-OPcp2_312*RLcp2_217-
 OPcp2_36*RLcp2_211;
    ACcp2_217 = qdd[2]-OMcp2_111*ORcp2_312-OMcp2_112*ORcp2_317-OMcp2_16*ORcp2_311+OMcp2_311*ORcp2_112+OMcp2_312*ORcp2_117+
 OMcp2_36*ORcp2_111-OPcp2_111*RLcp2_312-OPcp2_112*RLcp2_317-OPcp2_16*RLcp2_311+OPcp2_311*RLcp2_112+OPcp2_312*RLcp2_117+
 OPcp2_36*RLcp2_111;
    ACcp2_317 = qdd[3]+OMcp2_111*ORcp2_212+OMcp2_112*ORcp2_217+OMcp2_16*ORcp2_211-OMcp2_211*ORcp2_112-OMcp2_212*ORcp2_117-
 OMcp2_26*ORcp2_111+OPcp2_111*RLcp2_212+OPcp2_112*RLcp2_217+OPcp2_16*RLcp2_211-OPcp2_211*RLcp2_112-OPcp2_212*RLcp2_117-
 OPcp2_26*RLcp2_111;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_117;
    sens->P[2] = POcp2_217;
    sens->P[3] = POcp2_317;
    sens->R[1][1] = ROcp2_112;
    sens->R[1][2] = ROcp2_212;
    sens->R[1][3] = ROcp2_312;
    sens->R[2][1] = ROcp2_46;
    sens->R[2][2] = ROcp2_56;
    sens->R[2][3] = ROcp2_66;
    sens->R[3][1] = ROcp2_712;
    sens->R[3][2] = ROcp2_812;
    sens->R[3][3] = ROcp2_912;
    sens->V[1] = VIcp2_117;
    sens->V[2] = VIcp2_217;
    sens->V[3] = VIcp2_317;
    sens->OM[1] = OMcp2_112;
    sens->OM[2] = OMcp2_212;
    sens->OM[3] = OMcp2_312;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp2_117_5;
    sens->J[1][6] = JTcp2_117_6;
    sens->J[1][11] = JTcp2_117_7;
    sens->J[1][12] = JTcp2_117_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp2_217_4;
    sens->J[2][5] = JTcp2_217_5;
    sens->J[2][6] = JTcp2_217_6;
    sens->J[2][11] = JTcp2_217_7;
    sens->J[2][12] = JTcp2_217_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp2_317_4;
    sens->J[3][5] = JTcp2_317_5;
    sens->J[3][6] = JTcp2_317_6;
    sens->J[3][11] = JTcp2_317_7;
    sens->J[3][12] = JTcp2_317_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][11] = ROcp2_46;
    sens->J[4][12] = ROcp2_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp2_85;
    sens->J[5][11] = ROcp2_56;
    sens->J[5][12] = ROcp2_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp2_95;
    sens->J[6][11] = ROcp2_66;
    sens->J[6][12] = ROcp2_66;
    sens->A[1] = ACcp2_117;
    sens->A[2] = ACcp2_217;
    sens->A[3] = ACcp2_317;
    sens->OMP[1] = OPcp2_112;
    sens->OMP[2] = OPcp2_212;
    sens->OMP[3] = OPcp2_312;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_25 = qd[5]*C4;
    OMcp3_35 = qd[5]*S4;
    OMcp3_16 = qd[4]+qd[6]*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd[6];
    OMcp3_36 = OMcp3_35+ROcp3_95*qd[6];
    OPcp3_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp3_26 = ROcp3_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp3_35*S5-ROcp3_95*qd[4]);
    OPcp3_36 = ROcp3_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp3_25*S5-ROcp3_85*qd[4]);

// = = Block_1_0_0_4_0_5 = = 
 
// Sensor Kinematics 


    ROcp3_113 = ROcp3_16*C13-S13*S5;
    ROcp3_213 = ROcp3_26*C13-ROcp3_85*S13;
    ROcp3_313 = ROcp3_36*C13-ROcp3_95*S13;
    ROcp3_713 = ROcp3_16*S13+C13*S5;
    ROcp3_813 = ROcp3_26*S13+ROcp3_85*C13;
    ROcp3_913 = ROcp3_36*S13+ROcp3_95*C13;
    ROcp3_114 = ROcp3_113*C14-ROcp3_713*S14;
    ROcp3_214 = ROcp3_213*C14-ROcp3_813*S14;
    ROcp3_314 = ROcp3_313*C14-ROcp3_913*S14;
    ROcp3_714 = ROcp3_113*S14+ROcp3_713*C14;
    ROcp3_814 = ROcp3_213*S14+ROcp3_813*C14;
    ROcp3_914 = ROcp3_313*S14+ROcp3_913*C14;
    RLcp3_113 = ROcp3_16*s->dpt[1][4]+ROcp3_46*s->dpt[2][4]+s->dpt[3][4]*S5;
    RLcp3_213 = ROcp3_26*s->dpt[1][4]+ROcp3_56*s->dpt[2][4]+ROcp3_85*s->dpt[3][4];
    RLcp3_313 = ROcp3_36*s->dpt[1][4]+ROcp3_66*s->dpt[2][4]+ROcp3_95*s->dpt[3][4];
    OMcp3_113 = OMcp3_16+ROcp3_46*qd[13];
    OMcp3_213 = OMcp3_26+ROcp3_56*qd[13];
    OMcp3_313 = OMcp3_36+ROcp3_66*qd[13];
    ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
    ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
    ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
    OPcp3_113 = OPcp3_16+ROcp3_46*qdd[13]+qd[13]*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56);
    OPcp3_213 = OPcp3_26+ROcp3_56*qdd[13]-qd[13]*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46);
    OPcp3_313 = OPcp3_36+ROcp3_66*qdd[13]+qd[13]*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46);
    RLcp3_114 = s->dpt[2][11]*ROcp3_46+s->dpt[3][11]*ROcp3_713+ROcp3_113*s->dpt[1][11];
    RLcp3_214 = s->dpt[2][11]*ROcp3_56+s->dpt[3][11]*ROcp3_813+ROcp3_213*s->dpt[1][11];
    RLcp3_314 = s->dpt[2][11]*ROcp3_66+s->dpt[3][11]*ROcp3_913+ROcp3_313*s->dpt[1][11];
    OMcp3_114 = OMcp3_113+ROcp3_46*qd[14];
    OMcp3_214 = OMcp3_213+ROcp3_56*qd[14];
    OMcp3_314 = OMcp3_313+ROcp3_66*qd[14];
    ORcp3_114 = OMcp3_213*RLcp3_314-OMcp3_313*RLcp3_214;
    ORcp3_214 = -(OMcp3_113*RLcp3_314-OMcp3_313*RLcp3_114);
    ORcp3_314 = OMcp3_113*RLcp3_214-OMcp3_213*RLcp3_114;
    OPcp3_114 = OPcp3_113+ROcp3_46*qdd[14]+qd[14]*(OMcp3_213*ROcp3_66-OMcp3_313*ROcp3_56);
    OPcp3_214 = OPcp3_213+ROcp3_56*qdd[14]-qd[14]*(OMcp3_113*ROcp3_66-OMcp3_313*ROcp3_46);
    OPcp3_314 = OPcp3_313+ROcp3_66*qdd[14]+qd[14]*(OMcp3_113*ROcp3_56-OMcp3_213*ROcp3_46);
    RLcp3_118 = s->dpt[1][12]*ROcp3_114+s->dpt[2][12]*ROcp3_46+s->dpt[3][12]*ROcp3_714;
    RLcp3_218 = s->dpt[1][12]*ROcp3_214+s->dpt[2][12]*ROcp3_56+s->dpt[3][12]*ROcp3_814;
    RLcp3_318 = s->dpt[1][12]*ROcp3_314+s->dpt[2][12]*ROcp3_66+s->dpt[3][12]*ROcp3_914;
    POcp3_118 = RLcp3_113+RLcp3_114+RLcp3_118+q[1];
    POcp3_218 = RLcp3_213+RLcp3_214+RLcp3_218+q[2];
    POcp3_318 = RLcp3_313+RLcp3_314+RLcp3_318+q[3];
    JTcp3_218_4 = -(RLcp3_313+RLcp3_314+RLcp3_318);
    JTcp3_318_4 = RLcp3_213+RLcp3_214+RLcp3_218;
    JTcp3_118_5 = C4*(RLcp3_313+RLcp3_314)-S4*(RLcp3_213+RLcp3_214)-RLcp3_218*S4+RLcp3_318*C4;
    JTcp3_218_5 = S4*(RLcp3_113+RLcp3_114+RLcp3_118);
    JTcp3_318_5 = -C4*(RLcp3_113+RLcp3_114+RLcp3_118);
    JTcp3_118_6 = ROcp3_85*(RLcp3_313+RLcp3_314)-ROcp3_95*(RLcp3_213+RLcp3_214)-RLcp3_218*ROcp3_95+RLcp3_318*ROcp3_85;
    JTcp3_218_6 = -(RLcp3_318*S5-ROcp3_95*(RLcp3_113+RLcp3_114+RLcp3_118)+S5*(RLcp3_313+RLcp3_314));
    JTcp3_318_6 = RLcp3_218*S5-ROcp3_85*(RLcp3_113+RLcp3_114+RLcp3_118)+S5*(RLcp3_213+RLcp3_214);
    JTcp3_118_7 = ROcp3_56*(RLcp3_314+RLcp3_318)-ROcp3_66*(RLcp3_214+RLcp3_218);
    JTcp3_218_7 = -(ROcp3_46*(RLcp3_314+RLcp3_318)-ROcp3_66*(RLcp3_114+RLcp3_118));
    JTcp3_318_7 = ROcp3_46*(RLcp3_214+RLcp3_218)-ROcp3_56*(RLcp3_114+RLcp3_118);
    JTcp3_118_8 = -(RLcp3_218*ROcp3_66-RLcp3_318*ROcp3_56);
    JTcp3_218_8 = RLcp3_118*ROcp3_66-RLcp3_318*ROcp3_46;
    JTcp3_318_8 = -(RLcp3_118*ROcp3_56-RLcp3_218*ROcp3_46);
    ORcp3_118 = OMcp3_214*RLcp3_318-OMcp3_314*RLcp3_218;
    ORcp3_218 = -(OMcp3_114*RLcp3_318-OMcp3_314*RLcp3_118);
    ORcp3_318 = OMcp3_114*RLcp3_218-OMcp3_214*RLcp3_118;
    VIcp3_118 = ORcp3_113+ORcp3_114+ORcp3_118+qd[1];
    VIcp3_218 = ORcp3_213+ORcp3_214+ORcp3_218+qd[2];
    VIcp3_318 = ORcp3_313+ORcp3_314+ORcp3_318+qd[3];
    ACcp3_118 = qdd[1]+OMcp3_213*ORcp3_314+OMcp3_214*ORcp3_318+OMcp3_26*ORcp3_313-OMcp3_313*ORcp3_214-OMcp3_314*ORcp3_218-
 OMcp3_36*ORcp3_213+OPcp3_213*RLcp3_314+OPcp3_214*RLcp3_318+OPcp3_26*RLcp3_313-OPcp3_313*RLcp3_214-OPcp3_314*RLcp3_218-
 OPcp3_36*RLcp3_213;
    ACcp3_218 = qdd[2]-OMcp3_113*ORcp3_314-OMcp3_114*ORcp3_318-OMcp3_16*ORcp3_313+OMcp3_313*ORcp3_114+OMcp3_314*ORcp3_118+
 OMcp3_36*ORcp3_113-OPcp3_113*RLcp3_314-OPcp3_114*RLcp3_318-OPcp3_16*RLcp3_313+OPcp3_313*RLcp3_114+OPcp3_314*RLcp3_118+
 OPcp3_36*RLcp3_113;
    ACcp3_318 = qdd[3]+OMcp3_113*ORcp3_214+OMcp3_114*ORcp3_218+OMcp3_16*ORcp3_213-OMcp3_213*ORcp3_114-OMcp3_214*ORcp3_118-
 OMcp3_26*ORcp3_113+OPcp3_113*RLcp3_214+OPcp3_114*RLcp3_218+OPcp3_16*RLcp3_213-OPcp3_213*RLcp3_114-OPcp3_214*RLcp3_118-
 OPcp3_26*RLcp3_113;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_118;
    sens->P[2] = POcp3_218;
    sens->P[3] = POcp3_318;
    sens->R[1][1] = ROcp3_114;
    sens->R[1][2] = ROcp3_214;
    sens->R[1][3] = ROcp3_314;
    sens->R[2][1] = ROcp3_46;
    sens->R[2][2] = ROcp3_56;
    sens->R[2][3] = ROcp3_66;
    sens->R[3][1] = ROcp3_714;
    sens->R[3][2] = ROcp3_814;
    sens->R[3][3] = ROcp3_914;
    sens->V[1] = VIcp3_118;
    sens->V[2] = VIcp3_218;
    sens->V[3] = VIcp3_318;
    sens->OM[1] = OMcp3_114;
    sens->OM[2] = OMcp3_214;
    sens->OM[3] = OMcp3_314;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp3_118_5;
    sens->J[1][6] = JTcp3_118_6;
    sens->J[1][13] = JTcp3_118_7;
    sens->J[1][14] = JTcp3_118_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp3_218_4;
    sens->J[2][5] = JTcp3_218_5;
    sens->J[2][6] = JTcp3_218_6;
    sens->J[2][13] = JTcp3_218_7;
    sens->J[2][14] = JTcp3_218_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp3_318_4;
    sens->J[3][5] = JTcp3_318_5;
    sens->J[3][6] = JTcp3_318_6;
    sens->J[3][13] = JTcp3_318_7;
    sens->J[3][14] = JTcp3_318_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp3_46;
    sens->J[4][14] = ROcp3_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp3_85;
    sens->J[5][13] = ROcp3_56;
    sens->J[5][14] = ROcp3_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp3_95;
    sens->J[6][13] = ROcp3_66;
    sens->J[6][14] = ROcp3_66;
    sens->A[1] = ACcp3_118;
    sens->A[2] = ACcp3_218;
    sens->A[3] = ACcp3_318;
    sens->OMP[1] = OPcp3_114;
    sens->OMP[2] = OPcp3_214;
    sens->OMP[3] = OPcp3_314;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_25 = qd[5]*C4;
    OMcp4_35 = qd[5]*S4;
    OMcp4_16 = qd[4]+qd[6]*S5;
    OMcp4_26 = OMcp4_25+ROcp4_85*qd[6];
    OMcp4_36 = OMcp4_35+ROcp4_95*qd[6];
    OPcp4_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp4_26 = ROcp4_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp4_35*S5-ROcp4_95*qd[4]);
    OPcp4_36 = ROcp4_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp4_25*S5-ROcp4_85*qd[4]);

// = = Block_1_0_0_5_0_2 = = 
 
// Sensor Kinematics 


    ROcp4_17 = ROcp4_16*C7-S5*S7;
    ROcp4_27 = ROcp4_26*C7-ROcp4_85*S7;
    ROcp4_37 = ROcp4_36*C7-ROcp4_95*S7;
    ROcp4_77 = ROcp4_16*S7+S5*C7;
    ROcp4_87 = ROcp4_26*S7+ROcp4_85*C7;
    ROcp4_97 = ROcp4_36*S7+ROcp4_95*C7;
    ROcp4_18 = ROcp4_17*C8-ROcp4_77*S8;
    ROcp4_28 = ROcp4_27*C8-ROcp4_87*S8;
    ROcp4_38 = ROcp4_37*C8-ROcp4_97*S8;
    ROcp4_78 = ROcp4_17*S8+ROcp4_77*C8;
    ROcp4_88 = ROcp4_27*S8+ROcp4_87*C8;
    ROcp4_98 = ROcp4_37*S8+ROcp4_97*C8;
    RLcp4_17 = ROcp4_16*s->dpt[1][1]+ROcp4_46*s->dpt[2][1]+s->dpt[3][1]*S5;
    RLcp4_27 = ROcp4_26*s->dpt[1][1]+ROcp4_56*s->dpt[2][1]+ROcp4_85*s->dpt[3][1];
    RLcp4_37 = ROcp4_36*s->dpt[1][1]+ROcp4_66*s->dpt[2][1]+ROcp4_95*s->dpt[3][1];
    OMcp4_17 = OMcp4_16+ROcp4_46*qd[7];
    OMcp4_27 = OMcp4_26+ROcp4_56*qd[7];
    OMcp4_37 = OMcp4_36+ROcp4_66*qd[7];
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27;
    ORcp4_27 = -(OMcp4_16*RLcp4_37-OMcp4_36*RLcp4_17);
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17;
    OPcp4_17 = OPcp4_16+ROcp4_46*qdd[7]+qd[7]*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56);
    OPcp4_27 = OPcp4_26+ROcp4_56*qdd[7]-qd[7]*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46);
    OPcp4_37 = OPcp4_36+ROcp4_66*qdd[7]+qd[7]*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46);
    RLcp4_18 = s->dpt[2][5]*ROcp4_46+s->dpt[3][5]*ROcp4_77+ROcp4_17*s->dpt[1][5];
    RLcp4_28 = s->dpt[2][5]*ROcp4_56+s->dpt[3][5]*ROcp4_87+ROcp4_27*s->dpt[1][5];
    RLcp4_38 = s->dpt[2][5]*ROcp4_66+s->dpt[3][5]*ROcp4_97+ROcp4_37*s->dpt[1][5];
    OMcp4_18 = OMcp4_17+ROcp4_46*qd[8];
    OMcp4_28 = OMcp4_27+ROcp4_56*qd[8];
    OMcp4_38 = OMcp4_37+ROcp4_66*qd[8];
    ORcp4_18 = OMcp4_27*RLcp4_38-OMcp4_37*RLcp4_28;
    ORcp4_28 = -(OMcp4_17*RLcp4_38-OMcp4_37*RLcp4_18);
    ORcp4_38 = OMcp4_17*RLcp4_28-OMcp4_27*RLcp4_18;
    OPcp4_18 = OPcp4_17+ROcp4_46*qdd[8]+qd[8]*(OMcp4_27*ROcp4_66-OMcp4_37*ROcp4_56);
    OPcp4_28 = OPcp4_27+ROcp4_56*qdd[8]-qd[8]*(OMcp4_17*ROcp4_66-OMcp4_37*ROcp4_46);
    OPcp4_38 = OPcp4_37+ROcp4_66*qdd[8]+qd[8]*(OMcp4_17*ROcp4_56-OMcp4_27*ROcp4_46);
    RLcp4_119 = s->dpt[1][6]*ROcp4_18+s->dpt[2][6]*ROcp4_46+s->dpt[3][6]*ROcp4_78;
    RLcp4_219 = s->dpt[1][6]*ROcp4_28+s->dpt[2][6]*ROcp4_56+s->dpt[3][6]*ROcp4_88;
    RLcp4_319 = s->dpt[1][6]*ROcp4_38+s->dpt[2][6]*ROcp4_66+s->dpt[3][6]*ROcp4_98;
    POcp4_119 = RLcp4_119+RLcp4_17+RLcp4_18+q[1];
    POcp4_219 = RLcp4_219+RLcp4_27+RLcp4_28+q[2];
    POcp4_319 = RLcp4_319+RLcp4_37+RLcp4_38+q[3];
    ORcp4_119 = OMcp4_28*RLcp4_319-OMcp4_38*RLcp4_219;
    ORcp4_219 = -(OMcp4_18*RLcp4_319-OMcp4_38*RLcp4_119);
    ORcp4_319 = OMcp4_18*RLcp4_219-OMcp4_28*RLcp4_119;
    VIcp4_119 = ORcp4_119+ORcp4_17+ORcp4_18+qd[1];
    VIcp4_219 = ORcp4_219+ORcp4_27+ORcp4_28+qd[2];
    VIcp4_319 = ORcp4_319+ORcp4_37+ORcp4_38+qd[3];
    ACcp4_119 = qdd[1]+OMcp4_26*ORcp4_37+OMcp4_27*ORcp4_38+OMcp4_28*ORcp4_319-OMcp4_36*ORcp4_27-OMcp4_37*ORcp4_28-OMcp4_38
 *ORcp4_219+OPcp4_26*RLcp4_37+OPcp4_27*RLcp4_38+OPcp4_28*RLcp4_319-OPcp4_36*RLcp4_27-OPcp4_37*RLcp4_28-OPcp4_38*RLcp4_219;
    ACcp4_219 = qdd[2]-OMcp4_16*ORcp4_37-OMcp4_17*ORcp4_38-OMcp4_18*ORcp4_319+OMcp4_36*ORcp4_17+OMcp4_37*ORcp4_18+OMcp4_38
 *ORcp4_119-OPcp4_16*RLcp4_37-OPcp4_17*RLcp4_38-OPcp4_18*RLcp4_319+OPcp4_36*RLcp4_17+OPcp4_37*RLcp4_18+OPcp4_38*RLcp4_119;
    ACcp4_319 = qdd[3]+OMcp4_16*ORcp4_27+OMcp4_17*ORcp4_28+OMcp4_18*ORcp4_219-OMcp4_26*ORcp4_17-OMcp4_27*ORcp4_18-OMcp4_28
 *ORcp4_119+OPcp4_16*RLcp4_27+OPcp4_17*RLcp4_28+OPcp4_18*RLcp4_219-OPcp4_26*RLcp4_17-OPcp4_27*RLcp4_18-OPcp4_28*RLcp4_119;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_119;
    sens->P[2] = POcp4_219;
    sens->P[3] = POcp4_319;
    sens->R[1][1] = ROcp4_18;
    sens->R[1][2] = ROcp4_28;
    sens->R[1][3] = ROcp4_38;
    sens->R[2][1] = ROcp4_46;
    sens->R[2][2] = ROcp4_56;
    sens->R[2][3] = ROcp4_66;
    sens->R[3][1] = ROcp4_78;
    sens->R[3][2] = ROcp4_88;
    sens->R[3][3] = ROcp4_98;
    sens->V[1] = VIcp4_119;
    sens->V[2] = VIcp4_219;
    sens->V[3] = VIcp4_319;
    sens->OM[1] = OMcp4_18;
    sens->OM[2] = OMcp4_28;
    sens->OM[3] = OMcp4_38;
    sens->A[1] = ACcp4_119;
    sens->A[2] = ACcp4_219;
    sens->A[3] = ACcp4_319;
    sens->OMP[1] = OPcp4_18;
    sens->OMP[2] = OPcp4_28;
    sens->OMP[3] = OPcp4_38;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_25 = qd[5]*C4;
    OMcp5_35 = qd[5]*S4;
    OMcp5_16 = qd[4]+qd[6]*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd[6];
    OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
    OPcp5_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp5_26 = ROcp5_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_35*S5-ROcp5_95*qd[4]);
    OPcp5_36 = ROcp5_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_25*S5-ROcp5_85*qd[4]);

// = = Block_1_0_0_6_0_3 = = 
 
// Sensor Kinematics 


    ROcp5_19 = ROcp5_16*C9-S5*S9;
    ROcp5_29 = ROcp5_26*C9-ROcp5_85*S9;
    ROcp5_39 = ROcp5_36*C9-ROcp5_95*S9;
    ROcp5_79 = ROcp5_16*S9+S5*C9;
    ROcp5_89 = ROcp5_26*S9+ROcp5_85*C9;
    ROcp5_99 = ROcp5_36*S9+ROcp5_95*C9;
    ROcp5_110 = ROcp5_19*C10-ROcp5_79*S10;
    ROcp5_210 = ROcp5_29*C10-ROcp5_89*S10;
    ROcp5_310 = ROcp5_39*C10-ROcp5_99*S10;
    ROcp5_710 = ROcp5_19*S10+ROcp5_79*C10;
    ROcp5_810 = ROcp5_29*S10+ROcp5_89*C10;
    ROcp5_910 = ROcp5_39*S10+ROcp5_99*C10;
    RLcp5_19 = ROcp5_16*s->dpt[1][2]+ROcp5_46*s->dpt[2][2]+s->dpt[3][2]*S5;
    RLcp5_29 = ROcp5_26*s->dpt[1][2]+ROcp5_56*s->dpt[2][2]+ROcp5_85*s->dpt[3][2];
    RLcp5_39 = ROcp5_36*s->dpt[1][2]+ROcp5_66*s->dpt[2][2]+ROcp5_95*s->dpt[3][2];
    OMcp5_19 = OMcp5_16+ROcp5_46*qd[9];
    OMcp5_29 = OMcp5_26+ROcp5_56*qd[9];
    OMcp5_39 = OMcp5_36+ROcp5_66*qd[9];
    ORcp5_19 = OMcp5_26*RLcp5_39-OMcp5_36*RLcp5_29;
    ORcp5_29 = -(OMcp5_16*RLcp5_39-OMcp5_36*RLcp5_19);
    ORcp5_39 = OMcp5_16*RLcp5_29-OMcp5_26*RLcp5_19;
    OPcp5_19 = OPcp5_16+ROcp5_46*qdd[9]+qd[9]*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56);
    OPcp5_29 = OPcp5_26+ROcp5_56*qdd[9]-qd[9]*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46);
    OPcp5_39 = OPcp5_36+ROcp5_66*qdd[9]+qd[9]*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46);
    RLcp5_110 = s->dpt[2][7]*ROcp5_46+s->dpt[3][7]*ROcp5_79+ROcp5_19*s->dpt[1][7];
    RLcp5_210 = s->dpt[2][7]*ROcp5_56+s->dpt[3][7]*ROcp5_89+ROcp5_29*s->dpt[1][7];
    RLcp5_310 = s->dpt[2][7]*ROcp5_66+s->dpt[3][7]*ROcp5_99+ROcp5_39*s->dpt[1][7];
    OMcp5_110 = OMcp5_19+ROcp5_46*qd[10];
    OMcp5_210 = OMcp5_29+ROcp5_56*qd[10];
    OMcp5_310 = OMcp5_39+ROcp5_66*qd[10];
    ORcp5_110 = OMcp5_29*RLcp5_310-OMcp5_39*RLcp5_210;
    ORcp5_210 = -(OMcp5_19*RLcp5_310-OMcp5_39*RLcp5_110);
    ORcp5_310 = OMcp5_19*RLcp5_210-OMcp5_29*RLcp5_110;
    OPcp5_110 = OPcp5_19+ROcp5_46*qdd[10]+qd[10]*(OMcp5_29*ROcp5_66-OMcp5_39*ROcp5_56);
    OPcp5_210 = OPcp5_29+ROcp5_56*qdd[10]-qd[10]*(OMcp5_19*ROcp5_66-OMcp5_39*ROcp5_46);
    OPcp5_310 = OPcp5_39+ROcp5_66*qdd[10]+qd[10]*(OMcp5_19*ROcp5_56-OMcp5_29*ROcp5_46);
    RLcp5_120 = s->dpt[1][8]*ROcp5_110+s->dpt[2][8]*ROcp5_46+s->dpt[3][8]*ROcp5_710;
    RLcp5_220 = s->dpt[1][8]*ROcp5_210+s->dpt[2][8]*ROcp5_56+s->dpt[3][8]*ROcp5_810;
    RLcp5_320 = s->dpt[1][8]*ROcp5_310+s->dpt[2][8]*ROcp5_66+s->dpt[3][8]*ROcp5_910;
    POcp5_120 = RLcp5_110+RLcp5_120+RLcp5_19+q[1];
    POcp5_220 = RLcp5_210+RLcp5_220+RLcp5_29+q[2];
    POcp5_320 = RLcp5_310+RLcp5_320+RLcp5_39+q[3];
    ORcp5_120 = OMcp5_210*RLcp5_320-OMcp5_310*RLcp5_220;
    ORcp5_220 = -(OMcp5_110*RLcp5_320-OMcp5_310*RLcp5_120);
    ORcp5_320 = OMcp5_110*RLcp5_220-OMcp5_210*RLcp5_120;
    VIcp5_120 = ORcp5_110+ORcp5_120+ORcp5_19+qd[1];
    VIcp5_220 = ORcp5_210+ORcp5_220+ORcp5_29+qd[2];
    VIcp5_320 = ORcp5_310+ORcp5_320+ORcp5_39+qd[3];
    ACcp5_120 = qdd[1]+OMcp5_210*ORcp5_320+OMcp5_26*ORcp5_39+OMcp5_29*ORcp5_310-OMcp5_310*ORcp5_220-OMcp5_36*ORcp5_29-
 OMcp5_39*ORcp5_210+OPcp5_210*RLcp5_320+OPcp5_26*RLcp5_39+OPcp5_29*RLcp5_310-OPcp5_310*RLcp5_220-OPcp5_36*RLcp5_29-OPcp5_39*
 RLcp5_210;
    ACcp5_220 = qdd[2]-OMcp5_110*ORcp5_320-OMcp5_16*ORcp5_39-OMcp5_19*ORcp5_310+OMcp5_310*ORcp5_120+OMcp5_36*ORcp5_19+
 OMcp5_39*ORcp5_110-OPcp5_110*RLcp5_320-OPcp5_16*RLcp5_39-OPcp5_19*RLcp5_310+OPcp5_310*RLcp5_120+OPcp5_36*RLcp5_19+OPcp5_39*
 RLcp5_110;
    ACcp5_320 = qdd[3]+OMcp5_110*ORcp5_220+OMcp5_16*ORcp5_29+OMcp5_19*ORcp5_210-OMcp5_210*ORcp5_120-OMcp5_26*ORcp5_19-
 OMcp5_29*ORcp5_110+OPcp5_110*RLcp5_220+OPcp5_16*RLcp5_29+OPcp5_19*RLcp5_210-OPcp5_210*RLcp5_120-OPcp5_26*RLcp5_19-OPcp5_29*
 RLcp5_110;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_120;
    sens->P[2] = POcp5_220;
    sens->P[3] = POcp5_320;
    sens->R[1][1] = ROcp5_110;
    sens->R[1][2] = ROcp5_210;
    sens->R[1][3] = ROcp5_310;
    sens->R[2][1] = ROcp5_46;
    sens->R[2][2] = ROcp5_56;
    sens->R[2][3] = ROcp5_66;
    sens->R[3][1] = ROcp5_710;
    sens->R[3][2] = ROcp5_810;
    sens->R[3][3] = ROcp5_910;
    sens->V[1] = VIcp5_120;
    sens->V[2] = VIcp5_220;
    sens->V[3] = VIcp5_320;
    sens->OM[1] = OMcp5_110;
    sens->OM[2] = OMcp5_210;
    sens->OM[3] = OMcp5_310;
    sens->A[1] = ACcp5_120;
    sens->A[2] = ACcp5_220;
    sens->A[3] = ACcp5_320;
    sens->OMP[1] = OPcp5_110;
    sens->OMP[2] = OPcp5_210;
    sens->OMP[3] = OPcp5_310;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_25 = qd[5]*C4;
    OMcp6_35 = qd[5]*S4;
    OMcp6_16 = qd[4]+qd[6]*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd[6];
    OMcp6_36 = OMcp6_35+ROcp6_95*qd[6];
    OPcp6_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp6_26 = ROcp6_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4]);
    OPcp6_36 = ROcp6_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_25*S5-ROcp6_85*qd[4]);

// = = Block_1_0_0_7_0_4 = = 
 
// Sensor Kinematics 


    ROcp6_111 = ROcp6_16*C11-S11*S5;
    ROcp6_211 = ROcp6_26*C11-ROcp6_85*S11;
    ROcp6_311 = ROcp6_36*C11-ROcp6_95*S11;
    ROcp6_711 = ROcp6_16*S11+C11*S5;
    ROcp6_811 = ROcp6_26*S11+ROcp6_85*C11;
    ROcp6_911 = ROcp6_36*S11+ROcp6_95*C11;
    ROcp6_112 = ROcp6_111*C12-ROcp6_711*S12;
    ROcp6_212 = ROcp6_211*C12-ROcp6_811*S12;
    ROcp6_312 = ROcp6_311*C12-ROcp6_911*S12;
    ROcp6_712 = ROcp6_111*S12+ROcp6_711*C12;
    ROcp6_812 = ROcp6_211*S12+ROcp6_811*C12;
    ROcp6_912 = ROcp6_311*S12+ROcp6_911*C12;
    RLcp6_111 = ROcp6_16*s->dpt[1][3]+ROcp6_46*s->dpt[2][3]+s->dpt[3][3]*S5;
    RLcp6_211 = ROcp6_26*s->dpt[1][3]+ROcp6_56*s->dpt[2][3]+ROcp6_85*s->dpt[3][3];
    RLcp6_311 = ROcp6_36*s->dpt[1][3]+ROcp6_66*s->dpt[2][3]+ROcp6_95*s->dpt[3][3];
    OMcp6_111 = OMcp6_16+ROcp6_46*qd[11];
    OMcp6_211 = OMcp6_26+ROcp6_56*qd[11];
    OMcp6_311 = OMcp6_36+ROcp6_66*qd[11];
    ORcp6_111 = OMcp6_26*RLcp6_311-OMcp6_36*RLcp6_211;
    ORcp6_211 = -(OMcp6_16*RLcp6_311-OMcp6_36*RLcp6_111);
    ORcp6_311 = OMcp6_16*RLcp6_211-OMcp6_26*RLcp6_111;
    OPcp6_111 = OPcp6_16+ROcp6_46*qdd[11]+qd[11]*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
    OPcp6_211 = OPcp6_26+ROcp6_56*qdd[11]-qd[11]*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
    OPcp6_311 = OPcp6_36+ROcp6_66*qdd[11]+qd[11]*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
    RLcp6_112 = s->dpt[2][9]*ROcp6_46+s->dpt[3][9]*ROcp6_711+ROcp6_111*s->dpt[1][9];
    RLcp6_212 = s->dpt[2][9]*ROcp6_56+s->dpt[3][9]*ROcp6_811+ROcp6_211*s->dpt[1][9];
    RLcp6_312 = s->dpt[2][9]*ROcp6_66+s->dpt[3][9]*ROcp6_911+ROcp6_311*s->dpt[1][9];
    OMcp6_112 = OMcp6_111+ROcp6_46*qd[12];
    OMcp6_212 = OMcp6_211+ROcp6_56*qd[12];
    OMcp6_312 = OMcp6_311+ROcp6_66*qd[12];
    ORcp6_112 = OMcp6_211*RLcp6_312-OMcp6_311*RLcp6_212;
    ORcp6_212 = -(OMcp6_111*RLcp6_312-OMcp6_311*RLcp6_112);
    ORcp6_312 = OMcp6_111*RLcp6_212-OMcp6_211*RLcp6_112;
    OPcp6_112 = OPcp6_111+ROcp6_46*qdd[12]+qd[12]*(OMcp6_211*ROcp6_66-OMcp6_311*ROcp6_56);
    OPcp6_212 = OPcp6_211+ROcp6_56*qdd[12]-qd[12]*(OMcp6_111*ROcp6_66-OMcp6_311*ROcp6_46);
    OPcp6_312 = OPcp6_311+ROcp6_66*qdd[12]+qd[12]*(OMcp6_111*ROcp6_56-OMcp6_211*ROcp6_46);
    RLcp6_121 = s->dpt[1][10]*ROcp6_112+s->dpt[2][10]*ROcp6_46+s->dpt[3][10]*ROcp6_712;
    RLcp6_221 = s->dpt[1][10]*ROcp6_212+s->dpt[2][10]*ROcp6_56+s->dpt[3][10]*ROcp6_812;
    RLcp6_321 = s->dpt[1][10]*ROcp6_312+s->dpt[2][10]*ROcp6_66+s->dpt[3][10]*ROcp6_912;
    POcp6_121 = RLcp6_111+RLcp6_112+RLcp6_121+q[1];
    POcp6_221 = RLcp6_211+RLcp6_212+RLcp6_221+q[2];
    POcp6_321 = RLcp6_311+RLcp6_312+RLcp6_321+q[3];
    ORcp6_121 = OMcp6_212*RLcp6_321-OMcp6_312*RLcp6_221;
    ORcp6_221 = -(OMcp6_112*RLcp6_321-OMcp6_312*RLcp6_121);
    ORcp6_321 = OMcp6_112*RLcp6_221-OMcp6_212*RLcp6_121;
    VIcp6_121 = ORcp6_111+ORcp6_112+ORcp6_121+qd[1];
    VIcp6_221 = ORcp6_211+ORcp6_212+ORcp6_221+qd[2];
    VIcp6_321 = ORcp6_311+ORcp6_312+ORcp6_321+qd[3];
    ACcp6_121 = qdd[1]+OMcp6_211*ORcp6_312+OMcp6_212*ORcp6_321+OMcp6_26*ORcp6_311-OMcp6_311*ORcp6_212-OMcp6_312*ORcp6_221-
 OMcp6_36*ORcp6_211+OPcp6_211*RLcp6_312+OPcp6_212*RLcp6_321+OPcp6_26*RLcp6_311-OPcp6_311*RLcp6_212-OPcp6_312*RLcp6_221-
 OPcp6_36*RLcp6_211;
    ACcp6_221 = qdd[2]-OMcp6_111*ORcp6_312-OMcp6_112*ORcp6_321-OMcp6_16*ORcp6_311+OMcp6_311*ORcp6_112+OMcp6_312*ORcp6_121+
 OMcp6_36*ORcp6_111-OPcp6_111*RLcp6_312-OPcp6_112*RLcp6_321-OPcp6_16*RLcp6_311+OPcp6_311*RLcp6_112+OPcp6_312*RLcp6_121+
 OPcp6_36*RLcp6_111;
    ACcp6_321 = qdd[3]+OMcp6_111*ORcp6_212+OMcp6_112*ORcp6_221+OMcp6_16*ORcp6_211-OMcp6_211*ORcp6_112-OMcp6_212*ORcp6_121-
 OMcp6_26*ORcp6_111+OPcp6_111*RLcp6_212+OPcp6_112*RLcp6_221+OPcp6_16*RLcp6_211-OPcp6_211*RLcp6_112-OPcp6_212*RLcp6_121-
 OPcp6_26*RLcp6_111;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_121;
    sens->P[2] = POcp6_221;
    sens->P[3] = POcp6_321;
    sens->R[1][1] = ROcp6_112;
    sens->R[1][2] = ROcp6_212;
    sens->R[1][3] = ROcp6_312;
    sens->R[2][1] = ROcp6_46;
    sens->R[2][2] = ROcp6_56;
    sens->R[2][3] = ROcp6_66;
    sens->R[3][1] = ROcp6_712;
    sens->R[3][2] = ROcp6_812;
    sens->R[3][3] = ROcp6_912;
    sens->V[1] = VIcp6_121;
    sens->V[2] = VIcp6_221;
    sens->V[3] = VIcp6_321;
    sens->OM[1] = OMcp6_112;
    sens->OM[2] = OMcp6_212;
    sens->OM[3] = OMcp6_312;
    sens->A[1] = ACcp6_121;
    sens->A[2] = ACcp6_221;
    sens->A[3] = ACcp6_321;
    sens->OMP[1] = OPcp6_112;
    sens->OMP[2] = OPcp6_212;
    sens->OMP[3] = OPcp6_312;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_25 = qd[5]*C4;
    OMcp7_35 = qd[5]*S4;
    OMcp7_16 = qd[4]+qd[6]*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd[6];
    OMcp7_36 = OMcp7_35+ROcp7_95*qd[6];
    OPcp7_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp7_26 = ROcp7_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4]);
    OPcp7_36 = ROcp7_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_25*S5-ROcp7_85*qd[4]);

// = = Block_1_0_0_8_0_5 = = 
 
// Sensor Kinematics 


    ROcp7_113 = ROcp7_16*C13-S13*S5;
    ROcp7_213 = ROcp7_26*C13-ROcp7_85*S13;
    ROcp7_313 = ROcp7_36*C13-ROcp7_95*S13;
    ROcp7_713 = ROcp7_16*S13+C13*S5;
    ROcp7_813 = ROcp7_26*S13+ROcp7_85*C13;
    ROcp7_913 = ROcp7_36*S13+ROcp7_95*C13;
    ROcp7_114 = ROcp7_113*C14-ROcp7_713*S14;
    ROcp7_214 = ROcp7_213*C14-ROcp7_813*S14;
    ROcp7_314 = ROcp7_313*C14-ROcp7_913*S14;
    ROcp7_714 = ROcp7_113*S14+ROcp7_713*C14;
    ROcp7_814 = ROcp7_213*S14+ROcp7_813*C14;
    ROcp7_914 = ROcp7_313*S14+ROcp7_913*C14;
    RLcp7_113 = ROcp7_16*s->dpt[1][4]+ROcp7_46*s->dpt[2][4]+s->dpt[3][4]*S5;
    RLcp7_213 = ROcp7_26*s->dpt[1][4]+ROcp7_56*s->dpt[2][4]+ROcp7_85*s->dpt[3][4];
    RLcp7_313 = ROcp7_36*s->dpt[1][4]+ROcp7_66*s->dpt[2][4]+ROcp7_95*s->dpt[3][4];
    OMcp7_113 = OMcp7_16+ROcp7_46*qd[13];
    OMcp7_213 = OMcp7_26+ROcp7_56*qd[13];
    OMcp7_313 = OMcp7_36+ROcp7_66*qd[13];
    ORcp7_113 = OMcp7_26*RLcp7_313-OMcp7_36*RLcp7_213;
    ORcp7_213 = -(OMcp7_16*RLcp7_313-OMcp7_36*RLcp7_113);
    ORcp7_313 = OMcp7_16*RLcp7_213-OMcp7_26*RLcp7_113;
    OPcp7_113 = OPcp7_16+ROcp7_46*qdd[13]+qd[13]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_213 = OPcp7_26+ROcp7_56*qdd[13]-qd[13]*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_313 = OPcp7_36+ROcp7_66*qdd[13]+qd[13]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_114 = s->dpt[2][11]*ROcp7_46+s->dpt[3][11]*ROcp7_713+ROcp7_113*s->dpt[1][11];
    RLcp7_214 = s->dpt[2][11]*ROcp7_56+s->dpt[3][11]*ROcp7_813+ROcp7_213*s->dpt[1][11];
    RLcp7_314 = s->dpt[2][11]*ROcp7_66+s->dpt[3][11]*ROcp7_913+ROcp7_313*s->dpt[1][11];
    OMcp7_114 = OMcp7_113+ROcp7_46*qd[14];
    OMcp7_214 = OMcp7_213+ROcp7_56*qd[14];
    OMcp7_314 = OMcp7_313+ROcp7_66*qd[14];
    ORcp7_114 = OMcp7_213*RLcp7_314-OMcp7_313*RLcp7_214;
    ORcp7_214 = -(OMcp7_113*RLcp7_314-OMcp7_313*RLcp7_114);
    ORcp7_314 = OMcp7_113*RLcp7_214-OMcp7_213*RLcp7_114;
    OPcp7_114 = OPcp7_113+ROcp7_46*qdd[14]+qd[14]*(OMcp7_213*ROcp7_66-OMcp7_313*ROcp7_56);
    OPcp7_214 = OPcp7_213+ROcp7_56*qdd[14]-qd[14]*(OMcp7_113*ROcp7_66-OMcp7_313*ROcp7_46);
    OPcp7_314 = OPcp7_313+ROcp7_66*qdd[14]+qd[14]*(OMcp7_113*ROcp7_56-OMcp7_213*ROcp7_46);
    RLcp7_122 = s->dpt[1][12]*ROcp7_114+s->dpt[2][12]*ROcp7_46+s->dpt[3][12]*ROcp7_714;
    RLcp7_222 = s->dpt[1][12]*ROcp7_214+s->dpt[2][12]*ROcp7_56+s->dpt[3][12]*ROcp7_814;
    RLcp7_322 = s->dpt[1][12]*ROcp7_314+s->dpt[2][12]*ROcp7_66+s->dpt[3][12]*ROcp7_914;
    POcp7_122 = RLcp7_113+RLcp7_114+RLcp7_122+q[1];
    POcp7_222 = RLcp7_213+RLcp7_214+RLcp7_222+q[2];
    POcp7_322 = RLcp7_313+RLcp7_314+RLcp7_322+q[3];
    ORcp7_122 = OMcp7_214*RLcp7_322-OMcp7_314*RLcp7_222;
    ORcp7_222 = -(OMcp7_114*RLcp7_322-OMcp7_314*RLcp7_122);
    ORcp7_322 = OMcp7_114*RLcp7_222-OMcp7_214*RLcp7_122;
    VIcp7_122 = ORcp7_113+ORcp7_114+ORcp7_122+qd[1];
    VIcp7_222 = ORcp7_213+ORcp7_214+ORcp7_222+qd[2];
    VIcp7_322 = ORcp7_313+ORcp7_314+ORcp7_322+qd[3];
    ACcp7_122 = qdd[1]+OMcp7_213*ORcp7_314+OMcp7_214*ORcp7_322+OMcp7_26*ORcp7_313-OMcp7_313*ORcp7_214-OMcp7_314*ORcp7_222-
 OMcp7_36*ORcp7_213+OPcp7_213*RLcp7_314+OPcp7_214*RLcp7_322+OPcp7_26*RLcp7_313-OPcp7_313*RLcp7_214-OPcp7_314*RLcp7_222-
 OPcp7_36*RLcp7_213;
    ACcp7_222 = qdd[2]-OMcp7_113*ORcp7_314-OMcp7_114*ORcp7_322-OMcp7_16*ORcp7_313+OMcp7_313*ORcp7_114+OMcp7_314*ORcp7_122+
 OMcp7_36*ORcp7_113-OPcp7_113*RLcp7_314-OPcp7_114*RLcp7_322-OPcp7_16*RLcp7_313+OPcp7_313*RLcp7_114+OPcp7_314*RLcp7_122+
 OPcp7_36*RLcp7_113;
    ACcp7_322 = qdd[3]+OMcp7_113*ORcp7_214+OMcp7_114*ORcp7_222+OMcp7_16*ORcp7_213-OMcp7_213*ORcp7_114-OMcp7_214*ORcp7_122-
 OMcp7_26*ORcp7_113+OPcp7_113*RLcp7_214+OPcp7_114*RLcp7_222+OPcp7_16*RLcp7_213-OPcp7_213*RLcp7_114-OPcp7_214*RLcp7_122-
 OPcp7_26*RLcp7_113;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_122;
    sens->P[2] = POcp7_222;
    sens->P[3] = POcp7_322;
    sens->R[1][1] = ROcp7_114;
    sens->R[1][2] = ROcp7_214;
    sens->R[1][3] = ROcp7_314;
    sens->R[2][1] = ROcp7_46;
    sens->R[2][2] = ROcp7_56;
    sens->R[2][3] = ROcp7_66;
    sens->R[3][1] = ROcp7_714;
    sens->R[3][2] = ROcp7_814;
    sens->R[3][3] = ROcp7_914;
    sens->V[1] = VIcp7_122;
    sens->V[2] = VIcp7_222;
    sens->V[3] = VIcp7_322;
    sens->OM[1] = OMcp7_114;
    sens->OM[2] = OMcp7_214;
    sens->OM[3] = OMcp7_314;
    sens->A[1] = ACcp7_122;
    sens->A[2] = ACcp7_222;
    sens->A[3] = ACcp7_322;
    sens->OMP[1] = OPcp7_114;
    sens->OMP[2] = OPcp7_214;
    sens->OMP[3] = OPcp7_314;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

