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
//	==> Generation Date : Thu May  8 10:28:39 2014
//
//	==> Project name : OffRoadRobot
//	==> using XML input file 
//
//	==> Number of joints : 14
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 843
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.000 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void invdyna(double *Qq,
MBSdataStruct *s, double tsim)

// double Qq[14];
{ 
 
#include "mbs_invdyna_OffRoadRobot.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

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

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA33 = qdd[3]-s->g[3];
  ALPHA24 = qdd[2]*C4+ALPHA33*S4;
  ALPHA34 = -(qdd[2]*S4-ALPHA33*C4);
  OM15 = qd[4]*C5;
  OMp15 = -(qd[4]*qd[5]*S5-qdd[4]*C5);
  ALPHA15 = qdd[1]*C5-ALPHA34*S5;
  ALPHA35 = qdd[1]*S5+ALPHA34*C5;
  OM16 = qd[5]*S6+OM15*C6;
  OM26 = qd[5]*C6-OM15*S6;
  OM36 = qd[6]+qd[4]*S5;
  OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15);
  OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6]);
  OMp36 = qdd[6]+qd[4]*qd[5]*C5+qdd[4]*S5;
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BETA26 = BS26-OMp36;
  BETA36 = BS36+OMp26;
  BETA46 = BS26+OMp36;
  BETA66 = BS66-OMp16;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA15*C6+ALPHA24*S6;
  ALPHA26 = -(ALPHA15*S6-ALPHA24*C6);
  OM17 = OM16*C7-OM36*S7;
  OM37 = OM16*S7+OM36*C7;
  ALPHA17 = C7*(ALPHA16+BETA26*s->dpt[2][1]+BETA36*s->dpt[3][1]+BS16*s->dpt[1][1])-S7*(ALPHA35+BETA76*s->dpt[1][1]+
 BETA86*s->dpt[2][1]+BS96*s->dpt[3][1]);
  ALPHA37 = C7*(ALPHA35+BETA76*s->dpt[1][1]+BETA86*s->dpt[2][1]+BS96*s->dpt[3][1])+S7*(ALPHA16+BETA26*s->dpt[2][1]+
 BETA36*s->dpt[3][1]+BS16*s->dpt[1][1]);
  OM18 = OM17*C8-OM37*S8;
  OM28 = qd[7]+qd[8]+OM26;
  OM38 = OM17*S8+OM37*C8;
  OMp28 = qdd[7]+qdd[8]+OMp26;
  OM19 = OM16*C9-OM36*S9;
  OM39 = OM16*S9+OM36*C9;
  ALPHA19 = C9*(ALPHA16+BETA26*s->dpt[2][2]+BETA36*s->dpt[3][2]+BS16*s->dpt[1][2])-S9*(ALPHA35+BETA76*s->dpt[1][2]+
 BETA86*s->dpt[2][2]+BS96*s->dpt[3][2]);
  ALPHA39 = C9*(ALPHA35+BETA76*s->dpt[1][2]+BETA86*s->dpt[2][2]+BS96*s->dpt[3][2])+S9*(ALPHA16+BETA26*s->dpt[2][2]+
 BETA36*s->dpt[3][2]+BS16*s->dpt[1][2]);
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd[10]+qd[9]+OM26;
  OM310 = OM19*S10+OM39*C10;
  OMp210 = qdd[10]+qdd[9]+OMp26;
  OM111 = OM16*C11-OM36*S11;
  OM311 = OM16*S11+OM36*C11;
  ALPHA111 = C11*(ALPHA16+BETA26*s->dpt[2][3]+BETA36*s->dpt[3][3]+BS16*s->dpt[1][3])-S11*(ALPHA35+BETA76*s->dpt[1][3]+
 BETA86*s->dpt[2][3]+BS96*s->dpt[3][3]);
  ALPHA311 = C11*(ALPHA35+BETA76*s->dpt[1][3]+BETA86*s->dpt[2][3]+BS96*s->dpt[3][3])+S11*(ALPHA16+BETA26*s->dpt[2][3]+
 BETA36*s->dpt[3][3]+BS16*s->dpt[1][3]);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[11]+qd[12]+OM26;
  OM312 = OM111*S12+OM311*C12;
  OMp212 = qdd[11]+qdd[12]+OMp26;
  OM113 = OM16*C13-OM36*S13;
  OM313 = OM16*S13+OM36*C13;
  ALPHA113 = C13*(ALPHA16+BETA26*s->dpt[2][4]+BETA36*s->dpt[3][4]+BS16*s->dpt[1][4])-S13*(ALPHA35+BETA76*s->dpt[1][4]+
 BETA86*s->dpt[2][4]+BS96*s->dpt[3][4]);
  ALPHA313 = C13*(ALPHA35+BETA76*s->dpt[1][4]+BETA86*s->dpt[2][4]+BS96*s->dpt[3][4])+S13*(ALPHA16+BETA26*s->dpt[2][4]+
 BETA36*s->dpt[3][4]+BS16*s->dpt[1][4]);
  OM114 = OM113*C14-OM313*S14;
  OM214 = qd[13]+qd[14]+OM26;
  OM314 = OM113*S14+OM313*C14;
  OMp214 = qdd[13]+qdd[14]+OMp26;
 
// Backward Dynamics 

  Fs114 = -(s->frc[1][14]-s->m[14]*(ALPHA113*C14-ALPHA313*S14+s->l[3][14]*(OMp214+OM114*OM314)));
  Fs214 = -(s->frc[2][14]-s->m[14]*(ALPHA26+BETA46*s->dpt[1][4]+BETA66*s->dpt[3][4]+BS56*s->dpt[2][4]+s->l[3][14]*(OM214
 *OM314+C14*(qd[14]*OM313-C13*(OMp16-qd[13]*OM36)+S13*(OMp36+qd[13]*OM16))+S14*(qd[14]*OM113+C13*(OMp36+qd[13]*OM16)+S13*(
 OMp16-qd[13]*OM36)))));
  Fs314 = -(s->frc[3][14]-s->m[14]*(ALPHA113*S14+ALPHA313*C14-s->l[3][14]*(OM114*OM114+OM214*OM214)));
  Cq114 = -(s->trq[1][14]+s->In[5][14]*OM214*OM314+Fs214*s->l[3][14]);
  Cq214 = -(s->trq[2][14]-s->In[5][14]*OMp214-Fs114*s->l[3][14]);
  Cq314 = -(s->trq[3][14]-s->In[5][14]*OM114*OM214);
  Fq113 = Fs114*C14+Fs314*S14;
  Fq313 = -(Fs114*S14-Fs314*C14);
  Cq113 = Cq114*C14+Cq314*S14;
  Cq313 = -(Cq114*S14-Cq314*C14);
  Fs112 = -(s->frc[1][12]-s->m[12]*(ALPHA111*C12-ALPHA311*S12+s->l[3][12]*(OMp212+OM112*OM312)));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA26+BETA46*s->dpt[1][3]+BETA66*s->dpt[3][3]+BS56*s->dpt[2][3]+s->l[3][12]*(OM212
 *OM312+C12*(qd[12]*OM311-C11*(OMp16-qd[11]*OM36)+S11*(OMp36+qd[11]*OM16))+S12*(qd[12]*OM111+C11*(OMp36+qd[11]*OM16)+S11*(
 OMp16-qd[11]*OM36)))));
  Fs312 = -(s->frc[3][12]-s->m[12]*(ALPHA111*S12+ALPHA311*C12-s->l[3][12]*(OM112*OM112+OM212*OM212)));
  Cq112 = -(s->trq[1][12]+s->In[5][12]*OM212*OM312+Fs212*s->l[3][12]);
  Cq212 = -(s->trq[2][12]-s->In[5][12]*OMp212-Fs112*s->l[3][12]);
  Cq312 = -(s->trq[3][12]-s->In[5][12]*OM112*OM212);
  Fq111 = Fs112*C12+Fs312*S12;
  Fq311 = -(Fs112*S12-Fs312*C12);
  Cq111 = Cq112*C12+Cq312*S12;
  Cq311 = -(Cq112*S12-Cq312*C12);
  Fs110 = -(s->frc[1][10]-s->m[10]*(ALPHA19*C10-ALPHA39*S10+s->l[3][10]*(OMp210+OM110*OM310)));
  Fs210 = -(s->frc[2][10]-s->m[10]*(ALPHA26+BETA46*s->dpt[1][2]+BETA66*s->dpt[3][2]+BS56*s->dpt[2][2]+s->l[3][10]*(OM210
 *OM310+C10*(qd[10]*OM39-C9*(OMp16-qd[9]*OM36)+S9*(OMp36+qd[9]*OM16))+S10*(qd[10]*OM19+C9*(OMp36+qd[9]*OM16)+S9*(OMp16-qd[9]*
 OM36)))));
  Fs310 = -(s->frc[3][10]-s->m[10]*(ALPHA19*S10+ALPHA39*C10-s->l[3][10]*(OM110*OM110+OM210*OM210)));
  Cq110 = -(s->trq[1][10]+s->In[5][10]*OM210*OM310+Fs210*s->l[3][10]);
  Cq210 = -(s->trq[2][10]-s->In[5][10]*OMp210-Fs110*s->l[3][10]);
  Cq310 = -(s->trq[3][10]-s->In[5][10]*OM110*OM210);
  Fq19 = Fs110*C10+Fs310*S10;
  Fq39 = -(Fs110*S10-Fs310*C10);
  Cq19 = Cq110*C10+Cq310*S10;
  Cq39 = -(Cq110*S10-Cq310*C10);
  Fs18 = -(s->frc[1][8]-s->m[8]*(ALPHA17*C8-ALPHA37*S8+s->l[3][8]*(OMp28+OM18*OM38)));
  Fs28 = -(s->frc[2][8]-s->m[8]*(ALPHA26+BETA46*s->dpt[1][1]+BETA66*s->dpt[3][1]+BS56*s->dpt[2][1]+s->l[3][8]*(OM28*OM38
 +C8*(qd[8]*OM37-C7*(OMp16-qd[7]*OM36)+S7*(OMp36+qd[7]*OM16))+S8*(qd[8]*OM17+C7*(OMp36+qd[7]*OM16)+S7*(OMp16-qd[7]*OM36)))));
  Fs38 = -(s->frc[3][8]-s->m[8]*(ALPHA17*S8+ALPHA37*C8-s->l[3][8]*(OM18*OM18+OM28*OM28)));
  Cq18 = -(s->trq[1][8]+s->In[5][8]*OM28*OM38+Fs28*s->l[3][8]);
  Cq28 = -(s->trq[2][8]-s->In[5][8]*OMp28-Fs18*s->l[3][8]);
  Cq38 = -(s->trq[3][8]-s->In[5][8]*OM18*OM28);
  Fq17 = Fs18*C8+Fs38*S8;
  Fq37 = -(Fs18*S8-Fs38*C8);
  Cq17 = Cq18*C8+Cq38*S8;
  Cq37 = -(Cq18*S8-Cq38*C8);
  Fq16 = -(s->frc[1][6]-s->m[6]*ALPHA16-Fq111*C11-Fq113*C13-Fq17*C7-Fq19*C9-Fq311*S11-Fq313*S13-Fq37*S7-Fq39*S9);
  Fq26 = -(s->frc[2][6]-Fs210-Fs212-Fs214-Fs28-s->m[6]*ALPHA26);
  Fq36 = -(s->frc[3][6]-s->m[6]*ALPHA35+Fq111*S11+Fq113*S13+Fq17*S7+Fq19*S9-Fq311*C11-Fq313*C13-Fq37*C7-Fq39*C9);
  Cq16 = -(s->trq[1][6]-s->In[1][6]*OMp16-Cq111*C11-Cq113*C13-Cq17*C7-Cq19*C9-Cq311*S11-Cq313*S13-Cq37*S7-Cq39*S9+Fs210*
 s->dpt[3][2]+Fs212*s->dpt[3][3]+Fs214*s->dpt[3][4]+Fs28*s->dpt[3][1]+OM26*OM36*(s->In[5][6]-s->In[9][6])+s->dpt[2][1]*(Fq17*
 S7-Fq37*C7)+s->dpt[2][2]*(Fq19*S9-Fq39*C9)+s->dpt[2][3]*(Fq111*S11-Fq311*C11)+s->dpt[2][4]*(Fq113*S13-Fq313*C13));
  Cq26 = -(s->trq[2][6]-Cq210-Cq212-Cq214-Cq28-s->In[5][6]*OMp26-OM16*OM36*(s->In[1][6]-s->In[9][6])-s->dpt[1][1]*(Fq17*
 S7-Fq37*C7)-s->dpt[1][2]*(Fq19*S9-Fq39*C9)-s->dpt[1][3]*(Fq111*S11-Fq311*C11)-s->dpt[1][4]*(Fq113*S13-Fq313*C13)-
 s->dpt[3][1]*(Fq17*C7+Fq37*S7)-s->dpt[3][2]*(Fq19*C9+Fq39*S9)-s->dpt[3][3]*(Fq111*C11+Fq311*S11)-s->dpt[3][4]*(Fq113*C13+
 Fq313*S13));
  Cq36 = -(s->trq[3][6]-s->In[9][6]*OMp36+Cq111*S11+Cq113*S13+Cq17*S7+Cq19*S9-Cq311*C11-Cq313*C13-Cq37*C7-Cq39*C9-Fs210*
 s->dpt[1][2]-Fs212*s->dpt[1][3]-Fs214*s->dpt[1][4]-Fs28*s->dpt[1][1]+OM16*OM26*(s->In[1][6]-s->In[5][6])+s->dpt[2][1]*(Fq17*
 C7+Fq37*S7)+s->dpt[2][2]*(Fq19*C9+Fq39*S9)+s->dpt[2][3]*(Fq111*C11+Fq311*S11)+s->dpt[2][4]*(Fq113*C13+Fq313*S13));
  Fq15 = Fq16*C6-Fq26*S6;
  Fq25 = Fq16*S6+Fq26*C6;
  Cq25 = Cq16*S6+Cq26*C6;
  Fq14 = Fq15*C5+Fq36*S5;
  Fq34 = -(Fq15*S5-Fq36*C5);
  Cq14 = Cq36*S5+C5*(Cq16*C6-Cq26*S6);
  Fq23 = Fq25*C4-Fq34*S4;
  Fq33 = Fq25*S4+Fq34*C4;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Fq14;
  Qq[2] = Fq23;
  Qq[3] = Fq33;
  Qq[4] = Cq14;
  Qq[5] = Cq25;
  Qq[6] = Cq36;
  Qq[7] = Cq28;
  Qq[8] = Cq28;
  Qq[9] = Cq210;
  Qq[10] = Cq210;
  Qq[11] = Cq212;
  Qq[12] = Cq212;
  Qq[13] = Cq214;
  Qq[14] = Cq214;

// ====== END Task 0 ====== 


}
 

