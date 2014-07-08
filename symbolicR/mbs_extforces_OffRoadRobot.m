%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Tue Jul  8 17:19:31 2014
%
%	==> Project name : OffRoadRobot
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1402
%
%	==> All Parameter Symbols included
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,14);
 trq = zeros(3,14);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));

% = = Block_0_0_1_1_0_1 = = 
 
% Sensor Kinematics 


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
  OMcp4_25 = qd(5)*C4;
  OMcp4_35 = qd(5)*S4;
  OMcp4_16 = qd(4)+qd(6)*S5;
  OMcp4_26 = OMcp4_25+qd(6)*ROcp4_85;
  OMcp4_36 = OMcp4_35+qd(6)*ROcp4_95;
  OPcp4_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp4_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp4_95-OMcp4_35*S5)-qdd(5)*C4-qdd(6)*ROcp4_85);
  OPcp4_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp4_85-OMcp4_25*S5)+qdd(5)*S4+qdd(6)*ROcp4_95;

% = = Block_0_0_1_1_0_2 = = 
 
% Sensor Kinematics 


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
  RLcp4_17 = ROcp4_16*s.dpt(1,1)+ROcp4_46*s.dpt(2,1)+s.dpt(3,1)*S5;
  RLcp4_27 = ROcp4_26*s.dpt(1,1)+ROcp4_56*s.dpt(2,1)+ROcp4_85*s.dpt(3,1);
  RLcp4_37 = ROcp4_36*s.dpt(1,1)+ROcp4_66*s.dpt(2,1)+ROcp4_95*s.dpt(3,1);
  OMcp4_17 = OMcp4_16+qd(7)*ROcp4_46;
  OMcp4_27 = OMcp4_26+qd(7)*ROcp4_56;
  OMcp4_37 = OMcp4_36+qd(7)*ROcp4_66;
  ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27;
  ORcp4_27 = -(OMcp4_16*RLcp4_37-OMcp4_36*RLcp4_17);
  ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17;
  OPcp4_17 = OPcp4_16+qd(7)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qdd(7)*ROcp4_46;
  OPcp4_27 = OPcp4_26-qd(7)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)+qdd(7)*ROcp4_56;
  OPcp4_37 = OPcp4_36+qd(7)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qdd(7)*ROcp4_66;
  RLcp4_18 = s.dpt(2,5)*ROcp4_46+s.dpt(3,5)*ROcp4_77+ROcp4_17*s.dpt(1,5);
  RLcp4_28 = s.dpt(2,5)*ROcp4_56+s.dpt(3,5)*ROcp4_87+ROcp4_27*s.dpt(1,5);
  RLcp4_38 = s.dpt(2,5)*ROcp4_66+s.dpt(3,5)*ROcp4_97+ROcp4_37*s.dpt(1,5);
  OMcp4_18 = OMcp4_17+qd(8)*ROcp4_46;
  OMcp4_28 = OMcp4_27+qd(8)*ROcp4_56;
  OMcp4_38 = OMcp4_37+qd(8)*ROcp4_66;
  ORcp4_18 = OMcp4_27*RLcp4_38-OMcp4_37*RLcp4_28;
  ORcp4_28 = -(OMcp4_17*RLcp4_38-OMcp4_37*RLcp4_18);
  ORcp4_38 = OMcp4_17*RLcp4_28-OMcp4_27*RLcp4_18;
  OPcp4_18 = OPcp4_17+qd(8)*(OMcp4_27*ROcp4_66-OMcp4_37*ROcp4_56)+qdd(8)*ROcp4_46;
  OPcp4_28 = OPcp4_27-qd(8)*(OMcp4_17*ROcp4_66-OMcp4_37*ROcp4_46)+qdd(8)*ROcp4_56;
  OPcp4_38 = OPcp4_37+qd(8)*(OMcp4_17*ROcp4_56-OMcp4_27*ROcp4_46)+qdd(8)*ROcp4_66;
  RLcp4_119 = s.dpt(1,6)*ROcp4_18+s.dpt(2,6)*ROcp4_46+s.dpt(3,6)*ROcp4_78;
  RLcp4_219 = s.dpt(1,6)*ROcp4_28+s.dpt(2,6)*ROcp4_56+s.dpt(3,6)*ROcp4_88;
  RLcp4_319 = s.dpt(1,6)*ROcp4_38+s.dpt(2,6)*ROcp4_66+s.dpt(3,6)*ROcp4_98;
  ORcp4_119 = OMcp4_28*RLcp4_319-OMcp4_38*RLcp4_219;
  ORcp4_219 = -(OMcp4_18*RLcp4_319-OMcp4_38*RLcp4_119);
  ORcp4_319 = OMcp4_18*RLcp4_219-OMcp4_28*RLcp4_119;
  PxF1(1) = q(1)+RLcp4_119+RLcp4_17+RLcp4_18;
  PxF1(2) = q(2)+RLcp4_219+RLcp4_27+RLcp4_28;
  PxF1(3) = q(3)+RLcp4_319+RLcp4_37+RLcp4_38;
  RxF1(1,1) = ROcp4_18;
  RxF1(1,2) = ROcp4_28;
  RxF1(1,3) = ROcp4_38;
  RxF1(2,1) = ROcp4_46;
  RxF1(2,2) = ROcp4_56;
  RxF1(2,3) = ROcp4_66;
  RxF1(3,1) = ROcp4_78;
  RxF1(3,2) = ROcp4_88;
  RxF1(3,3) = ROcp4_98;
  VxF1(1) = qd(1)+ORcp4_119+ORcp4_17+ORcp4_18;
  VxF1(2) = qd(2)+ORcp4_219+ORcp4_27+ORcp4_28;
  VxF1(3) = qd(3)+ORcp4_319+ORcp4_37+ORcp4_38;
  OMxF1(1) = OMcp4_18;
  OMxF1(2) = OMcp4_28;
  OMxF1(3) = OMcp4_38;
  AxF1(1) = qdd(1)+OMcp4_26*ORcp4_37+OMcp4_27*ORcp4_38+OMcp4_28*ORcp4_319-OMcp4_36*ORcp4_27-OMcp4_37*ORcp4_28-OMcp4_38*ORcp4_219+OPcp4_26*...
 RLcp4_37+OPcp4_27*RLcp4_38+OPcp4_28*RLcp4_319-OPcp4_36*RLcp4_27-OPcp4_37*RLcp4_28-OPcp4_38*RLcp4_219;
  AxF1(2) = qdd(2)-OMcp4_16*ORcp4_37-OMcp4_17*ORcp4_38-OMcp4_18*ORcp4_319+OMcp4_36*ORcp4_17+OMcp4_37*ORcp4_18+OMcp4_38*ORcp4_119-OPcp4_16*...
 RLcp4_37-OPcp4_17*RLcp4_38-OPcp4_18*RLcp4_319+OPcp4_36*RLcp4_17+OPcp4_37*RLcp4_18+OPcp4_38*RLcp4_119;
  AxF1(3) = qdd(3)+OMcp4_16*ORcp4_27+OMcp4_17*ORcp4_28+OMcp4_18*ORcp4_219-OMcp4_26*ORcp4_17-OMcp4_27*ORcp4_18-OMcp4_28*ORcp4_119+OPcp4_16*...
 RLcp4_27+OPcp4_17*RLcp4_28+OPcp4_18*RLcp4_219-OPcp4_26*RLcp4_17-OPcp4_27*RLcp4_18-OPcp4_28*RLcp4_119;
  OMPxF1(1) = OPcp4_18;
  OMPxF1(2) = OPcp4_28;
  OMPxF1(3) = OPcp4_38;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_18*SWr1(1)+ROcp4_28*SWr1(2)+ROcp4_38*SWr1(3);
  xfrc25 = ROcp4_46*SWr1(1)+ROcp4_56*SWr1(2)+ROcp4_66*SWr1(3);
  xfrc35 = ROcp4_78*SWr1(1)+ROcp4_88*SWr1(2)+ROcp4_98*SWr1(3);
  frc(1,8) = s.frc(1,8)+xfrc15;
  frc(2,8) = s.frc(2,8)+xfrc25;
  frc(3,8) = s.frc(3,8)+xfrc35;
  xtrq15 = ROcp4_18*SWr1(4)+ROcp4_28*SWr1(5)+ROcp4_38*SWr1(6);
  xtrq25 = ROcp4_46*SWr1(4)+ROcp4_56*SWr1(5)+ROcp4_66*SWr1(6);
  xtrq35 = ROcp4_78*SWr1(4)+ROcp4_88*SWr1(5)+ROcp4_98*SWr1(6);
  trq(1,8) = s.trq(1,8)+xtrq15-xfrc25*(SWr1(9)-s.l(3,8))-xfrc35*(s.l(2,8)-SWr1(8));
  trq(2,8) = s.trq(2,8)+xtrq25+xfrc15*(SWr1(9)-s.l(3,8))+xfrc35*(s.l(1,8)-SWr1(7));
  trq(3,8) = s.trq(3,8)+xtrq35+xfrc15*(s.l(2,8)-SWr1(8))-xfrc25*(s.l(1,8)-SWr1(7));

% = = Block_0_0_1_2_0_1 = = 
 
% Sensor Kinematics 


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
  OMcp5_25 = qd(5)*C4;
  OMcp5_35 = qd(5)*S4;
  OMcp5_16 = qd(4)+qd(6)*S5;
  OMcp5_26 = OMcp5_25+qd(6)*ROcp5_85;
  OMcp5_36 = OMcp5_35+qd(6)*ROcp5_95;
  OPcp5_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp5_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp5_95-OMcp5_35*S5)-qdd(5)*C4-qdd(6)*ROcp5_85);
  OPcp5_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp5_85-OMcp5_25*S5)+qdd(5)*S4+qdd(6)*ROcp5_95;

% = = Block_0_0_1_2_0_3 = = 
 
% Sensor Kinematics 


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
  RLcp5_19 = ROcp5_16*s.dpt(1,2)+ROcp5_46*s.dpt(2,2)+s.dpt(3,2)*S5;
  RLcp5_29 = ROcp5_26*s.dpt(1,2)+ROcp5_56*s.dpt(2,2)+ROcp5_85*s.dpt(3,2);
  RLcp5_39 = ROcp5_36*s.dpt(1,2)+ROcp5_66*s.dpt(2,2)+ROcp5_95*s.dpt(3,2);
  OMcp5_19 = OMcp5_16+qd(9)*ROcp5_46;
  OMcp5_29 = OMcp5_26+qd(9)*ROcp5_56;
  OMcp5_39 = OMcp5_36+qd(9)*ROcp5_66;
  ORcp5_19 = OMcp5_26*RLcp5_39-OMcp5_36*RLcp5_29;
  ORcp5_29 = -(OMcp5_16*RLcp5_39-OMcp5_36*RLcp5_19);
  ORcp5_39 = OMcp5_16*RLcp5_29-OMcp5_26*RLcp5_19;
  OPcp5_19 = OPcp5_16+qd(9)*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56)+qdd(9)*ROcp5_46;
  OPcp5_29 = OPcp5_26-qd(9)*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46)+qdd(9)*ROcp5_56;
  OPcp5_39 = OPcp5_36+qd(9)*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46)+qdd(9)*ROcp5_66;
  RLcp5_110 = s.dpt(2,7)*ROcp5_46+s.dpt(3,7)*ROcp5_79+ROcp5_19*s.dpt(1,7);
  RLcp5_210 = s.dpt(2,7)*ROcp5_56+s.dpt(3,7)*ROcp5_89+ROcp5_29*s.dpt(1,7);
  RLcp5_310 = s.dpt(2,7)*ROcp5_66+s.dpt(3,7)*ROcp5_99+ROcp5_39*s.dpt(1,7);
  OMcp5_110 = OMcp5_19+qd(10)*ROcp5_46;
  OMcp5_210 = OMcp5_29+qd(10)*ROcp5_56;
  OMcp5_310 = OMcp5_39+qd(10)*ROcp5_66;
  ORcp5_110 = OMcp5_29*RLcp5_310-OMcp5_39*RLcp5_210;
  ORcp5_210 = -(OMcp5_19*RLcp5_310-OMcp5_39*RLcp5_110);
  ORcp5_310 = OMcp5_19*RLcp5_210-OMcp5_29*RLcp5_110;
  OPcp5_110 = OPcp5_19+qd(10)*(OMcp5_29*ROcp5_66-OMcp5_39*ROcp5_56)+qdd(10)*ROcp5_46;
  OPcp5_210 = OPcp5_29-qd(10)*(OMcp5_19*ROcp5_66-OMcp5_39*ROcp5_46)+qdd(10)*ROcp5_56;
  OPcp5_310 = OPcp5_39+qd(10)*(OMcp5_19*ROcp5_56-OMcp5_29*ROcp5_46)+qdd(10)*ROcp5_66;
  RLcp5_120 = s.dpt(1,8)*ROcp5_110+s.dpt(2,8)*ROcp5_46+s.dpt(3,8)*ROcp5_710;
  RLcp5_220 = s.dpt(1,8)*ROcp5_210+s.dpt(2,8)*ROcp5_56+s.dpt(3,8)*ROcp5_810;
  RLcp5_320 = s.dpt(1,8)*ROcp5_310+s.dpt(2,8)*ROcp5_66+s.dpt(3,8)*ROcp5_910;
  ORcp5_120 = OMcp5_210*RLcp5_320-OMcp5_310*RLcp5_220;
  ORcp5_220 = -(OMcp5_110*RLcp5_320-OMcp5_310*RLcp5_120);
  ORcp5_320 = OMcp5_110*RLcp5_220-OMcp5_210*RLcp5_120;
  PxF2(1) = q(1)+RLcp5_110+RLcp5_120+RLcp5_19;
  PxF2(2) = q(2)+RLcp5_210+RLcp5_220+RLcp5_29;
  PxF2(3) = q(3)+RLcp5_310+RLcp5_320+RLcp5_39;
  RxF2(1,1) = ROcp5_110;
  RxF2(1,2) = ROcp5_210;
  RxF2(1,3) = ROcp5_310;
  RxF2(2,1) = ROcp5_46;
  RxF2(2,2) = ROcp5_56;
  RxF2(2,3) = ROcp5_66;
  RxF2(3,1) = ROcp5_710;
  RxF2(3,2) = ROcp5_810;
  RxF2(3,3) = ROcp5_910;
  VxF2(1) = qd(1)+ORcp5_110+ORcp5_120+ORcp5_19;
  VxF2(2) = qd(2)+ORcp5_210+ORcp5_220+ORcp5_29;
  VxF2(3) = qd(3)+ORcp5_310+ORcp5_320+ORcp5_39;
  OMxF2(1) = OMcp5_110;
  OMxF2(2) = OMcp5_210;
  OMxF2(3) = OMcp5_310;
  AxF2(1) = qdd(1)+OMcp5_210*ORcp5_320+OMcp5_26*ORcp5_39+OMcp5_29*ORcp5_310-OMcp5_310*ORcp5_220-OMcp5_36*ORcp5_29-OMcp5_39*ORcp5_210+OPcp5_210*...
 RLcp5_320+OPcp5_26*RLcp5_39+OPcp5_29*RLcp5_310-OPcp5_310*RLcp5_220-OPcp5_36*RLcp5_29-OPcp5_39*RLcp5_210;
  AxF2(2) = qdd(2)-OMcp5_110*ORcp5_320-OMcp5_16*ORcp5_39-OMcp5_19*ORcp5_310+OMcp5_310*ORcp5_120+OMcp5_36*ORcp5_19+OMcp5_39*ORcp5_110-OPcp5_110*...
 RLcp5_320-OPcp5_16*RLcp5_39-OPcp5_19*RLcp5_310+OPcp5_310*RLcp5_120+OPcp5_36*RLcp5_19+OPcp5_39*RLcp5_110;
  AxF2(3) = qdd(3)+OMcp5_110*ORcp5_220+OMcp5_16*ORcp5_29+OMcp5_19*ORcp5_210-OMcp5_210*ORcp5_120-OMcp5_26*ORcp5_19-OMcp5_29*ORcp5_110+OPcp5_110*...
 RLcp5_220+OPcp5_16*RLcp5_29+OPcp5_19*RLcp5_210-OPcp5_210*RLcp5_120-OPcp5_26*RLcp5_19-OPcp5_29*RLcp5_110;
  OMPxF2(1) = OPcp5_110;
  OMPxF2(2) = OPcp5_210;
  OMPxF2(3) = OPcp5_310;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_110*SWr2(1)+ROcp5_210*SWr2(2)+ROcp5_310*SWr2(3);
  xfrc26 = ROcp5_46*SWr2(1)+ROcp5_56*SWr2(2)+ROcp5_66*SWr2(3);
  xfrc36 = ROcp5_710*SWr2(1)+ROcp5_810*SWr2(2)+ROcp5_910*SWr2(3);
  frc(1,10) = s.frc(1,10)+xfrc16;
  frc(2,10) = s.frc(2,10)+xfrc26;
  frc(3,10) = s.frc(3,10)+xfrc36;
  xtrq16 = ROcp5_110*SWr2(4)+ROcp5_210*SWr2(5)+ROcp5_310*SWr2(6);
  xtrq26 = ROcp5_46*SWr2(4)+ROcp5_56*SWr2(5)+ROcp5_66*SWr2(6);
  xtrq36 = ROcp5_710*SWr2(4)+ROcp5_810*SWr2(5)+ROcp5_910*SWr2(6);
  trq(1,10) = s.trq(1,10)+xtrq16-xfrc26*(SWr2(9)-s.l(3,10))-xfrc36*(s.l(2,10)-SWr2(8));
  trq(2,10) = s.trq(2,10)+xtrq26+xfrc16*(SWr2(9)-s.l(3,10))+xfrc36*(s.l(1,10)-SWr2(7));
  trq(3,10) = s.trq(3,10)+xtrq36+xfrc16*(s.l(2,10)-SWr2(8))-xfrc26*(s.l(1,10)-SWr2(7));

% = = Block_0_0_1_3_0_1 = = 
 
% Sensor Kinematics 


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
  OMcp6_25 = qd(5)*C4;
  OMcp6_35 = qd(5)*S4;
  OMcp6_16 = qd(4)+qd(6)*S5;
  OMcp6_26 = OMcp6_25+qd(6)*ROcp6_85;
  OMcp6_36 = OMcp6_35+qd(6)*ROcp6_95;
  OPcp6_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp6_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp6_95-OMcp6_35*S5)-qdd(5)*C4-qdd(6)*ROcp6_85);
  OPcp6_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp6_85-OMcp6_25*S5)+qdd(5)*S4+qdd(6)*ROcp6_95;

% = = Block_0_0_1_3_0_4 = = 
 
% Sensor Kinematics 


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
  RLcp6_111 = ROcp6_16*s.dpt(1,3)+ROcp6_46*s.dpt(2,3)+s.dpt(3,3)*S5;
  RLcp6_211 = ROcp6_26*s.dpt(1,3)+ROcp6_56*s.dpt(2,3)+ROcp6_85*s.dpt(3,3);
  RLcp6_311 = ROcp6_36*s.dpt(1,3)+ROcp6_66*s.dpt(2,3)+ROcp6_95*s.dpt(3,3);
  OMcp6_111 = OMcp6_16+qd(11)*ROcp6_46;
  OMcp6_211 = OMcp6_26+qd(11)*ROcp6_56;
  OMcp6_311 = OMcp6_36+qd(11)*ROcp6_66;
  ORcp6_111 = OMcp6_26*RLcp6_311-OMcp6_36*RLcp6_211;
  ORcp6_211 = -(OMcp6_16*RLcp6_311-OMcp6_36*RLcp6_111);
  ORcp6_311 = OMcp6_16*RLcp6_211-OMcp6_26*RLcp6_111;
  OPcp6_111 = OPcp6_16+qd(11)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56)+qdd(11)*ROcp6_46;
  OPcp6_211 = OPcp6_26-qd(11)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46)+qdd(11)*ROcp6_56;
  OPcp6_311 = OPcp6_36+qd(11)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46)+qdd(11)*ROcp6_66;
  RLcp6_112 = s.dpt(2,9)*ROcp6_46+s.dpt(3,9)*ROcp6_711+ROcp6_111*s.dpt(1,9);
  RLcp6_212 = s.dpt(2,9)*ROcp6_56+s.dpt(3,9)*ROcp6_811+ROcp6_211*s.dpt(1,9);
  RLcp6_312 = s.dpt(2,9)*ROcp6_66+s.dpt(3,9)*ROcp6_911+ROcp6_311*s.dpt(1,9);
  OMcp6_112 = OMcp6_111+qd(12)*ROcp6_46;
  OMcp6_212 = OMcp6_211+qd(12)*ROcp6_56;
  OMcp6_312 = OMcp6_311+qd(12)*ROcp6_66;
  ORcp6_112 = OMcp6_211*RLcp6_312-OMcp6_311*RLcp6_212;
  ORcp6_212 = -(OMcp6_111*RLcp6_312-OMcp6_311*RLcp6_112);
  ORcp6_312 = OMcp6_111*RLcp6_212-OMcp6_211*RLcp6_112;
  OPcp6_112 = OPcp6_111+qd(12)*(OMcp6_211*ROcp6_66-OMcp6_311*ROcp6_56)+qdd(12)*ROcp6_46;
  OPcp6_212 = OPcp6_211-qd(12)*(OMcp6_111*ROcp6_66-OMcp6_311*ROcp6_46)+qdd(12)*ROcp6_56;
  OPcp6_312 = OPcp6_311+qd(12)*(OMcp6_111*ROcp6_56-OMcp6_211*ROcp6_46)+qdd(12)*ROcp6_66;
  RLcp6_121 = s.dpt(1,10)*ROcp6_112+s.dpt(2,10)*ROcp6_46+s.dpt(3,10)*ROcp6_712;
  RLcp6_221 = s.dpt(1,10)*ROcp6_212+s.dpt(2,10)*ROcp6_56+s.dpt(3,10)*ROcp6_812;
  RLcp6_321 = s.dpt(1,10)*ROcp6_312+s.dpt(2,10)*ROcp6_66+s.dpt(3,10)*ROcp6_912;
  ORcp6_121 = OMcp6_212*RLcp6_321-OMcp6_312*RLcp6_221;
  ORcp6_221 = -(OMcp6_112*RLcp6_321-OMcp6_312*RLcp6_121);
  ORcp6_321 = OMcp6_112*RLcp6_221-OMcp6_212*RLcp6_121;
  PxF3(1) = q(1)+RLcp6_111+RLcp6_112+RLcp6_121;
  PxF3(2) = q(2)+RLcp6_211+RLcp6_212+RLcp6_221;
  PxF3(3) = q(3)+RLcp6_311+RLcp6_312+RLcp6_321;
  RxF3(1,1) = ROcp6_112;
  RxF3(1,2) = ROcp6_212;
  RxF3(1,3) = ROcp6_312;
  RxF3(2,1) = ROcp6_46;
  RxF3(2,2) = ROcp6_56;
  RxF3(2,3) = ROcp6_66;
  RxF3(3,1) = ROcp6_712;
  RxF3(3,2) = ROcp6_812;
  RxF3(3,3) = ROcp6_912;
  VxF3(1) = qd(1)+ORcp6_111+ORcp6_112+ORcp6_121;
  VxF3(2) = qd(2)+ORcp6_211+ORcp6_212+ORcp6_221;
  VxF3(3) = qd(3)+ORcp6_311+ORcp6_312+ORcp6_321;
  OMxF3(1) = OMcp6_112;
  OMxF3(2) = OMcp6_212;
  OMxF3(3) = OMcp6_312;
  AxF3(1) = qdd(1)+OMcp6_211*ORcp6_312+OMcp6_212*ORcp6_321+OMcp6_26*ORcp6_311-OMcp6_311*ORcp6_212-OMcp6_312*ORcp6_221-OMcp6_36*ORcp6_211+...
 OPcp6_211*RLcp6_312+OPcp6_212*RLcp6_321+OPcp6_26*RLcp6_311-OPcp6_311*RLcp6_212-OPcp6_312*RLcp6_221-OPcp6_36*RLcp6_211;
  AxF3(2) = qdd(2)-OMcp6_111*ORcp6_312-OMcp6_112*ORcp6_321-OMcp6_16*ORcp6_311+OMcp6_311*ORcp6_112+OMcp6_312*ORcp6_121+OMcp6_36*ORcp6_111-...
 OPcp6_111*RLcp6_312-OPcp6_112*RLcp6_321-OPcp6_16*RLcp6_311+OPcp6_311*RLcp6_112+OPcp6_312*RLcp6_121+OPcp6_36*RLcp6_111;
  AxF3(3) = qdd(3)+OMcp6_111*ORcp6_212+OMcp6_112*ORcp6_221+OMcp6_16*ORcp6_211-OMcp6_211*ORcp6_112-OMcp6_212*ORcp6_121-OMcp6_26*ORcp6_111+...
 OPcp6_111*RLcp6_212+OPcp6_112*RLcp6_221+OPcp6_16*RLcp6_211-OPcp6_211*RLcp6_112-OPcp6_212*RLcp6_121-OPcp6_26*RLcp6_111;
  OMPxF3(1) = OPcp6_112;
  OMPxF3(2) = OPcp6_212;
  OMPxF3(3) = OPcp6_312;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_112*SWr3(1)+ROcp6_212*SWr3(2)+ROcp6_312*SWr3(3);
  xfrc27 = ROcp6_46*SWr3(1)+ROcp6_56*SWr3(2)+ROcp6_66*SWr3(3);
  xfrc37 = ROcp6_712*SWr3(1)+ROcp6_812*SWr3(2)+ROcp6_912*SWr3(3);
  frc(1,12) = s.frc(1,12)+xfrc17;
  frc(2,12) = s.frc(2,12)+xfrc27;
  frc(3,12) = s.frc(3,12)+xfrc37;
  xtrq17 = ROcp6_112*SWr3(4)+ROcp6_212*SWr3(5)+ROcp6_312*SWr3(6);
  xtrq27 = ROcp6_46*SWr3(4)+ROcp6_56*SWr3(5)+ROcp6_66*SWr3(6);
  xtrq37 = ROcp6_712*SWr3(4)+ROcp6_812*SWr3(5)+ROcp6_912*SWr3(6);
  trq(1,12) = s.trq(1,12)+xtrq17-xfrc27*(SWr3(9)-s.l(3,12))-xfrc37*(s.l(2,12)-SWr3(8));
  trq(2,12) = s.trq(2,12)+xtrq27+xfrc17*(SWr3(9)-s.l(3,12))+xfrc37*(s.l(1,12)-SWr3(7));
  trq(3,12) = s.trq(3,12)+xtrq37+xfrc17*(s.l(2,12)-SWr3(8))-xfrc27*(s.l(1,12)-SWr3(7));

% = = Block_0_0_1_4_0_1 = = 
 
% Sensor Kinematics 


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
  OMcp7_25 = qd(5)*C4;
  OMcp7_35 = qd(5)*S4;
  OMcp7_16 = qd(4)+qd(6)*S5;
  OMcp7_26 = OMcp7_25+qd(6)*ROcp7_85;
  OMcp7_36 = OMcp7_35+qd(6)*ROcp7_95;
  OPcp7_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp7_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp7_95-OMcp7_35*S5)-qdd(5)*C4-qdd(6)*ROcp7_85);
  OPcp7_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp7_85-OMcp7_25*S5)+qdd(5)*S4+qdd(6)*ROcp7_95;

% = = Block_0_0_1_4_0_5 = = 
 
% Sensor Kinematics 


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
  RLcp7_113 = ROcp7_16*s.dpt(1,4)+ROcp7_46*s.dpt(2,4)+s.dpt(3,4)*S5;
  RLcp7_213 = ROcp7_26*s.dpt(1,4)+ROcp7_56*s.dpt(2,4)+ROcp7_85*s.dpt(3,4);
  RLcp7_313 = ROcp7_36*s.dpt(1,4)+ROcp7_66*s.dpt(2,4)+ROcp7_95*s.dpt(3,4);
  OMcp7_113 = OMcp7_16+qd(13)*ROcp7_46;
  OMcp7_213 = OMcp7_26+qd(13)*ROcp7_56;
  OMcp7_313 = OMcp7_36+qd(13)*ROcp7_66;
  ORcp7_113 = OMcp7_26*RLcp7_313-OMcp7_36*RLcp7_213;
  ORcp7_213 = -(OMcp7_16*RLcp7_313-OMcp7_36*RLcp7_113);
  ORcp7_313 = OMcp7_16*RLcp7_213-OMcp7_26*RLcp7_113;
  OPcp7_113 = OPcp7_16+qd(13)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56)+qdd(13)*ROcp7_46;
  OPcp7_213 = OPcp7_26-qd(13)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46)+qdd(13)*ROcp7_56;
  OPcp7_313 = OPcp7_36+qd(13)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46)+qdd(13)*ROcp7_66;
  RLcp7_114 = s.dpt(2,11)*ROcp7_46+s.dpt(3,11)*ROcp7_713+ROcp7_113*s.dpt(1,11);
  RLcp7_214 = s.dpt(2,11)*ROcp7_56+s.dpt(3,11)*ROcp7_813+ROcp7_213*s.dpt(1,11);
  RLcp7_314 = s.dpt(2,11)*ROcp7_66+s.dpt(3,11)*ROcp7_913+ROcp7_313*s.dpt(1,11);
  OMcp7_114 = OMcp7_113+qd(14)*ROcp7_46;
  OMcp7_214 = OMcp7_213+qd(14)*ROcp7_56;
  OMcp7_314 = OMcp7_313+qd(14)*ROcp7_66;
  ORcp7_114 = OMcp7_213*RLcp7_314-OMcp7_313*RLcp7_214;
  ORcp7_214 = -(OMcp7_113*RLcp7_314-OMcp7_313*RLcp7_114);
  ORcp7_314 = OMcp7_113*RLcp7_214-OMcp7_213*RLcp7_114;
  OPcp7_114 = OPcp7_113+qd(14)*(OMcp7_213*ROcp7_66-OMcp7_313*ROcp7_56)+qdd(14)*ROcp7_46;
  OPcp7_214 = OPcp7_213-qd(14)*(OMcp7_113*ROcp7_66-OMcp7_313*ROcp7_46)+qdd(14)*ROcp7_56;
  OPcp7_314 = OPcp7_313+qd(14)*(OMcp7_113*ROcp7_56-OMcp7_213*ROcp7_46)+qdd(14)*ROcp7_66;
  RLcp7_122 = s.dpt(1,12)*ROcp7_114+s.dpt(2,12)*ROcp7_46+s.dpt(3,12)*ROcp7_714;
  RLcp7_222 = s.dpt(1,12)*ROcp7_214+s.dpt(2,12)*ROcp7_56+s.dpt(3,12)*ROcp7_814;
  RLcp7_322 = s.dpt(1,12)*ROcp7_314+s.dpt(2,12)*ROcp7_66+s.dpt(3,12)*ROcp7_914;
  ORcp7_122 = OMcp7_214*RLcp7_322-OMcp7_314*RLcp7_222;
  ORcp7_222 = -(OMcp7_114*RLcp7_322-OMcp7_314*RLcp7_122);
  ORcp7_322 = OMcp7_114*RLcp7_222-OMcp7_214*RLcp7_122;
  PxF4(1) = q(1)+RLcp7_113+RLcp7_114+RLcp7_122;
  PxF4(2) = q(2)+RLcp7_213+RLcp7_214+RLcp7_222;
  PxF4(3) = q(3)+RLcp7_313+RLcp7_314+RLcp7_322;
  RxF4(1,1) = ROcp7_114;
  RxF4(1,2) = ROcp7_214;
  RxF4(1,3) = ROcp7_314;
  RxF4(2,1) = ROcp7_46;
  RxF4(2,2) = ROcp7_56;
  RxF4(2,3) = ROcp7_66;
  RxF4(3,1) = ROcp7_714;
  RxF4(3,2) = ROcp7_814;
  RxF4(3,3) = ROcp7_914;
  VxF4(1) = qd(1)+ORcp7_113+ORcp7_114+ORcp7_122;
  VxF4(2) = qd(2)+ORcp7_213+ORcp7_214+ORcp7_222;
  VxF4(3) = qd(3)+ORcp7_313+ORcp7_314+ORcp7_322;
  OMxF4(1) = OMcp7_114;
  OMxF4(2) = OMcp7_214;
  OMxF4(3) = OMcp7_314;
  AxF4(1) = qdd(1)+OMcp7_213*ORcp7_314+OMcp7_214*ORcp7_322+OMcp7_26*ORcp7_313-OMcp7_313*ORcp7_214-OMcp7_314*ORcp7_222-OMcp7_36*ORcp7_213+...
 OPcp7_213*RLcp7_314+OPcp7_214*RLcp7_322+OPcp7_26*RLcp7_313-OPcp7_313*RLcp7_214-OPcp7_314*RLcp7_222-OPcp7_36*RLcp7_213;
  AxF4(2) = qdd(2)-OMcp7_113*ORcp7_314-OMcp7_114*ORcp7_322-OMcp7_16*ORcp7_313+OMcp7_313*ORcp7_114+OMcp7_314*ORcp7_122+OMcp7_36*ORcp7_113-...
 OPcp7_113*RLcp7_314-OPcp7_114*RLcp7_322-OPcp7_16*RLcp7_313+OPcp7_313*RLcp7_114+OPcp7_314*RLcp7_122+OPcp7_36*RLcp7_113;
  AxF4(3) = qdd(3)+OMcp7_113*ORcp7_214+OMcp7_114*ORcp7_222+OMcp7_16*ORcp7_213-OMcp7_213*ORcp7_114-OMcp7_214*ORcp7_122-OMcp7_26*ORcp7_113+...
 OPcp7_113*RLcp7_214+OPcp7_114*RLcp7_222+OPcp7_16*RLcp7_213-OPcp7_213*RLcp7_114-OPcp7_214*RLcp7_122-OPcp7_26*RLcp7_113;
  OMPxF4(1) = OPcp7_114;
  OMPxF4(2) = OPcp7_214;
  OMPxF4(3) = OPcp7_314;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_114*SWr4(1)+ROcp7_214*SWr4(2)+ROcp7_314*SWr4(3);
  xfrc28 = ROcp7_46*SWr4(1)+ROcp7_56*SWr4(2)+ROcp7_66*SWr4(3);
  xfrc38 = ROcp7_714*SWr4(1)+ROcp7_814*SWr4(2)+ROcp7_914*SWr4(3);
  frc(1,14) = s.frc(1,14)+xfrc18;
  frc(2,14) = s.frc(2,14)+xfrc28;
  frc(3,14) = s.frc(3,14)+xfrc38;
  xtrq18 = ROcp7_114*SWr4(4)+ROcp7_214*SWr4(5)+ROcp7_314*SWr4(6);
  xtrq28 = ROcp7_46*SWr4(4)+ROcp7_56*SWr4(5)+ROcp7_66*SWr4(6);
  xtrq38 = ROcp7_714*SWr4(4)+ROcp7_814*SWr4(5)+ROcp7_914*SWr4(6);
  trq(1,14) = s.trq(1,14)+xtrq18-xfrc28*(SWr4(9)-s.l(3,14))-xfrc38*(s.l(2,14)-SWr4(8));
  trq(2,14) = s.trq(2,14)+xtrq28+xfrc18*(SWr4(9)-s.l(3,14))+xfrc38*(s.l(1,14)-SWr4(7));
  trq(3,14) = s.trq(3,14)+xtrq38+xfrc18*(s.l(2,14)-SWr4(8))-xfrc28*(s.l(1,14)-SWr4(7));

% = = Block_0_0_1_4_1_0 = = 
 
% Symbolic Outputs  

  frc(1,1) = s.frc(1,1);
  frc(2,1) = s.frc(2,1);
  frc(3,1) = s.frc(3,1);
  frc(1,2) = s.frc(1,2);
  frc(2,2) = s.frc(2,2);
  frc(3,2) = s.frc(3,2);
  frc(1,3) = s.frc(1,3);
  frc(2,3) = s.frc(2,3);
  frc(3,3) = s.frc(3,3);
  frc(1,4) = s.frc(1,4);
  frc(2,4) = s.frc(2,4);
  frc(3,4) = s.frc(3,4);
  frc(1,5) = s.frc(1,5);
  frc(2,5) = s.frc(2,5);
  frc(3,5) = s.frc(3,5);
  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,9) = s.frc(1,9);
  frc(2,9) = s.frc(2,9);
  frc(3,9) = s.frc(3,9);
  frc(1,11) = s.frc(1,11);
  frc(2,11) = s.frc(2,11);
  frc(3,11) = s.frc(3,11);
  frc(1,13) = s.frc(1,13);
  frc(2,13) = s.frc(2,13);
  frc(3,13) = s.frc(3,13);
  trq(1,1) = s.trq(1,1);
  trq(2,1) = s.trq(2,1);
  trq(3,1) = s.trq(3,1);
  trq(1,2) = s.trq(1,2);
  trq(2,2) = s.trq(2,2);
  trq(3,2) = s.trq(3,2);
  trq(1,3) = s.trq(1,3);
  trq(2,3) = s.trq(2,3);
  trq(3,3) = s.trq(3,3);
  trq(1,4) = s.trq(1,4);
  trq(2,4) = s.trq(2,4);
  trq(3,4) = s.trq(3,4);
  trq(1,5) = s.trq(1,5);
  trq(2,5) = s.trq(2,5);
  trq(3,5) = s.trq(3,5);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,9) = s.trq(1,9);
  trq(2,9) = s.trq(2,9);
  trq(3,9) = s.trq(3,9);
  trq(1,11) = s.trq(1,11);
  trq(2,11) = s.trq(2,11);
  trq(3,11) = s.trq(3,11);
  trq(1,13) = s.trq(1,13);
  trq(2,13) = s.trq(2,13);
  trq(3,13) = s.trq(3,13);

% ====== END Task 0 ====== 

  

