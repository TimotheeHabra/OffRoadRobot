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
%	==> Generation Date : Thu May  8 10:28:38 2014
%
%	==> Project name : OffRoadRobot
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 1718
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.030 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,14);

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

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_1 = = 
 
% Sensor Kinematics 


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
    OMcp0_25 = qd(5)*C4;
    OMcp0_35 = qd(5)*S4;
    OMcp0_16 = qd(4)+qd(6)*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd(6);
    OMcp0_36 = OMcp0_35+ROcp0_95*qd(6);
    OPcp0_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp0_26 = ROcp0_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp0_35*S5-ROcp0_95*qd(4));
    OPcp0_36 = ROcp0_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp0_25*S5-ROcp0_85*qd(4));

% = = Block_1_0_0_1_0_2 = = 
 
% Sensor Kinematics 


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
    RLcp0_17 = ROcp0_16*s.dpt(1,1)+ROcp0_46*s.dpt(2,1)+s.dpt(3,1)*S5;
    RLcp0_27 = ROcp0_26*s.dpt(1,1)+ROcp0_56*s.dpt(2,1)+ROcp0_85*s.dpt(3,1);
    RLcp0_37 = ROcp0_36*s.dpt(1,1)+ROcp0_66*s.dpt(2,1)+ROcp0_95*s.dpt(3,1);
    POcp0_17 = RLcp0_17+q(1);
    POcp0_27 = RLcp0_27+q(2);
    POcp0_37 = RLcp0_37+q(3);
    JTcp0_17_5 = -(RLcp0_27*S4-RLcp0_37*C4);
    JTcp0_27_5 = RLcp0_17*S4;
    JTcp0_37_5 = -RLcp0_17*C4;
    JTcp0_17_6 = -(RLcp0_27*ROcp0_95-RLcp0_37*ROcp0_85);
    JTcp0_27_6 = RLcp0_17*ROcp0_95-RLcp0_37*S5;
    JTcp0_37_6 = -(RLcp0_17*ROcp0_85-RLcp0_27*S5);
    OMcp0_17 = OMcp0_16+ROcp0_46*qd(7);
    OMcp0_27 = OMcp0_26+ROcp0_56*qd(7);
    OMcp0_37 = OMcp0_36+ROcp0_66*qd(7);
    ORcp0_17 = OMcp0_26*RLcp0_37-OMcp0_36*RLcp0_27;
    ORcp0_27 = -(OMcp0_16*RLcp0_37-OMcp0_36*RLcp0_17);
    ORcp0_37 = OMcp0_16*RLcp0_27-OMcp0_26*RLcp0_17;
    VIcp0_17 = ORcp0_17+qd(1);
    VIcp0_27 = ORcp0_27+qd(2);
    VIcp0_37 = ORcp0_37+qd(3);
    ACcp0_17 = qdd(1)+OMcp0_26*ORcp0_37-OMcp0_36*ORcp0_27+OPcp0_26*RLcp0_37-OPcp0_36*RLcp0_27;
    ACcp0_27 = qdd(2)-OMcp0_16*ORcp0_37+OMcp0_36*ORcp0_17-OPcp0_16*RLcp0_37+OPcp0_36*RLcp0_17;
    ACcp0_37 = qdd(3)+OMcp0_16*ORcp0_27-OMcp0_26*ORcp0_17+OPcp0_16*RLcp0_27-OPcp0_26*RLcp0_17;
    OMcp0_18 = OMcp0_17+ROcp0_46*qd(8);
    OMcp0_28 = OMcp0_27+ROcp0_56*qd(8);
    OMcp0_38 = OMcp0_37+ROcp0_66*qd(8);
    OPcp0_18 = OPcp0_16+ROcp0_46*qdd(7)+ROcp0_46*qdd(8)+qd(7)*(OMcp0_26*ROcp0_66-OMcp0_36*ROcp0_56)+qd(8)*(OMcp0_27*ROcp0_66-OMcp0_37*ROcp0_56);
    OPcp0_28 = OPcp0_26+ROcp0_56*qdd(7)+ROcp0_56*qdd(8)-qd(7)*(OMcp0_16*ROcp0_66-OMcp0_36*ROcp0_46)-qd(8)*(OMcp0_17*ROcp0_66-OMcp0_37*ROcp0_46);
    OPcp0_38 = OPcp0_36+ROcp0_66*qdd(7)+ROcp0_66*qdd(8)+qd(7)*(OMcp0_16*ROcp0_56-OMcp0_26*ROcp0_46)+qd(8)*(OMcp0_17*ROcp0_56-OMcp0_27*ROcp0_46);

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_17;
    sens.P(2) = POcp0_27;
    sens.P(3) = POcp0_37;
    sens.R(1,1) = ROcp0_18;
    sens.R(1,2) = ROcp0_28;
    sens.R(1,3) = ROcp0_38;
    sens.R(2,1) = ROcp0_46;
    sens.R(2,2) = ROcp0_56;
    sens.R(2,3) = ROcp0_66;
    sens.R(3,1) = ROcp0_78;
    sens.R(3,2) = ROcp0_88;
    sens.R(3,3) = ROcp0_98;
    sens.V(1) = VIcp0_17;
    sens.V(2) = VIcp0_27;
    sens.V(3) = VIcp0_37;
    sens.OM(1) = OMcp0_18;
    sens.OM(2) = OMcp0_28;
    sens.OM(3) = OMcp0_38;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp0_17_5;
    sens.J(1,6) = JTcp0_17_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp0_37;
    sens.J(2,5) = JTcp0_27_5;
    sens.J(2,6) = JTcp0_27_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp0_27;
    sens.J(3,5) = JTcp0_37_5;
    sens.J(3,6) = JTcp0_37_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp0_46;
    sens.J(4,8) = ROcp0_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp0_85;
    sens.J(5,7) = ROcp0_56;
    sens.J(5,8) = ROcp0_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp0_95;
    sens.J(6,7) = ROcp0_66;
    sens.J(6,8) = ROcp0_66;
    sens.A(1) = ACcp0_17;
    sens.A(2) = ACcp0_27;
    sens.A(3) = ACcp0_37;
    sens.OMP(1) = OPcp0_18;
    sens.OMP(2) = OPcp0_28;
    sens.OMP(3) = OPcp0_38;
 
% 
case 2, 


% = = Block_1_0_0_2_0_1 = = 
 
% Sensor Kinematics 


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
    OMcp1_25 = qd(5)*C4;
    OMcp1_35 = qd(5)*S4;
    OMcp1_16 = qd(4)+qd(6)*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd(6);
    OMcp1_36 = OMcp1_35+ROcp1_95*qd(6);
    OPcp1_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp1_26 = ROcp1_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp1_35*S5-ROcp1_95*qd(4));
    OPcp1_36 = ROcp1_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp1_25*S5-ROcp1_85*qd(4));

% = = Block_1_0_0_2_0_3 = = 
 
% Sensor Kinematics 


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
    RLcp1_19 = ROcp1_16*s.dpt(1,2)+ROcp1_46*s.dpt(2,2)+s.dpt(3,2)*S5;
    RLcp1_29 = ROcp1_26*s.dpt(1,2)+ROcp1_56*s.dpt(2,2)+ROcp1_85*s.dpt(3,2);
    RLcp1_39 = ROcp1_36*s.dpt(1,2)+ROcp1_66*s.dpt(2,2)+ROcp1_95*s.dpt(3,2);
    POcp1_19 = RLcp1_19+q(1);
    POcp1_29 = RLcp1_29+q(2);
    POcp1_39 = RLcp1_39+q(3);
    JTcp1_19_5 = -(RLcp1_29*S4-RLcp1_39*C4);
    JTcp1_29_5 = RLcp1_19*S4;
    JTcp1_39_5 = -RLcp1_19*C4;
    JTcp1_19_6 = -(RLcp1_29*ROcp1_95-RLcp1_39*ROcp1_85);
    JTcp1_29_6 = RLcp1_19*ROcp1_95-RLcp1_39*S5;
    JTcp1_39_6 = -(RLcp1_19*ROcp1_85-RLcp1_29*S5);
    OMcp1_19 = OMcp1_16+ROcp1_46*qd(9);
    OMcp1_29 = OMcp1_26+ROcp1_56*qd(9);
    OMcp1_39 = OMcp1_36+ROcp1_66*qd(9);
    ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
    ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
    ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
    VIcp1_19 = ORcp1_19+qd(1);
    VIcp1_29 = ORcp1_29+qd(2);
    VIcp1_39 = ORcp1_39+qd(3);
    ACcp1_19 = qdd(1)+OMcp1_26*ORcp1_39-OMcp1_36*ORcp1_29+OPcp1_26*RLcp1_39-OPcp1_36*RLcp1_29;
    ACcp1_29 = qdd(2)-OMcp1_16*ORcp1_39+OMcp1_36*ORcp1_19-OPcp1_16*RLcp1_39+OPcp1_36*RLcp1_19;
    ACcp1_39 = qdd(3)+OMcp1_16*ORcp1_29-OMcp1_26*ORcp1_19+OPcp1_16*RLcp1_29-OPcp1_26*RLcp1_19;
    OMcp1_110 = OMcp1_19+ROcp1_46*qd(10);
    OMcp1_210 = OMcp1_29+ROcp1_56*qd(10);
    OMcp1_310 = OMcp1_39+ROcp1_66*qd(10);
    OPcp1_110 = OPcp1_16+ROcp1_46*qdd(10)+ROcp1_46*qdd(9)+qd(10)*(OMcp1_29*ROcp1_66-OMcp1_39*ROcp1_56)+qd(9)*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56);
    OPcp1_210 = OPcp1_26+ROcp1_56*qdd(10)+ROcp1_56*qdd(9)-qd(10)*(OMcp1_19*ROcp1_66-OMcp1_39*ROcp1_46)-qd(9)*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46);
    OPcp1_310 = OPcp1_36+ROcp1_66*qdd(10)+ROcp1_66*qdd(9)+qd(10)*(OMcp1_19*ROcp1_56-OMcp1_29*ROcp1_46)+qd(9)*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46);

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp1_19;
    sens.P(2) = POcp1_29;
    sens.P(3) = POcp1_39;
    sens.R(1,1) = ROcp1_110;
    sens.R(1,2) = ROcp1_210;
    sens.R(1,3) = ROcp1_310;
    sens.R(2,1) = ROcp1_46;
    sens.R(2,2) = ROcp1_56;
    sens.R(2,3) = ROcp1_66;
    sens.R(3,1) = ROcp1_710;
    sens.R(3,2) = ROcp1_810;
    sens.R(3,3) = ROcp1_910;
    sens.V(1) = VIcp1_19;
    sens.V(2) = VIcp1_29;
    sens.V(3) = VIcp1_39;
    sens.OM(1) = OMcp1_110;
    sens.OM(2) = OMcp1_210;
    sens.OM(3) = OMcp1_310;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp1_19_5;
    sens.J(1,6) = JTcp1_19_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp1_39;
    sens.J(2,5) = JTcp1_29_5;
    sens.J(2,6) = JTcp1_29_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp1_29;
    sens.J(3,5) = JTcp1_39_5;
    sens.J(3,6) = JTcp1_39_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,9) = ROcp1_46;
    sens.J(4,10) = ROcp1_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp1_85;
    sens.J(5,9) = ROcp1_56;
    sens.J(5,10) = ROcp1_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp1_95;
    sens.J(6,9) = ROcp1_66;
    sens.J(6,10) = ROcp1_66;
    sens.A(1) = ACcp1_19;
    sens.A(2) = ACcp1_29;
    sens.A(3) = ACcp1_39;
    sens.OMP(1) = OPcp1_110;
    sens.OMP(2) = OPcp1_210;
    sens.OMP(3) = OPcp1_310;
 
% 
case 3, 


% = = Block_1_0_0_3_0_1 = = 
 
% Sensor Kinematics 


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
    OMcp2_25 = qd(5)*C4;
    OMcp2_35 = qd(5)*S4;
    OMcp2_16 = qd(4)+qd(6)*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd(6);
    OMcp2_36 = OMcp2_35+ROcp2_95*qd(6);
    OPcp2_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp2_26 = ROcp2_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp2_35*S5-ROcp2_95*qd(4));
    OPcp2_36 = ROcp2_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp2_25*S5-ROcp2_85*qd(4));

% = = Block_1_0_0_3_0_4 = = 
 
% Sensor Kinematics 


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
    RLcp2_111 = ROcp2_16*s.dpt(1,3)+ROcp2_46*s.dpt(2,3)+s.dpt(3,3)*S5;
    RLcp2_211 = ROcp2_26*s.dpt(1,3)+ROcp2_56*s.dpt(2,3)+ROcp2_85*s.dpt(3,3);
    RLcp2_311 = ROcp2_36*s.dpt(1,3)+ROcp2_66*s.dpt(2,3)+ROcp2_95*s.dpt(3,3);
    POcp2_111 = RLcp2_111+q(1);
    POcp2_211 = RLcp2_211+q(2);
    POcp2_311 = RLcp2_311+q(3);
    JTcp2_111_5 = -(RLcp2_211*S4-RLcp2_311*C4);
    JTcp2_211_5 = RLcp2_111*S4;
    JTcp2_311_5 = -RLcp2_111*C4;
    JTcp2_111_6 = -(RLcp2_211*ROcp2_95-RLcp2_311*ROcp2_85);
    JTcp2_211_6 = RLcp2_111*ROcp2_95-RLcp2_311*S5;
    JTcp2_311_6 = -(RLcp2_111*ROcp2_85-RLcp2_211*S5);
    OMcp2_111 = OMcp2_16+ROcp2_46*qd(11);
    OMcp2_211 = OMcp2_26+ROcp2_56*qd(11);
    OMcp2_311 = OMcp2_36+ROcp2_66*qd(11);
    ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
    ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
    ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
    VIcp2_111 = ORcp2_111+qd(1);
    VIcp2_211 = ORcp2_211+qd(2);
    VIcp2_311 = ORcp2_311+qd(3);
    ACcp2_111 = qdd(1)+OMcp2_26*ORcp2_311-OMcp2_36*ORcp2_211+OPcp2_26*RLcp2_311-OPcp2_36*RLcp2_211;
    ACcp2_211 = qdd(2)-OMcp2_16*ORcp2_311+OMcp2_36*ORcp2_111-OPcp2_16*RLcp2_311+OPcp2_36*RLcp2_111;
    ACcp2_311 = qdd(3)+OMcp2_16*ORcp2_211-OMcp2_26*ORcp2_111+OPcp2_16*RLcp2_211-OPcp2_26*RLcp2_111;
    OMcp2_112 = OMcp2_111+ROcp2_46*qd(12);
    OMcp2_212 = OMcp2_211+ROcp2_56*qd(12);
    OMcp2_312 = OMcp2_311+ROcp2_66*qd(12);
    OPcp2_112 = OPcp2_16+ROcp2_46*qdd(11)+ROcp2_46*qdd(12)+qd(11)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qd(12)*(OMcp2_211*ROcp2_66-OMcp2_311*...
 ROcp2_56);
    OPcp2_212 = OPcp2_26+ROcp2_56*qdd(11)+ROcp2_56*qdd(12)-qd(11)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)-qd(12)*(OMcp2_111*ROcp2_66-OMcp2_311*...
 ROcp2_46);
    OPcp2_312 = OPcp2_36+ROcp2_66*qdd(11)+ROcp2_66*qdd(12)+qd(11)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qd(12)*(OMcp2_111*ROcp2_56-OMcp2_211*...
 ROcp2_46);

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_111;
    sens.P(2) = POcp2_211;
    sens.P(3) = POcp2_311;
    sens.R(1,1) = ROcp2_112;
    sens.R(1,2) = ROcp2_212;
    sens.R(1,3) = ROcp2_312;
    sens.R(2,1) = ROcp2_46;
    sens.R(2,2) = ROcp2_56;
    sens.R(2,3) = ROcp2_66;
    sens.R(3,1) = ROcp2_712;
    sens.R(3,2) = ROcp2_812;
    sens.R(3,3) = ROcp2_912;
    sens.V(1) = VIcp2_111;
    sens.V(2) = VIcp2_211;
    sens.V(3) = VIcp2_311;
    sens.OM(1) = OMcp2_112;
    sens.OM(2) = OMcp2_212;
    sens.OM(3) = OMcp2_312;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp2_111_5;
    sens.J(1,6) = JTcp2_111_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp2_311;
    sens.J(2,5) = JTcp2_211_5;
    sens.J(2,6) = JTcp2_211_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp2_211;
    sens.J(3,5) = JTcp2_311_5;
    sens.J(3,6) = JTcp2_311_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,11) = ROcp2_46;
    sens.J(4,12) = ROcp2_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp2_85;
    sens.J(5,11) = ROcp2_56;
    sens.J(5,12) = ROcp2_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp2_95;
    sens.J(6,11) = ROcp2_66;
    sens.J(6,12) = ROcp2_66;
    sens.A(1) = ACcp2_111;
    sens.A(2) = ACcp2_211;
    sens.A(3) = ACcp2_311;
    sens.OMP(1) = OPcp2_112;
    sens.OMP(2) = OPcp2_212;
    sens.OMP(3) = OPcp2_312;
 
% 
case 4, 


% = = Block_1_0_0_4_0_1 = = 
 
% Sensor Kinematics 


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
    OMcp3_25 = qd(5)*C4;
    OMcp3_35 = qd(5)*S4;
    OMcp3_16 = qd(4)+qd(6)*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd(6);
    OMcp3_36 = OMcp3_35+ROcp3_95*qd(6);
    OPcp3_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp3_26 = ROcp3_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp3_35*S5-ROcp3_95*qd(4));
    OPcp3_36 = ROcp3_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp3_25*S5-ROcp3_85*qd(4));

% = = Block_1_0_0_4_0_5 = = 
 
% Sensor Kinematics 


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
    RLcp3_113 = ROcp3_16*s.dpt(1,4)+ROcp3_46*s.dpt(2,4)+s.dpt(3,4)*S5;
    RLcp3_213 = ROcp3_26*s.dpt(1,4)+ROcp3_56*s.dpt(2,4)+ROcp3_85*s.dpt(3,4);
    RLcp3_313 = ROcp3_36*s.dpt(1,4)+ROcp3_66*s.dpt(2,4)+ROcp3_95*s.dpt(3,4);
    POcp3_113 = RLcp3_113+q(1);
    POcp3_213 = RLcp3_213+q(2);
    POcp3_313 = RLcp3_313+q(3);
    JTcp3_113_5 = -(RLcp3_213*S4-RLcp3_313*C4);
    JTcp3_213_5 = RLcp3_113*S4;
    JTcp3_313_5 = -RLcp3_113*C4;
    JTcp3_113_6 = -(RLcp3_213*ROcp3_95-RLcp3_313*ROcp3_85);
    JTcp3_213_6 = RLcp3_113*ROcp3_95-RLcp3_313*S5;
    JTcp3_313_6 = -(RLcp3_113*ROcp3_85-RLcp3_213*S5);
    OMcp3_113 = OMcp3_16+ROcp3_46*qd(13);
    OMcp3_213 = OMcp3_26+ROcp3_56*qd(13);
    OMcp3_313 = OMcp3_36+ROcp3_66*qd(13);
    ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
    ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
    ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
    VIcp3_113 = ORcp3_113+qd(1);
    VIcp3_213 = ORcp3_213+qd(2);
    VIcp3_313 = ORcp3_313+qd(3);
    ACcp3_113 = qdd(1)+OMcp3_26*ORcp3_313-OMcp3_36*ORcp3_213+OPcp3_26*RLcp3_313-OPcp3_36*RLcp3_213;
    ACcp3_213 = qdd(2)-OMcp3_16*ORcp3_313+OMcp3_36*ORcp3_113-OPcp3_16*RLcp3_313+OPcp3_36*RLcp3_113;
    ACcp3_313 = qdd(3)+OMcp3_16*ORcp3_213-OMcp3_26*ORcp3_113+OPcp3_16*RLcp3_213-OPcp3_26*RLcp3_113;
    OMcp3_114 = OMcp3_113+ROcp3_46*qd(14);
    OMcp3_214 = OMcp3_213+ROcp3_56*qd(14);
    OMcp3_314 = OMcp3_313+ROcp3_66*qd(14);
    OPcp3_114 = OPcp3_16+ROcp3_46*qdd(13)+ROcp3_46*qdd(14)+qd(13)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(14)*(OMcp3_213*ROcp3_66-OMcp3_313*...
 ROcp3_56);
    OPcp3_214 = OPcp3_26+ROcp3_56*qdd(13)+ROcp3_56*qdd(14)-qd(13)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(14)*(OMcp3_113*ROcp3_66-OMcp3_313*...
 ROcp3_46);
    OPcp3_314 = OPcp3_36+ROcp3_66*qdd(13)+ROcp3_66*qdd(14)+qd(13)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(14)*(OMcp3_113*ROcp3_56-OMcp3_213*...
 ROcp3_46);

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_113;
    sens.P(2) = POcp3_213;
    sens.P(3) = POcp3_313;
    sens.R(1,1) = ROcp3_114;
    sens.R(1,2) = ROcp3_214;
    sens.R(1,3) = ROcp3_314;
    sens.R(2,1) = ROcp3_46;
    sens.R(2,2) = ROcp3_56;
    sens.R(2,3) = ROcp3_66;
    sens.R(3,1) = ROcp3_714;
    sens.R(3,2) = ROcp3_814;
    sens.R(3,3) = ROcp3_914;
    sens.V(1) = VIcp3_113;
    sens.V(2) = VIcp3_213;
    sens.V(3) = VIcp3_313;
    sens.OM(1) = OMcp3_114;
    sens.OM(2) = OMcp3_214;
    sens.OM(3) = OMcp3_314;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp3_113_5;
    sens.J(1,6) = JTcp3_113_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp3_313;
    sens.J(2,5) = JTcp3_213_5;
    sens.J(2,6) = JTcp3_213_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp3_213;
    sens.J(3,5) = JTcp3_313_5;
    sens.J(3,6) = JTcp3_313_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,13) = ROcp3_46;
    sens.J(4,14) = ROcp3_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp3_85;
    sens.J(5,13) = ROcp3_56;
    sens.J(5,14) = ROcp3_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp3_95;
    sens.J(6,13) = ROcp3_66;
    sens.J(6,14) = ROcp3_66;
    sens.A(1) = ACcp3_113;
    sens.A(2) = ACcp3_213;
    sens.A(3) = ACcp3_313;
    sens.OMP(1) = OPcp3_114;
    sens.OMP(2) = OPcp3_214;
    sens.OMP(3) = OPcp3_314;
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
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
    OMcp4_26 = OMcp4_25+ROcp4_85*qd(6);
    OMcp4_36 = OMcp4_35+ROcp4_95*qd(6);
    OPcp4_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp4_26 = ROcp4_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp4_35*S5-ROcp4_95*qd(4));
    OPcp4_36 = ROcp4_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp4_25*S5-ROcp4_85*qd(4));

% = = Block_1_0_0_5_0_2 = = 
 
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
    POcp4_17 = RLcp4_17+q(1);
    POcp4_27 = RLcp4_27+q(2);
    POcp4_37 = RLcp4_37+q(3);
    OMcp4_17 = OMcp4_16+ROcp4_46*qd(7);
    OMcp4_27 = OMcp4_26+ROcp4_56*qd(7);
    OMcp4_37 = OMcp4_36+ROcp4_66*qd(7);
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27;
    ORcp4_27 = -(OMcp4_16*RLcp4_37-OMcp4_36*RLcp4_17);
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17;
    VIcp4_17 = ORcp4_17+qd(1);
    VIcp4_27 = ORcp4_27+qd(2);
    VIcp4_37 = ORcp4_37+qd(3);
    ACcp4_17 = qdd(1)+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27;
    ACcp4_27 = qdd(2)-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17;
    ACcp4_37 = qdd(3)+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17;
    OMcp4_18 = OMcp4_17+ROcp4_46*qd(8);
    OMcp4_28 = OMcp4_27+ROcp4_56*qd(8);
    OMcp4_38 = OMcp4_37+ROcp4_66*qd(8);
    OPcp4_18 = OPcp4_16+ROcp4_46*qdd(7)+ROcp4_46*qdd(8)+qd(7)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qd(8)*(OMcp4_27*ROcp4_66-OMcp4_37*ROcp4_56);
    OPcp4_28 = OPcp4_26+ROcp4_56*qdd(7)+ROcp4_56*qdd(8)-qd(7)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)-qd(8)*(OMcp4_17*ROcp4_66-OMcp4_37*ROcp4_46);
    OPcp4_38 = OPcp4_36+ROcp4_66*qdd(7)+ROcp4_66*qdd(8)+qd(7)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qd(8)*(OMcp4_17*ROcp4_56-OMcp4_27*ROcp4_46);

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp4_17;
    sens.P(2) = POcp4_27;
    sens.P(3) = POcp4_37;
    sens.R(1,1) = ROcp4_18;
    sens.R(1,2) = ROcp4_28;
    sens.R(1,3) = ROcp4_38;
    sens.R(2,1) = ROcp4_46;
    sens.R(2,2) = ROcp4_56;
    sens.R(2,3) = ROcp4_66;
    sens.R(3,1) = ROcp4_78;
    sens.R(3,2) = ROcp4_88;
    sens.R(3,3) = ROcp4_98;
    sens.V(1) = VIcp4_17;
    sens.V(2) = VIcp4_27;
    sens.V(3) = VIcp4_37;
    sens.OM(1) = OMcp4_18;
    sens.OM(2) = OMcp4_28;
    sens.OM(3) = OMcp4_38;
    sens.A(1) = ACcp4_17;
    sens.A(2) = ACcp4_27;
    sens.A(3) = ACcp4_37;
    sens.OMP(1) = OPcp4_18;
    sens.OMP(2) = OPcp4_28;
    sens.OMP(3) = OPcp4_38;
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
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
    OMcp5_26 = OMcp5_25+ROcp5_85*qd(6);
    OMcp5_36 = OMcp5_35+ROcp5_95*qd(6);
    OPcp5_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp5_26 = ROcp5_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_35*S5-ROcp5_95*qd(4));
    OPcp5_36 = ROcp5_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_25*S5-ROcp5_85*qd(4));

% = = Block_1_0_0_6_0_3 = = 
 
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
    POcp5_19 = RLcp5_19+q(1);
    POcp5_29 = RLcp5_29+q(2);
    POcp5_39 = RLcp5_39+q(3);
    OMcp5_19 = OMcp5_16+ROcp5_46*qd(9);
    OMcp5_29 = OMcp5_26+ROcp5_56*qd(9);
    OMcp5_39 = OMcp5_36+ROcp5_66*qd(9);
    ORcp5_19 = OMcp5_26*RLcp5_39-OMcp5_36*RLcp5_29;
    ORcp5_29 = -(OMcp5_16*RLcp5_39-OMcp5_36*RLcp5_19);
    ORcp5_39 = OMcp5_16*RLcp5_29-OMcp5_26*RLcp5_19;
    VIcp5_19 = ORcp5_19+qd(1);
    VIcp5_29 = ORcp5_29+qd(2);
    VIcp5_39 = ORcp5_39+qd(3);
    ACcp5_19 = qdd(1)+OMcp5_26*ORcp5_39-OMcp5_36*ORcp5_29+OPcp5_26*RLcp5_39-OPcp5_36*RLcp5_29;
    ACcp5_29 = qdd(2)-OMcp5_16*ORcp5_39+OMcp5_36*ORcp5_19-OPcp5_16*RLcp5_39+OPcp5_36*RLcp5_19;
    ACcp5_39 = qdd(3)+OMcp5_16*ORcp5_29-OMcp5_26*ORcp5_19+OPcp5_16*RLcp5_29-OPcp5_26*RLcp5_19;
    OMcp5_110 = OMcp5_19+ROcp5_46*qd(10);
    OMcp5_210 = OMcp5_29+ROcp5_56*qd(10);
    OMcp5_310 = OMcp5_39+ROcp5_66*qd(10);
    OPcp5_110 = OPcp5_16+ROcp5_46*qdd(10)+ROcp5_46*qdd(9)+qd(10)*(OMcp5_29*ROcp5_66-OMcp5_39*ROcp5_56)+qd(9)*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56);
    OPcp5_210 = OPcp5_26+ROcp5_56*qdd(10)+ROcp5_56*qdd(9)-qd(10)*(OMcp5_19*ROcp5_66-OMcp5_39*ROcp5_46)-qd(9)*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46);
    OPcp5_310 = OPcp5_36+ROcp5_66*qdd(10)+ROcp5_66*qdd(9)+qd(10)*(OMcp5_19*ROcp5_56-OMcp5_29*ROcp5_46)+qd(9)*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46);

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp5_19;
    sens.P(2) = POcp5_29;
    sens.P(3) = POcp5_39;
    sens.R(1,1) = ROcp5_110;
    sens.R(1,2) = ROcp5_210;
    sens.R(1,3) = ROcp5_310;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = ROcp5_710;
    sens.R(3,2) = ROcp5_810;
    sens.R(3,3) = ROcp5_910;
    sens.V(1) = VIcp5_19;
    sens.V(2) = VIcp5_29;
    sens.V(3) = VIcp5_39;
    sens.OM(1) = OMcp5_110;
    sens.OM(2) = OMcp5_210;
    sens.OM(3) = OMcp5_310;
    sens.A(1) = ACcp5_19;
    sens.A(2) = ACcp5_29;
    sens.A(3) = ACcp5_39;
    sens.OMP(1) = OPcp5_110;
    sens.OMP(2) = OPcp5_210;
    sens.OMP(3) = OPcp5_310;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
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
    OMcp6_26 = OMcp6_25+ROcp6_85*qd(6);
    OMcp6_36 = OMcp6_35+ROcp6_95*qd(6);
    OPcp6_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp6_26 = ROcp6_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_35*S5-ROcp6_95*qd(4));
    OPcp6_36 = ROcp6_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_25*S5-ROcp6_85*qd(4));

% = = Block_1_0_0_7_0_4 = = 
 
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
    POcp6_111 = RLcp6_111+q(1);
    POcp6_211 = RLcp6_211+q(2);
    POcp6_311 = RLcp6_311+q(3);
    OMcp6_111 = OMcp6_16+ROcp6_46*qd(11);
    OMcp6_211 = OMcp6_26+ROcp6_56*qd(11);
    OMcp6_311 = OMcp6_36+ROcp6_66*qd(11);
    ORcp6_111 = OMcp6_26*RLcp6_311-OMcp6_36*RLcp6_211;
    ORcp6_211 = -(OMcp6_16*RLcp6_311-OMcp6_36*RLcp6_111);
    ORcp6_311 = OMcp6_16*RLcp6_211-OMcp6_26*RLcp6_111;
    VIcp6_111 = ORcp6_111+qd(1);
    VIcp6_211 = ORcp6_211+qd(2);
    VIcp6_311 = ORcp6_311+qd(3);
    ACcp6_111 = qdd(1)+OMcp6_26*ORcp6_311-OMcp6_36*ORcp6_211+OPcp6_26*RLcp6_311-OPcp6_36*RLcp6_211;
    ACcp6_211 = qdd(2)-OMcp6_16*ORcp6_311+OMcp6_36*ORcp6_111-OPcp6_16*RLcp6_311+OPcp6_36*RLcp6_111;
    ACcp6_311 = qdd(3)+OMcp6_16*ORcp6_211-OMcp6_26*ORcp6_111+OPcp6_16*RLcp6_211-OPcp6_26*RLcp6_111;
    OMcp6_112 = OMcp6_111+ROcp6_46*qd(12);
    OMcp6_212 = OMcp6_211+ROcp6_56*qd(12);
    OMcp6_312 = OMcp6_311+ROcp6_66*qd(12);
    OPcp6_112 = OPcp6_16+ROcp6_46*qdd(11)+ROcp6_46*qdd(12)+qd(11)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56)+qd(12)*(OMcp6_211*ROcp6_66-OMcp6_311*...
 ROcp6_56);
    OPcp6_212 = OPcp6_26+ROcp6_56*qdd(11)+ROcp6_56*qdd(12)-qd(11)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46)-qd(12)*(OMcp6_111*ROcp6_66-OMcp6_311*...
 ROcp6_46);
    OPcp6_312 = OPcp6_36+ROcp6_66*qdd(11)+ROcp6_66*qdd(12)+qd(11)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46)+qd(12)*(OMcp6_111*ROcp6_56-OMcp6_211*...
 ROcp6_46);

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_111;
    sens.P(2) = POcp6_211;
    sens.P(3) = POcp6_311;
    sens.R(1,1) = ROcp6_112;
    sens.R(1,2) = ROcp6_212;
    sens.R(1,3) = ROcp6_312;
    sens.R(2,1) = ROcp6_46;
    sens.R(2,2) = ROcp6_56;
    sens.R(2,3) = ROcp6_66;
    sens.R(3,1) = ROcp6_712;
    sens.R(3,2) = ROcp6_812;
    sens.R(3,3) = ROcp6_912;
    sens.V(1) = VIcp6_111;
    sens.V(2) = VIcp6_211;
    sens.V(3) = VIcp6_311;
    sens.OM(1) = OMcp6_112;
    sens.OM(2) = OMcp6_212;
    sens.OM(3) = OMcp6_312;
    sens.A(1) = ACcp6_111;
    sens.A(2) = ACcp6_211;
    sens.A(3) = ACcp6_311;
    sens.OMP(1) = OPcp6_112;
    sens.OMP(2) = OPcp6_212;
    sens.OMP(3) = OPcp6_312;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
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
    OMcp7_26 = OMcp7_25+ROcp7_85*qd(6);
    OMcp7_36 = OMcp7_35+ROcp7_95*qd(6);
    OPcp7_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp7_26 = ROcp7_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_35*S5-ROcp7_95*qd(4));
    OPcp7_36 = ROcp7_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_25*S5-ROcp7_85*qd(4));

% = = Block_1_0_0_8_0_5 = = 
 
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
    POcp7_113 = RLcp7_113+q(1);
    POcp7_213 = RLcp7_213+q(2);
    POcp7_313 = RLcp7_313+q(3);
    OMcp7_113 = OMcp7_16+ROcp7_46*qd(13);
    OMcp7_213 = OMcp7_26+ROcp7_56*qd(13);
    OMcp7_313 = OMcp7_36+ROcp7_66*qd(13);
    ORcp7_113 = OMcp7_26*RLcp7_313-OMcp7_36*RLcp7_213;
    ORcp7_213 = -(OMcp7_16*RLcp7_313-OMcp7_36*RLcp7_113);
    ORcp7_313 = OMcp7_16*RLcp7_213-OMcp7_26*RLcp7_113;
    VIcp7_113 = ORcp7_113+qd(1);
    VIcp7_213 = ORcp7_213+qd(2);
    VIcp7_313 = ORcp7_313+qd(3);
    ACcp7_113 = qdd(1)+OMcp7_26*ORcp7_313-OMcp7_36*ORcp7_213+OPcp7_26*RLcp7_313-OPcp7_36*RLcp7_213;
    ACcp7_213 = qdd(2)-OMcp7_16*ORcp7_313+OMcp7_36*ORcp7_113-OPcp7_16*RLcp7_313+OPcp7_36*RLcp7_113;
    ACcp7_313 = qdd(3)+OMcp7_16*ORcp7_213-OMcp7_26*ORcp7_113+OPcp7_16*RLcp7_213-OPcp7_26*RLcp7_113;
    OMcp7_114 = OMcp7_113+ROcp7_46*qd(14);
    OMcp7_214 = OMcp7_213+ROcp7_56*qd(14);
    OMcp7_314 = OMcp7_313+ROcp7_66*qd(14);
    OPcp7_114 = OPcp7_16+ROcp7_46*qdd(13)+ROcp7_46*qdd(14)+qd(13)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56)+qd(14)*(OMcp7_213*ROcp7_66-OMcp7_313*...
 ROcp7_56);
    OPcp7_214 = OPcp7_26+ROcp7_56*qdd(13)+ROcp7_56*qdd(14)-qd(13)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46)-qd(14)*(OMcp7_113*ROcp7_66-OMcp7_313*...
 ROcp7_46);
    OPcp7_314 = OPcp7_36+ROcp7_66*qdd(13)+ROcp7_66*qdd(14)+qd(13)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46)+qd(14)*(OMcp7_113*ROcp7_56-OMcp7_213*...
 ROcp7_46);

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_113;
    sens.P(2) = POcp7_213;
    sens.P(3) = POcp7_313;
    sens.R(1,1) = ROcp7_114;
    sens.R(1,2) = ROcp7_214;
    sens.R(1,3) = ROcp7_314;
    sens.R(2,1) = ROcp7_46;
    sens.R(2,2) = ROcp7_56;
    sens.R(2,3) = ROcp7_66;
    sens.R(3,1) = ROcp7_714;
    sens.R(3,2) = ROcp7_814;
    sens.R(3,3) = ROcp7_914;
    sens.V(1) = VIcp7_113;
    sens.V(2) = VIcp7_213;
    sens.V(3) = VIcp7_313;
    sens.OM(1) = OMcp7_114;
    sens.OM(2) = OMcp7_214;
    sens.OM(3) = OMcp7_314;
    sens.A(1) = ACcp7_113;
    sens.A(2) = ACcp7_213;
    sens.A(3) = ACcp7_313;
    sens.OMP(1) = OPcp7_114;
    sens.OMP(2) = OPcp7_214;
    sens.OMP(3) = OPcp7_314;

end


% ====== END Task 1 ====== 

  

