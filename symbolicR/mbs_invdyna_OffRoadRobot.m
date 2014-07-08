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
%	==> Generation Date : Tue Jul  8 17:19:30 2014
%
%	==> Project name : OffRoadRobot
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 1971
%
%	==> All Parameter Symbols included
%	==> Generation Time :  0.030 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(14,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  ALPHA11 = qdd(1)-s.g(1);
  ALPHA22 = qdd(2)-s.g(2);
  ALPHA33 = qdd(3)-s.g(3);
  ALPHA24 = ALPHA22*C4+ALPHA33*S4;
  ALPHA34 = -(ALPHA22*S4-ALPHA33*C4);
  OM15 = qd(4)*C5;
  OM35 = qd(4)*S5;
  OMp15 = -(qd(4)*qd(5)*S5-qdd(4)*C5);
  OMp35 = qd(4)*qd(5)*C5+qdd(4)*S5;
  BS25 = qd(5)*OM15;
  BS35 = OM15*OM35;
  BS65 = qd(5)*OM35;
  ALPHA15 = ALPHA11*C5-ALPHA34*S5;
  ALPHA35 = ALPHA11*S5+ALPHA34*C5;
  OM16 = qd(5)*S6+OM15*C6;
  OM26 = qd(5)*C6-OM15*S6;
  OM36 = qd(6)+OM35;
  OMp16 = C6*(OMp15+qd(5)*qd(6))+S6*(qdd(5)-qd(6)*OM15);
  OMp26 = C6*(qdd(5)-qd(6)*OM15)-S6*(OMp15+qd(5)*qd(6));
  OMp36 = qdd(6)+OMp35;
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
  OM27 = qd(7)+OM26;
  OM37 = OM16*S7+OM36*C7;
  OMp17 = C7*(OMp16-qd(7)*OM36)-S7*(OMp36+qd(7)*OM16);
  OMp27 = qdd(7)+OMp26;
  OMp37 = C7*(OMp36+qd(7)*OM16)+S7*(OMp16-qd(7)*OM36);
  BS17 = -(OM27*OM27+OM37*OM37);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BS97 = -(OM17*OM17+OM27*OM27);
  BETA27 = BS27-OMp37;
  BETA37 = BS37+OMp27;
  BETA47 = BS27+OMp37;
  BETA67 = BS67-OMp17;
  BETA77 = BS37-OMp27;
  BETA87 = BS67+OMp17;
  ALPHA17 = C7*(ALPHA16+BETA26*s.dpt(2,1)+BETA36*s.dpt(3,1)+BS16*s.dpt(1,1))-S7*(ALPHA35+BETA76*s.dpt(1,1)+BETA86*s.dpt(2,1)+BS96*s.dpt(3,1));
  ALPHA27 = ALPHA26+BETA46*s.dpt(1,1)+BETA66*s.dpt(3,1)+BS56*s.dpt(2,1);
  ALPHA37 = C7*(ALPHA35+BETA76*s.dpt(1,1)+BETA86*s.dpt(2,1)+BS96*s.dpt(3,1))+S7*(ALPHA16+BETA26*s.dpt(2,1)+BETA36*s.dpt(3,1)+BS16*s.dpt(1,1));
  OM18 = OM17*C8-OM37*S8;
  OM28 = qd(8)+OM27;
  OM38 = OM17*S8+OM37*C8;
  OMp18 = C8*(OMp17-qd(8)*OM37)-S8*(OMp37+qd(8)*OM17);
  OMp28 = qdd(8)+OMp27;
  OMp38 = C8*(OMp37+qd(8)*OM17)+S8*(OMp17-qd(8)*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  OM19 = OM16*C9-OM36*S9;
  OM29 = qd(9)+OM26;
  OM39 = OM16*S9+OM36*C9;
  OMp19 = C9*(OMp16-qd(9)*OM36)-S9*(OMp36+qd(9)*OM16);
  OMp29 = qdd(9)+OMp26;
  OMp39 = C9*(OMp36+qd(9)*OM16)+S9*(OMp16-qd(9)*OM36);
  BS19 = -(OM29*OM29+OM39*OM39);
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS59 = -(OM19*OM19+OM39*OM39);
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BETA29 = BS29-OMp39;
  BETA39 = BS39+OMp29;
  BETA49 = BS29+OMp39;
  BETA69 = BS69-OMp19;
  BETA79 = BS39-OMp29;
  BETA89 = BS69+OMp19;
  ALPHA19 = C9*(ALPHA16+BETA26*s.dpt(2,2)+BETA36*s.dpt(3,2)+BS16*s.dpt(1,2))-S9*(ALPHA35+BETA76*s.dpt(1,2)+BETA86*s.dpt(2,2)+BS96*s.dpt(3,2));
  ALPHA29 = ALPHA26+BETA46*s.dpt(1,2)+BETA66*s.dpt(3,2)+BS56*s.dpt(2,2);
  ALPHA39 = C9*(ALPHA35+BETA76*s.dpt(1,2)+BETA86*s.dpt(2,2)+BS96*s.dpt(3,2))+S9*(ALPHA16+BETA26*s.dpt(2,2)+BETA36*s.dpt(3,2)+BS16*s.dpt(1,2));
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd(10)+OM29;
  OM310 = OM19*S10+OM39*C10;
  OMp110 = C10*(OMp19-qd(10)*OM39)-S10*(OMp39+qd(10)*OM19);
  OMp210 = qdd(10)+OMp29;
  OMp310 = C10*(OMp39+qd(10)*OM19)+S10*(OMp19-qd(10)*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  OM111 = OM16*C11-OM36*S11;
  OM211 = qd(11)+OM26;
  OM311 = OM16*S11+OM36*C11;
  OMp111 = C11*(OMp16-qd(11)*OM36)-S11*(OMp36+qd(11)*OM16);
  OMp211 = qdd(11)+OMp26;
  OMp311 = C11*(OMp36+qd(11)*OM16)+S11*(OMp16-qd(11)*OM36);
  BS111 = -(OM211*OM211+OM311*OM311);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS511 = -(OM111*OM111+OM311*OM311);
  BS611 = OM211*OM311;
  BS911 = -(OM111*OM111+OM211*OM211);
  BETA211 = BS211-OMp311;
  BETA311 = BS311+OMp211;
  BETA411 = BS211+OMp311;
  BETA611 = BS611-OMp111;
  BETA711 = BS311-OMp211;
  BETA811 = BS611+OMp111;
  ALPHA111 = C11*(ALPHA16+BETA26*s.dpt(2,3)+BETA36*s.dpt(3,3)+BS16*s.dpt(1,3))-S11*(ALPHA35+BETA76*s.dpt(1,3)+BETA86*s.dpt(2,3)+BS96*s.dpt(3,3));
  ALPHA211 = ALPHA26+BETA46*s.dpt(1,3)+BETA66*s.dpt(3,3)+BS56*s.dpt(2,3);
  ALPHA311 = C11*(ALPHA35+BETA76*s.dpt(1,3)+BETA86*s.dpt(2,3)+BS96*s.dpt(3,3))+S11*(ALPHA16+BETA26*s.dpt(2,3)+BETA36*s.dpt(3,3)+BS16*s.dpt(1,3));
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd(12)+OM211;
  OM312 = OM111*S12+OM311*C12;
  OMp112 = C12*(OMp111-qd(12)*OM311)-S12*(OMp311+qd(12)*OM111);
  OMp212 = qdd(12)+OMp211;
  OMp312 = C12*(OMp311+qd(12)*OM111)+S12*(OMp111-qd(12)*OM311);
  BS212 = OM112*OM212;
  BS312 = OM112*OM312;
  BS612 = OM212*OM312;
  OM113 = OM16*C13-OM36*S13;
  OM213 = qd(13)+OM26;
  OM313 = OM16*S13+OM36*C13;
  OMp113 = C13*(OMp16-qd(13)*OM36)-S13*(OMp36+qd(13)*OM16);
  OMp213 = qdd(13)+OMp26;
  OMp313 = C13*(OMp36+qd(13)*OM16)+S13*(OMp16-qd(13)*OM36);
  BS113 = -(OM213*OM213+OM313*OM313);
  BS213 = OM113*OM213;
  BS313 = OM113*OM313;
  BS513 = -(OM113*OM113+OM313*OM313);
  BS613 = OM213*OM313;
  BS913 = -(OM113*OM113+OM213*OM213);
  BETA213 = BS213-OMp313;
  BETA313 = BS313+OMp213;
  BETA413 = BS213+OMp313;
  BETA613 = BS613-OMp113;
  BETA713 = BS313-OMp213;
  BETA813 = BS613+OMp113;
  ALPHA113 = C13*(ALPHA16+BETA26*s.dpt(2,4)+BETA36*s.dpt(3,4)+BS16*s.dpt(1,4))-S13*(ALPHA35+BETA76*s.dpt(1,4)+BETA86*s.dpt(2,4)+BS96*s.dpt(3,4));
  ALPHA213 = ALPHA26+BETA46*s.dpt(1,4)+BETA66*s.dpt(3,4)+BS56*s.dpt(2,4);
  ALPHA313 = C13*(ALPHA35+BETA76*s.dpt(1,4)+BETA86*s.dpt(2,4)+BS96*s.dpt(3,4))+S13*(ALPHA16+BETA26*s.dpt(2,4)+BETA36*s.dpt(3,4)+BS16*s.dpt(1,4));
  OM114 = OM113*C14-OM313*S14;
  OM214 = qd(14)+OM213;
  OM314 = OM113*S14+OM313*C14;
  OMp114 = C14*(OMp113-qd(14)*OM313)-S14*(OMp313+qd(14)*OM113);
  OMp214 = qdd(14)+OMp213;
  OMp314 = C14*(OMp313+qd(14)*OM113)+S14*(OMp113-qd(14)*OM313);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS614 = OM214*OM314;
 
% Backward Dynamics 

  Fs114 = -(s.frc(1,14)+s.m(14)*(s.l(1,14)*(OM214*OM214+OM314*OM314)-s.l(2,14)*(BS214-OMp314)-s.l(3,14)*(BS314+OMp214)-C14*(ALPHA113+s.dpt(2,11)*...
 BETA213+s.dpt(3,11)*BETA313+BS113*s.dpt(1,11))+S14*(ALPHA313+s.dpt(2,11)*BETA813+s.dpt(3,11)*BS913+BETA713*s.dpt(1,11))));
  Fs214 = -(s.frc(2,14)-s.m(14)*(ALPHA213+s.dpt(2,11)*BS513+s.dpt(3,11)*BETA613+s.l(1,14)*(BS214+OMp314)-s.l(2,14)*(OM114*OM114+OM314*OM314)+...
 BETA413*s.dpt(1,11)+s.l(3,14)*(BS614-OMp114)));
  Fs314 = -(s.frc(3,14)-s.m(14)*(s.l(1,14)*(BS314-OMp214)+s.l(2,14)*(BS614+OMp114)-s.l(3,14)*(OM114*OM114+OM214*OM214)+C14*(ALPHA313+s.dpt(2,11)*...
 BETA813+s.dpt(3,11)*BS913+BETA713*s.dpt(1,11))+S14*(ALPHA113+s.dpt(2,11)*BETA213+s.dpt(3,11)*BETA313+BS113*s.dpt(1,11))));
  Cq114 = -(s.trq(1,14)-s.In(1,14)*OMp114-s.In(2,14)*OMp214-s.In(3,14)*OMp314-s.l(2,14)*Fs314+Fs214*s.l(3,14)+OM314*(s.In(5,14)*OM214+s.In(6,14)*...
 OM314-s.In(9,14)*OM214));
  Cq214 = -(s.trq(2,14)-s.In(5,14)*OMp214-s.In(6,14)*OMp314+s.l(1,14)*Fs314-Fs114*s.l(3,14)-OM314*(s.In(1,14)*OM114+s.In(2,14)*OM214+s.In(3,14)*...
 OM314-s.In(9,14)*OM114));
  Cq314 = -(s.trq(3,14)-s.In(9,14)*OMp314-s.l(1,14)*Fs214+s.l(2,14)*Fs114-OM114*(s.In(5,14)*OM214+s.In(6,14)*OM314)+OM214*(s.In(1,14)*OM114+...
 s.In(2,14)*OM214+s.In(3,14)*OM314));
  Fs113 = -(s.frc(1,13)-s.m(13)*(ALPHA113+s.l(1,13)*BS113+s.l(2,13)*BETA213+s.l(3,13)*BETA313));
  Fs213 = -(s.frc(2,13)-s.m(13)*(ALPHA213+s.l(1,13)*BETA413+s.l(2,13)*BS513+s.l(3,13)*BETA613));
  Fs313 = -(s.frc(3,13)-s.m(13)*(ALPHA313+s.l(1,13)*BETA713+s.l(2,13)*BETA813+s.l(3,13)*BS913));
  Fq113 = Fs113+Fs114*C14+Fs314*S14;
  Fq213 = Fs213+Fs214;
  Fq313 = Fs313-Fs114*S14+Fs314*C14;
  Cq113 = -(s.trq(1,13)-s.In(1,13)*OMp113-s.In(2,13)*OMp213-s.In(3,13)*OMp313+s.dpt(2,11)*(Fs114*S14-Fs314*C14)+s.dpt(3,11)*Fs214-s.l(2,13)*Fs313...
 +s.l(3,13)*Fs213-Cq114*C14-Cq314*S14+OM313*(s.In(5,13)*OM213+s.In(6,13)*OM313-s.In(9,13)*OM213));
  Cq213 = -(s.trq(2,13)-Cq214-s.In(5,13)*OMp213-s.In(6,13)*OMp313-s.dpt(3,11)*(Fs114*C14+Fs314*S14)+s.l(1,13)*Fs313-s.l(3,13)*Fs113-OM313*(...
 s.In(1,13)*OM113+s.In(2,13)*OM213+s.In(3,13)*OM313-s.In(9,13)*OM113)-s.dpt(1,11)*(Fs114*S14-Fs314*C14));
  Cq313 = -(s.trq(3,13)-s.In(9,13)*OMp313+s.dpt(2,11)*(Fs114*C14+Fs314*S14)-s.l(1,13)*Fs213+s.l(2,13)*Fs113+Cq114*S14-Cq314*C14-Fs214*s.dpt(1,11)...
 -OM113*(s.In(5,13)*OM213+s.In(6,13)*OM313)+OM213*(s.In(1,13)*OM113+s.In(2,13)*OM213+s.In(3,13)*OM313));
  Fs112 = -(s.frc(1,12)+s.m(12)*(s.l(1,12)*(OM212*OM212+OM312*OM312)-s.l(2,12)*(BS212-OMp312)-s.l(3,12)*(BS312+OMp212)-C12*(ALPHA111+s.dpt(2,9)*...
 BETA211+s.dpt(3,9)*BETA311+BS111*s.dpt(1,9))+S12*(ALPHA311+s.dpt(2,9)*BETA811+s.dpt(3,9)*BS911+BETA711*s.dpt(1,9))));
  Fs212 = -(s.frc(2,12)-s.m(12)*(ALPHA211+s.dpt(2,9)*BS511+s.dpt(3,9)*BETA611+s.l(1,12)*(BS212+OMp312)-s.l(2,12)*(OM112*OM112+OM312*OM312)+...
 BETA411*s.dpt(1,9)+s.l(3,12)*(BS612-OMp112)));
  Fs312 = -(s.frc(3,12)-s.m(12)*(s.l(1,12)*(BS312-OMp212)+s.l(2,12)*(BS612+OMp112)-s.l(3,12)*(OM112*OM112+OM212*OM212)+C12*(ALPHA311+s.dpt(2,9)*...
 BETA811+s.dpt(3,9)*BS911+BETA711*s.dpt(1,9))+S12*(ALPHA111+s.dpt(2,9)*BETA211+s.dpt(3,9)*BETA311+BS111*s.dpt(1,9))));
  Cq112 = -(s.trq(1,12)-s.In(1,12)*OMp112-s.In(2,12)*OMp212-s.In(3,12)*OMp312-s.l(2,12)*Fs312+Fs212*s.l(3,12)+OM312*(s.In(5,12)*OM212+s.In(6,12)*...
 OM312-s.In(9,12)*OM212));
  Cq212 = -(s.trq(2,12)-s.In(5,12)*OMp212-s.In(6,12)*OMp312+s.l(1,12)*Fs312-Fs112*s.l(3,12)-OM312*(s.In(1,12)*OM112+s.In(2,12)*OM212+s.In(3,12)*...
 OM312-s.In(9,12)*OM112));
  Cq312 = -(s.trq(3,12)-s.In(9,12)*OMp312-s.l(1,12)*Fs212+s.l(2,12)*Fs112-OM112*(s.In(5,12)*OM212+s.In(6,12)*OM312)+OM212*(s.In(1,12)*OM112+...
 s.In(2,12)*OM212+s.In(3,12)*OM312));
  Fs111 = -(s.frc(1,11)-s.m(11)*(ALPHA111+s.l(1,11)*BS111+s.l(2,11)*BETA211+s.l(3,11)*BETA311));
  Fs211 = -(s.frc(2,11)-s.m(11)*(ALPHA211+s.l(1,11)*BETA411+s.l(2,11)*BS511+s.l(3,11)*BETA611));
  Fs311 = -(s.frc(3,11)-s.m(11)*(ALPHA311+s.l(1,11)*BETA711+s.l(2,11)*BETA811+s.l(3,11)*BS911));
  Fq111 = Fs111+Fs112*C12+Fs312*S12;
  Fq211 = Fs211+Fs212;
  Fq311 = Fs311-Fs112*S12+Fs312*C12;
  Cq111 = -(s.trq(1,11)-s.In(1,11)*OMp111-s.In(2,11)*OMp211-s.In(3,11)*OMp311+s.dpt(2,9)*(Fs112*S12-Fs312*C12)+s.dpt(3,9)*Fs212-s.l(2,11)*Fs311+...
 s.l(3,11)*Fs211-Cq112*C12-Cq312*S12+OM311*(s.In(5,11)*OM211+s.In(6,11)*OM311-s.In(9,11)*OM211));
  Cq211 = -(s.trq(2,11)-Cq212-s.In(5,11)*OMp211-s.In(6,11)*OMp311-s.dpt(3,9)*(Fs112*C12+Fs312*S12)+s.l(1,11)*Fs311-s.l(3,11)*Fs111-OM311*(...
 s.In(1,11)*OM111+s.In(2,11)*OM211+s.In(3,11)*OM311-s.In(9,11)*OM111)-s.dpt(1,9)*(Fs112*S12-Fs312*C12));
  Cq311 = -(s.trq(3,11)-s.In(9,11)*OMp311+s.dpt(2,9)*(Fs112*C12+Fs312*S12)-s.l(1,11)*Fs211+s.l(2,11)*Fs111+Cq112*S12-Cq312*C12-Fs212*s.dpt(1,9)-...
 OM111*(s.In(5,11)*OM211+s.In(6,11)*OM311)+OM211*(s.In(1,11)*OM111+s.In(2,11)*OM211+s.In(3,11)*OM311));
  Fs110 = -(s.frc(1,10)+s.m(10)*(s.l(1,10)*(OM210*OM210+OM310*OM310)-s.l(2,10)*(BS210-OMp310)-s.l(3,10)*(BS310+OMp210)-C10*(ALPHA19+s.dpt(2,7)*...
 BETA29+s.dpt(3,7)*BETA39+BS19*s.dpt(1,7))+S10*(ALPHA39+s.dpt(2,7)*BETA89+s.dpt(3,7)*BS99+BETA79*s.dpt(1,7))));
  Fs210 = -(s.frc(2,10)-s.m(10)*(ALPHA29+s.dpt(2,7)*BS59+s.dpt(3,7)*BETA69+s.l(1,10)*(BS210+OMp310)-s.l(2,10)*(OM110*OM110+OM310*OM310)+BETA49*...
 s.dpt(1,7)+s.l(3,10)*(BS610-OMp110)));
  Fs310 = -(s.frc(3,10)-s.m(10)*(s.l(1,10)*(BS310-OMp210)+s.l(2,10)*(BS610+OMp110)-s.l(3,10)*(OM110*OM110+OM210*OM210)+C10*(ALPHA39+s.dpt(2,7)*...
 BETA89+s.dpt(3,7)*BS99+BETA79*s.dpt(1,7))+S10*(ALPHA19+s.dpt(2,7)*BETA29+s.dpt(3,7)*BETA39+BS19*s.dpt(1,7))));
  Cq110 = -(s.trq(1,10)-s.In(1,10)*OMp110-s.In(2,10)*OMp210-s.In(3,10)*OMp310-s.l(2,10)*Fs310+Fs210*s.l(3,10)+OM310*(s.In(5,10)*OM210+s.In(6,10)*...
 OM310-s.In(9,10)*OM210));
  Cq210 = -(s.trq(2,10)-s.In(5,10)*OMp210-s.In(6,10)*OMp310+s.l(1,10)*Fs310-Fs110*s.l(3,10)-OM310*(s.In(1,10)*OM110+s.In(2,10)*OM210+s.In(3,10)*...
 OM310-s.In(9,10)*OM110));
  Cq310 = -(s.trq(3,10)-s.In(9,10)*OMp310-s.l(1,10)*Fs210+s.l(2,10)*Fs110-OM110*(s.In(5,10)*OM210+s.In(6,10)*OM310)+OM210*(s.In(1,10)*OM110+...
 s.In(2,10)*OM210+s.In(3,10)*OM310));
  Fs19 = -(s.frc(1,9)-s.m(9)*(ALPHA19+s.l(1,9)*BS19+s.l(2,9)*BETA29+s.l(3,9)*BETA39));
  Fs29 = -(s.frc(2,9)-s.m(9)*(ALPHA29+s.l(1,9)*BETA49+s.l(2,9)*BS59+s.l(3,9)*BETA69));
  Fs39 = -(s.frc(3,9)-s.m(9)*(ALPHA39+s.l(1,9)*BETA79+s.l(2,9)*BETA89+s.l(3,9)*BS99));
  Fq19 = Fs19+Fs110*C10+Fs310*S10;
  Fq29 = Fs210+Fs29;
  Fq39 = Fs39-Fs110*S10+Fs310*C10;
  Cq19 = -(s.trq(1,9)-s.In(1,9)*OMp19-s.In(2,9)*OMp29-s.In(3,9)*OMp39+s.dpt(2,7)*(Fs110*S10-Fs310*C10)+s.dpt(3,7)*Fs210-s.l(2,9)*Fs39+s.l(3,9)*...
 Fs29-Cq110*C10-Cq310*S10+OM39*(s.In(5,9)*OM29+s.In(6,9)*OM39-s.In(9,9)*OM29));
  Cq29 = -(s.trq(2,9)-Cq210-s.In(5,9)*OMp29-s.In(6,9)*OMp39-s.dpt(3,7)*(Fs110*C10+Fs310*S10)+s.l(1,9)*Fs39-s.l(3,9)*Fs19-OM39*(s.In(1,9)*OM19+...
 s.In(2,9)*OM29+s.In(3,9)*OM39-s.In(9,9)*OM19)-s.dpt(1,7)*(Fs110*S10-Fs310*C10));
  Cq39 = -(s.trq(3,9)-s.In(9,9)*OMp39+s.dpt(2,7)*(Fs110*C10+Fs310*S10)-s.l(1,9)*Fs29+s.l(2,9)*Fs19+Cq110*S10-Cq310*C10-Fs210*s.dpt(1,7)-OM19*(...
 s.In(5,9)*OM29+s.In(6,9)*OM39)+OM29*(s.In(1,9)*OM19+s.In(2,9)*OM29+s.In(3,9)*OM39));
  Fs18 = -(s.frc(1,8)+s.m(8)*(s.l(1,8)*(OM28*OM28+OM38*OM38)-s.l(2,8)*(BS28-OMp38)-s.l(3,8)*(BS38+OMp28)-C8*(ALPHA17+s.dpt(2,5)*BETA27+s.dpt(3,5)...
 *BETA37+BS17*s.dpt(1,5))+S8*(ALPHA37+s.dpt(2,5)*BETA87+s.dpt(3,5)*BS97+BETA77*s.dpt(1,5))));
  Fs28 = -(s.frc(2,8)-s.m(8)*(ALPHA27+s.dpt(2,5)*BS57+s.dpt(3,5)*BETA67+s.l(1,8)*(BS28+OMp38)-s.l(2,8)*(OM18*OM18+OM38*OM38)+BETA47*s.dpt(1,5)+...
 s.l(3,8)*(BS68-OMp18)));
  Fs38 = -(s.frc(3,8)-s.m(8)*(s.l(1,8)*(BS38-OMp28)+s.l(2,8)*(BS68+OMp18)-s.l(3,8)*(OM18*OM18+OM28*OM28)+C8*(ALPHA37+s.dpt(2,5)*BETA87+s.dpt(3,5)...
 *BS97+BETA77*s.dpt(1,5))+S8*(ALPHA17+s.dpt(2,5)*BETA27+s.dpt(3,5)*BETA37+BS17*s.dpt(1,5))));
  Cq18 = -(s.trq(1,8)-s.In(1,8)*OMp18-s.In(2,8)*OMp28-s.In(3,8)*OMp38-s.l(2,8)*Fs38+Fs28*s.l(3,8)+OM38*(s.In(5,8)*OM28+s.In(6,8)*OM38-s.In(9,8)*...
 OM28));
  Cq28 = -(s.trq(2,8)-s.In(5,8)*OMp28-s.In(6,8)*OMp38+s.l(1,8)*Fs38-Fs18*s.l(3,8)-OM38*(s.In(1,8)*OM18+s.In(2,8)*OM28+s.In(3,8)*OM38-s.In(9,8)*...
 OM18));
  Cq38 = -(s.trq(3,8)-s.In(9,8)*OMp38-s.l(1,8)*Fs28+s.l(2,8)*Fs18-OM18*(s.In(5,8)*OM28+s.In(6,8)*OM38)+OM28*(s.In(1,8)*OM18+s.In(2,8)*OM28+...
 s.In(3,8)*OM38));
  Fs17 = -(s.frc(1,7)-s.m(7)*(ALPHA17+s.l(1,7)*BS17+s.l(2,7)*BETA27+s.l(3,7)*BETA37));
  Fs27 = -(s.frc(2,7)-s.m(7)*(ALPHA27+s.l(1,7)*BETA47+s.l(2,7)*BS57+s.l(3,7)*BETA67));
  Fs37 = -(s.frc(3,7)-s.m(7)*(ALPHA37+s.l(1,7)*BETA77+s.l(2,7)*BETA87+s.l(3,7)*BS97));
  Fq17 = Fs17+Fs18*C8+Fs38*S8;
  Fq27 = Fs27+Fs28;
  Fq37 = Fs37-Fs18*S8+Fs38*C8;
  Cq17 = -(s.trq(1,7)-s.In(1,7)*OMp17-s.In(2,7)*OMp27-s.In(3,7)*OMp37+s.dpt(2,5)*(Fs18*S8-Fs38*C8)+s.dpt(3,5)*Fs28-s.l(2,7)*Fs37+s.l(3,7)*Fs27-...
 Cq18*C8-Cq38*S8+OM37*(s.In(5,7)*OM27+s.In(6,7)*OM37-s.In(9,7)*OM27));
  Cq27 = -(s.trq(2,7)-Cq28-s.In(5,7)*OMp27-s.In(6,7)*OMp37-s.dpt(3,5)*(Fs18*C8+Fs38*S8)+s.l(1,7)*Fs37-s.l(3,7)*Fs17-OM37*(s.In(1,7)*OM17+...
 s.In(2,7)*OM27+s.In(3,7)*OM37-s.In(9,7)*OM17)-s.dpt(1,5)*(Fs18*S8-Fs38*C8));
  Cq37 = -(s.trq(3,7)-s.In(9,7)*OMp37+s.dpt(2,5)*(Fs18*C8+Fs38*S8)-s.l(1,7)*Fs27+s.l(2,7)*Fs17+Cq18*S8-Cq38*C8-Fs28*s.dpt(1,5)-OM17*(s.In(5,7)*...
 OM27+s.In(6,7)*OM37)+OM27*(s.In(1,7)*OM17+s.In(2,7)*OM27+s.In(3,7)*OM37));
  Fs16 = -(s.frc(1,6)-s.m(6)*(ALPHA16+s.l(1,6)*BS16+s.l(2,6)*BETA26+s.l(3,6)*BETA36));
  Fs26 = -(s.frc(2,6)-s.m(6)*(ALPHA26+s.l(1,6)*BETA46+s.l(2,6)*BS56+s.l(3,6)*BETA66));
  Fs36 = -(s.frc(3,6)-s.m(6)*(ALPHA35+s.l(1,6)*BETA76+s.l(2,6)*BETA86+s.l(3,6)*BS96));
  Fq16 = Fs16+Fq111*C11+Fq113*C13+Fq17*C7+Fq19*C9+Fq311*S11+Fq313*S13+Fq37*S7+Fq39*S9;
  Fq26 = Fq211+Fq213+Fq27+Fq29+Fs26;
  Cq16 = -(s.trq(1,6)-s.In(1,6)*OMp16-s.In(2,6)*OMp26-s.In(3,6)*OMp36-s.l(2,6)*Fs36+s.l(3,6)*Fs26-Cq111*C11-Cq113*C13-Cq17*C7-Cq19*C9-Cq311*S11-...
 Cq313*S13-Cq37*S7-Cq39*S9+Fq211*s.dpt(3,3)+Fq213*s.dpt(3,4)+Fq27*s.dpt(3,1)+Fq29*s.dpt(3,2)+OM36*(s.In(5,6)*OM26+s.In(6,6)*OM36-s.In(9,6)*OM26)+...
 s.dpt(2,1)*(Fq17*S7-Fq37*C7)+s.dpt(2,2)*(Fq19*S9-Fq39*C9)+s.dpt(2,3)*(Fq111*S11-Fq311*C11)+s.dpt(2,4)*(Fq113*S13-Fq313*C13));
  Cq26 = -(s.trq(2,6)-Cq211-Cq213-Cq27-Cq29-s.In(5,6)*OMp26-s.In(6,6)*OMp36+s.l(1,6)*Fs36-s.l(3,6)*Fs16-OM36*(s.In(1,6)*OM16+s.In(2,6)*OM26+...
 s.In(3,6)*OM36-s.In(9,6)*OM16)-s.dpt(1,1)*(Fq17*S7-Fq37*C7)-s.dpt(1,2)*(Fq19*S9-Fq39*C9)-s.dpt(1,3)*(Fq111*S11-Fq311*C11)-s.dpt(1,4)*(Fq113*S13-Fq313...
 *C13)-s.dpt(3,1)*(Fq17*C7+Fq37*S7)-s.dpt(3,2)*(Fq19*C9+Fq39*S9)-s.dpt(3,3)*(Fq111*C11+Fq311*S11)-s.dpt(3,4)*(Fq113*C13+Fq313*S13));
  Cq36 = -(s.trq(3,6)-s.In(9,6)*OMp36-s.l(1,6)*Fs26+s.l(2,6)*Fs16+Cq111*S11+Cq113*S13+Cq17*S7+Cq19*S9-Cq311*C11-Cq313*C13-Cq37*C7-Cq39*C9-Fq211*...
 s.dpt(1,3)-Fq213*s.dpt(1,4)-Fq27*s.dpt(1,1)-Fq29*s.dpt(1,2)-OM16*(s.In(5,6)*OM26+s.In(6,6)*OM36)+OM26*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)+...
 s.dpt(2,1)*(Fq17*C7+Fq37*S7)+s.dpt(2,2)*(Fq19*C9+Fq39*S9)+s.dpt(2,3)*(Fq111*C11+Fq311*S11)+s.dpt(2,4)*(Fq113*C13+Fq313*S13));
  Fs15 = -(s.frc(1,5)-s.m(5)*(ALPHA15-s.l(1,5)*(qd(5)*qd(5)+OM35*OM35)+s.l(2,5)*(BS25-OMp35)+s.l(3,5)*(qdd(5)+BS35)));
  Fs25 = -(s.frc(2,5)-s.m(5)*(ALPHA24-qd(4)*qd(4)*s.l(2,5)+s.l(1,5)*(BS25+OMp35)+s.l(3,5)*(BS65-OMp15)));
  Fs35 = -(s.frc(3,5)-s.m(5)*(ALPHA35-s.l(1,5)*(qdd(5)-BS35)+s.l(2,5)*(BS65+OMp15)-s.l(3,5)*(qd(5)*qd(5)+OM15*OM15)));
  Fq15 = Fs15+Fq16*C6-Fq26*S6;
  Fq35 = Fs35+Fs36-Fq111*S11-Fq113*S13-Fq17*S7-Fq19*S9+Fq311*C11+Fq313*C13+Fq37*C7+Fq39*C9;
  Cq25 = -(s.trq(2,5)-qdd(5)*s.In(5,5)-s.In(6,5)*OMp35+s.l(1,5)*Fs35-s.l(3,5)*Fs15-Cq16*S6-Cq26*C6-OM35*(qd(5)*s.In(2,5)+s.In(1,5)*OM15+s.In(3,5)...
 *OM35-s.In(9,5)*OM15));
  Fs24 = -(s.frc(2,4)-s.m(4)*(ALPHA24-qd(4)*qd(4)*s.l(2,4)-qdd(4)*s.l(3,4)));
  Fs34 = -(s.frc(3,4)-s.m(4)*(ALPHA34-qd(4)*qd(4)*s.l(3,4)+qdd(4)*s.l(2,4)));
  Fq24 = Fs24+Fs25+Fq16*S6+Fq26*C6;
  Fq34 = Fs34-Fq15*S5+Fq35*C5;
  Cq14 = -(s.trq(1,4)-qdd(4)*s.In(1,4)-s.l(2,4)*Fs34+s.l(3,4)*Fs24+C5*(s.trq(1,5)-qdd(5)*s.In(2,5)-s.In(1,5)*OMp15-s.In(3,5)*OMp35-s.l(2,5)*Fs35+...
 s.l(3,5)*Fs25-Cq16*C6+Cq26*S6+OM35*(qd(5)*(s.In(5,5)-s.In(9,5))+s.In(6,5)*OM35))+S5*(s.trq(3,5)-Cq36+qd(5)*(qd(5)*s.In(2,5)+s.In(1,5)*OM15+s.In(3,5)*...
 OM35)-s.In(9,5)*OMp35-s.l(1,5)*Fs25+s.l(2,5)*Fs15-OM15*(qd(5)*s.In(5,5)+s.In(6,5)*OM35)));
  Fq33 = -(s.frc(3,3)-s.m(3)*ALPHA33-Fq24*S4-Fq34*C4);
  Fq22 = -(s.frc(2,2)+s.frc(2,3)-s.m(2)*ALPHA22-s.m(3)*ALPHA22-Fq24*C4+Fq34*S4);
  Fq11 = -(s.frc(1,1)+s.frc(1,2)+s.frc(1,3)+s.frc(1,4)-s.m(1)*ALPHA11-s.m(2)*ALPHA11-s.m(3)*ALPHA11-s.m(4)*ALPHA11-Fq15*C5-Fq35*S5);

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Fq11;
  Qq(2) = Fq22;
  Qq(3) = Fq33;
  Qq(4) = Cq14;
  Qq(5) = Cq25;
  Qq(6) = Cq36;
  Qq(7) = Cq27;
  Qq(8) = Cq28;
  Qq(9) = Cq29;
  Qq(10) = Cq210;
  Qq(11) = Cq211;
  Qq(12) = Cq212;
  Qq(13) = Cq213;
  Qq(14) = Cq214;

% ====== END Task 0 ====== 

  

