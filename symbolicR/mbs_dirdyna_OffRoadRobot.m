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
%	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
%	==> Flops complexity : 6000
%
%	==> All Parameter Symbols included
%	==> Generation Time :  0.060 seconds
%	==> Post-Processing :  0.080 seconds
%
%-------------------------------------------------------------
%
function [M,c] = dirdyna(s,tsim,usrfun)

 M = zeros(14,14);
 c = zeros(14,1);

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

% = = Block_0_1_0_0_0_1 = = 
 
% Forward Kinematics 

  AlF24 = -(s.g(2)*C4+s.g(3)*S4);
  AlF34 = s.g(2)*S4-s.g(3)*C4;
  OM15 = qd(4)*C5;
  OM35 = qd(4)*S5;
  OpF15 = -qd(4)*qd(5)*S5;
  OpF35 = qd(4)*qd(5)*C5;
  BS35 = OM15*OM35;
  AlF15 = -(s.g(1)*C5+AlF34*S5);
  AlF35 = -(s.g(1)*S5-AlF34*C5);
  AlM15_2 = S4*S5;
  AlM35_2 = -S4*C5;
  AlM15_3 = -C4*S5;
  AlM35_3 = C4*C5;
  OM16 = qd(5)*S6+OM15*C6;
  OM26 = qd(5)*C6-OM15*S6;
  OM36 = qd(6)+OM35;
  OpF16 = -(qd(6)*OM15*S6-C6*(OpF15+qd(5)*qd(6)));
  OpF26 = -(qd(6)*OM15*C6+S6*(OpF15+qd(5)*qd(6)));
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BeF26 = BS26-OpF35;
  BeF36 = BS36+OpF26;
  BeF46 = BS26+OpF35;
  BeF66 = BS66-OpF16;
  BeF76 = BS36-OpF26;
  BeF86 = BS66+OpF16;
  AlF16 = AlF15*C6+AlF24*S6;
  AlF26 = -(AlF15*S6-AlF24*C6);
  AlM16_1 = C5*C6;
  AlM26_1 = -C5*S6;
  AlM16_2 = AlM15_2*C6+C4*S6;
  AlM26_2 = -(AlM15_2*S6-C4*C6);
  AlM16_3 = AlM15_3*C6+S4*S6;
  AlM26_3 = -(AlM15_3*S6-S4*C6);
  OpM16_4 = C5*C6;
  OpM26_4 = -C5*S6;

% = = Block_0_1_0_1_0_2 = = 
 
% Trigonometric Variables  

  C7p8 = C7*C8-S7*S8;
  S7p8 = C7*S8+S7*C8;
 
% Forward Kinematics 

  OM17 = OM16*C7-OM36*S7;
  OM27 = qd(7)+OM26;
  OM37 = OM16*S7+OM36*C7;
  OpF17 = C7*(OpF16-qd(7)*OM36)-S7*(OpF35+qd(7)*OM16);
  OpF37 = C7*(OpF35+qd(7)*OM16)+S7*(OpF16-qd(7)*OM36);
  BS17 = -(OM27*OM27+OM37*OM37);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BS97 = -(OM17*OM17+OM27*OM27);
  BeF27 = BS27-OpF37;
  BeF37 = BS37+OpF26;
  BeF47 = BS27+OpF37;
  BeF67 = BS67-OpF17;
  BeF77 = BS37-OpF26;
  BeF87 = BS67+OpF17;
  AlF17 = C7*(AlF16+BS16*s.dpt(1,1)+BeF26*s.dpt(2,1)+BeF36*s.dpt(3,1))-S7*(AlF35+BS96*s.dpt(3,1)+BeF76*s.dpt(1,1)+BeF86*s.dpt(2,1));
  AlF27 = AlF26+BS56*s.dpt(2,1)+BeF46*s.dpt(1,1)+BeF66*s.dpt(3,1);
  AlF37 = C7*(AlF35+BS96*s.dpt(3,1)+BeF76*s.dpt(1,1)+BeF86*s.dpt(2,1))+S7*(AlF16+BS16*s.dpt(1,1)+BeF26*s.dpt(2,1)+BeF36*s.dpt(3,1));
  AlM17_1 = AlM16_1*C7-S5*S7;
  AlM37_1 = AlM16_1*S7+S5*C7;
  AlM17_2 = AlM16_2*C7-AlM35_2*S7;
  AlM37_2 = AlM16_2*S7+AlM35_2*C7;
  AlM17_3 = AlM16_3*C7-AlM35_3*S7;
  AlM37_3 = AlM16_3*S7+AlM35_3*C7;
  OpM17_4 = OpM16_4*C7-S5*S7;
  OpM37_4 = OpM16_4*S7+S5*C7;
  AlM17_4 = C7*(OpM26_4*s.dpt(3,1)-s.dpt(2,1)*S5)-S7*(OpM16_4*s.dpt(2,1)-OpM26_4*s.dpt(1,1));
  AlM27_4 = -(OpM16_4*s.dpt(3,1)-s.dpt(1,1)*S5);
  AlM37_4 = C7*(OpM16_4*s.dpt(2,1)-OpM26_4*s.dpt(1,1))+S7*(OpM26_4*s.dpt(3,1)-s.dpt(2,1)*S5);
  OpM17_5 = S6*C7;
  OpM37_5 = S6*S7;
  AlM17_5 = s.dpt(3,1)*C6*C7+S7*(s.dpt(1,1)*C6-s.dpt(2,1)*S6);
  AlM27_5 = -s.dpt(3,1)*S6;
  AlM37_5 = s.dpt(3,1)*C6*S7-C7*(s.dpt(1,1)*C6-s.dpt(2,1)*S6);
  OM18 = OM17*C8-OM37*S8;
  OM28 = qd(8)+OM27;
  OM38 = OM17*S8+OM37*C8;
  OpF18 = C8*(OpF17-qd(8)*OM37)-S8*(OpF37+qd(8)*OM17);
  OpF38 = C8*(OpF37+qd(8)*OM17)+S8*(OpF17-qd(8)*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  OpM18_4 = OpM17_4*C8-OpM37_4*S8;
  OpM38_4 = OpM17_4*S8+OpM37_4*C8;
  OpM18_5 = S6*C7p8;
  OpM38_5 = S6*S7p8;

% = = Block_0_1_0_1_0_3 = = 
 
% Trigonometric Variables  

  C10p9 = C10*C9-S10*S9;
  S10p9 = C10*S9+S10*C9;
 
% Forward Kinematics 

  OM19 = OM16*C9-OM36*S9;
  OM29 = qd(9)+OM26;
  OM39 = OM16*S9+OM36*C9;
  OpF19 = C9*(OpF16-qd(9)*OM36)-S9*(OpF35+qd(9)*OM16);
  OpF39 = C9*(OpF35+qd(9)*OM16)+S9*(OpF16-qd(9)*OM36);
  BS19 = -(OM29*OM29+OM39*OM39);
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS59 = -(OM19*OM19+OM39*OM39);
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BeF29 = BS29-OpF39;
  BeF39 = BS39+OpF26;
  BeF49 = BS29+OpF39;
  BeF69 = BS69-OpF19;
  BeF79 = BS39-OpF26;
  BeF89 = BS69+OpF19;
  AlF19 = C9*(AlF16+BS16*s.dpt(1,2)+BeF26*s.dpt(2,2)+BeF36*s.dpt(3,2))-S9*(AlF35+BS96*s.dpt(3,2)+BeF76*s.dpt(1,2)+BeF86*s.dpt(2,2));
  AlF29 = AlF26+BS56*s.dpt(2,2)+BeF46*s.dpt(1,2)+BeF66*s.dpt(3,2);
  AlF39 = C9*(AlF35+BS96*s.dpt(3,2)+BeF76*s.dpt(1,2)+BeF86*s.dpt(2,2))+S9*(AlF16+BS16*s.dpt(1,2)+BeF26*s.dpt(2,2)+BeF36*s.dpt(3,2));
  AlM19_1 = AlM16_1*C9-S5*S9;
  AlM39_1 = AlM16_1*S9+S5*C9;
  AlM19_2 = AlM16_2*C9-AlM35_2*S9;
  AlM39_2 = AlM16_2*S9+AlM35_2*C9;
  AlM19_3 = AlM16_3*C9-AlM35_3*S9;
  AlM39_3 = AlM16_3*S9+AlM35_3*C9;
  OpM19_4 = OpM16_4*C9-S5*S9;
  OpM39_4 = OpM16_4*S9+S5*C9;
  AlM19_4 = C9*(OpM26_4*s.dpt(3,2)-s.dpt(2,2)*S5)-S9*(OpM16_4*s.dpt(2,2)-OpM26_4*s.dpt(1,2));
  AlM29_4 = -(OpM16_4*s.dpt(3,2)-s.dpt(1,2)*S5);
  AlM39_4 = C9*(OpM16_4*s.dpt(2,2)-OpM26_4*s.dpt(1,2))+S9*(OpM26_4*s.dpt(3,2)-s.dpt(2,2)*S5);
  OpM19_5 = S6*C9;
  OpM39_5 = S6*S9;
  AlM19_5 = s.dpt(3,2)*C6*C9+S9*(s.dpt(1,2)*C6-s.dpt(2,2)*S6);
  AlM29_5 = -s.dpt(3,2)*S6;
  AlM39_5 = s.dpt(3,2)*C6*S9-C9*(s.dpt(1,2)*C6-s.dpt(2,2)*S6);
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd(10)+OM29;
  OM310 = OM19*S10+OM39*C10;
  OpF110 = C10*(OpF19-qd(10)*OM39)-S10*(OpF39+qd(10)*OM19);
  OpF310 = C10*(OpF39+qd(10)*OM19)+S10*(OpF19-qd(10)*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  OpM110_4 = OpM19_4*C10-OpM39_4*S10;
  OpM310_4 = OpM19_4*S10+OpM39_4*C10;
  OpM110_5 = S6*C10p9;
  OpM310_5 = S6*S10p9;

% = = Block_0_1_0_1_0_4 = = 
 
% Trigonometric Variables  

  C11p12 = C11*C12-S11*S12;
  S11p12 = C11*S12+S11*C12;
 
% Forward Kinematics 

  OM111 = OM16*C11-OM36*S11;
  OM211 = qd(11)+OM26;
  OM311 = OM16*S11+OM36*C11;
  OpF111 = C11*(OpF16-qd(11)*OM36)-S11*(OpF35+qd(11)*OM16);
  OpF311 = C11*(OpF35+qd(11)*OM16)+S11*(OpF16-qd(11)*OM36);
  BS111 = -(OM211*OM211+OM311*OM311);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS511 = -(OM111*OM111+OM311*OM311);
  BS611 = OM211*OM311;
  BS911 = -(OM111*OM111+OM211*OM211);
  BeF211 = BS211-OpF311;
  BeF311 = BS311+OpF26;
  BeF411 = BS211+OpF311;
  BeF611 = BS611-OpF111;
  BeF711 = BS311-OpF26;
  BeF811 = BS611+OpF111;
  AlF111 = C11*(AlF16+BS16*s.dpt(1,3)+BeF26*s.dpt(2,3)+BeF36*s.dpt(3,3))-S11*(AlF35+BS96*s.dpt(3,3)+BeF76*s.dpt(1,3)+BeF86*s.dpt(2,3));
  AlF211 = AlF26+BS56*s.dpt(2,3)+BeF46*s.dpt(1,3)+BeF66*s.dpt(3,3);
  AlF311 = C11*(AlF35+BS96*s.dpt(3,3)+BeF76*s.dpt(1,3)+BeF86*s.dpt(2,3))+S11*(AlF16+BS16*s.dpt(1,3)+BeF26*s.dpt(2,3)+BeF36*s.dpt(3,3));
  AlM111_1 = AlM16_1*C11-S11*S5;
  AlM311_1 = AlM16_1*S11+C11*S5;
  AlM111_2 = AlM16_2*C11-AlM35_2*S11;
  AlM311_2 = AlM16_2*S11+AlM35_2*C11;
  AlM111_3 = AlM16_3*C11-AlM35_3*S11;
  AlM311_3 = AlM16_3*S11+AlM35_3*C11;
  OpM111_4 = OpM16_4*C11-S11*S5;
  OpM311_4 = OpM16_4*S11+C11*S5;
  AlM111_4 = C11*(OpM26_4*s.dpt(3,3)-s.dpt(2,3)*S5)-S11*(OpM16_4*s.dpt(2,3)-OpM26_4*s.dpt(1,3));
  AlM211_4 = -(OpM16_4*s.dpt(3,3)-s.dpt(1,3)*S5);
  AlM311_4 = C11*(OpM16_4*s.dpt(2,3)-OpM26_4*s.dpt(1,3))+S11*(OpM26_4*s.dpt(3,3)-s.dpt(2,3)*S5);
  OpM111_5 = C11*S6;
  OpM311_5 = S11*S6;
  AlM111_5 = s.dpt(3,3)*C11*C6+S11*(s.dpt(1,3)*C6-s.dpt(2,3)*S6);
  AlM211_5 = -s.dpt(3,3)*S6;
  AlM311_5 = s.dpt(3,3)*S11*C6-C11*(s.dpt(1,3)*C6-s.dpt(2,3)*S6);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd(12)+OM211;
  OM312 = OM111*S12+OM311*C12;
  OpF112 = C12*(OpF111-qd(12)*OM311)-S12*(OpF311+qd(12)*OM111);
  OpF312 = C12*(OpF311+qd(12)*OM111)+S12*(OpF111-qd(12)*OM311);
  BS212 = OM112*OM212;
  BS312 = OM112*OM312;
  BS612 = OM212*OM312;
  OpM112_4 = OpM111_4*C12-OpM311_4*S12;
  OpM312_4 = OpM111_4*S12+OpM311_4*C12;
  OpM112_5 = S6*C11p12;
  OpM312_5 = S6*S11p12;

% = = Block_0_1_0_1_0_5 = = 
 
% Trigonometric Variables  

  C13p14 = C13*C14-S13*S14;
  S13p14 = C13*S14+S13*C14;
 
% Forward Kinematics 

  OM113 = OM16*C13-OM36*S13;
  OM213 = qd(13)+OM26;
  OM313 = OM16*S13+OM36*C13;
  OpF113 = C13*(OpF16-qd(13)*OM36)-S13*(OpF35+qd(13)*OM16);
  OpF313 = C13*(OpF35+qd(13)*OM16)+S13*(OpF16-qd(13)*OM36);
  BS113 = -(OM213*OM213+OM313*OM313);
  BS213 = OM113*OM213;
  BS313 = OM113*OM313;
  BS513 = -(OM113*OM113+OM313*OM313);
  BS613 = OM213*OM313;
  BS913 = -(OM113*OM113+OM213*OM213);
  BeF213 = BS213-OpF313;
  BeF313 = BS313+OpF26;
  BeF413 = BS213+OpF313;
  BeF613 = BS613-OpF113;
  BeF713 = BS313-OpF26;
  BeF813 = BS613+OpF113;
  AlF113 = C13*(AlF16+BS16*s.dpt(1,4)+BeF26*s.dpt(2,4)+BeF36*s.dpt(3,4))-S13*(AlF35+BS96*s.dpt(3,4)+BeF76*s.dpt(1,4)+BeF86*s.dpt(2,4));
  AlF213 = AlF26+BS56*s.dpt(2,4)+BeF46*s.dpt(1,4)+BeF66*s.dpt(3,4);
  AlF313 = C13*(AlF35+BS96*s.dpt(3,4)+BeF76*s.dpt(1,4)+BeF86*s.dpt(2,4))+S13*(AlF16+BS16*s.dpt(1,4)+BeF26*s.dpt(2,4)+BeF36*s.dpt(3,4));
  AlM113_1 = AlM16_1*C13-S13*S5;
  AlM313_1 = AlM16_1*S13+C13*S5;
  AlM113_2 = AlM16_2*C13-AlM35_2*S13;
  AlM313_2 = AlM16_2*S13+AlM35_2*C13;
  AlM113_3 = AlM16_3*C13-AlM35_3*S13;
  AlM313_3 = AlM16_3*S13+AlM35_3*C13;
  OpM113_4 = OpM16_4*C13-S13*S5;
  OpM313_4 = OpM16_4*S13+C13*S5;
  AlM113_4 = C13*(OpM26_4*s.dpt(3,4)-s.dpt(2,4)*S5)-S13*(OpM16_4*s.dpt(2,4)-OpM26_4*s.dpt(1,4));
  AlM213_4 = -(OpM16_4*s.dpt(3,4)-s.dpt(1,4)*S5);
  AlM313_4 = C13*(OpM16_4*s.dpt(2,4)-OpM26_4*s.dpt(1,4))+S13*(OpM26_4*s.dpt(3,4)-s.dpt(2,4)*S5);
  OpM113_5 = C13*S6;
  OpM313_5 = S13*S6;
  AlM113_5 = s.dpt(3,4)*C13*C6+S13*(s.dpt(1,4)*C6-s.dpt(2,4)*S6);
  AlM213_5 = -s.dpt(3,4)*S6;
  AlM313_5 = s.dpt(3,4)*S13*C6-C13*(s.dpt(1,4)*C6-s.dpt(2,4)*S6);
  OM114 = OM113*C14-OM313*S14;
  OM214 = qd(14)+OM213;
  OM314 = OM113*S14+OM313*C14;
  OpF114 = C14*(OpF113-qd(14)*OM313)-S14*(OpF313+qd(14)*OM113);
  OpF314 = C14*(OpF313+qd(14)*OM113)+S14*(OpF113-qd(14)*OM313);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS614 = OM214*OM314;
  OpM114_4 = OpM113_4*C14-OpM313_4*S14;
  OpM314_4 = OpM113_4*S14+OpM313_4*C14;
  OpM114_5 = S6*C13p14;
  OpM314_5 = S6*S13p14;

% = = Block_0_2_0_1_0_2 = = 
 
% Backward Dynamics 

  FA18 = -(s.frc(1,8)+s.m(8)*(s.l(1,8)*(OM28*OM28+OM38*OM38)-s.l(2,8)*(BS28-OpF38)-s.l(3,8)*(BS38+OpF26)-C8*(AlF17+s.dpt(2,5)*BeF27+s.dpt(3,5)*...
 BeF37+BS17*s.dpt(1,5))+S8*(AlF37+s.dpt(2,5)*BeF87+s.dpt(3,5)*BS97+BeF77*s.dpt(1,5))));
  FA28 = -(s.frc(2,8)-s.m(8)*(AlF27+s.dpt(2,5)*BS57+s.dpt(3,5)*BeF67+s.l(1,8)*(BS28+OpF38)-s.l(2,8)*(OM18*OM18+OM38*OM38)+BeF47*s.dpt(1,5)+...
 s.l(3,8)*(BS68-OpF18)));
  FA38 = -(s.frc(3,8)-s.m(8)*(s.l(1,8)*(BS38-OpF26)+s.l(2,8)*(BS68+OpF18)-s.l(3,8)*(OM18*OM18+OM28*OM28)+C8*(AlF37+s.dpt(2,5)*BeF87+s.dpt(3,5)*...
 BS97+BeF77*s.dpt(1,5))+S8*(AlF17+s.dpt(2,5)*BeF27+s.dpt(3,5)*BeF37+BS17*s.dpt(1,5))));
  CF18 = -(s.trq(1,8)-s.In(1,8)*OpF18-s.In(2,8)*OpF26-s.In(3,8)*OpF38-s.l(2,8)*FA38+FA28*s.l(3,8)+OM38*(s.In(5,8)*OM28+s.In(6,8)*OM38-s.In(9,8)*...
 OM28));
  CF28 = -(s.trq(2,8)-s.In(5,8)*OpF26-s.In(6,8)*OpF38+s.l(1,8)*FA38-FA18*s.l(3,8)-OM38*(s.In(1,8)*OM18+s.In(2,8)*OM28+s.In(3,8)*OM38-s.In(9,8)*...
 OM18));
  CF38 = -(s.trq(3,8)-s.In(9,8)*OpF38-s.l(1,8)*FA28+s.l(2,8)*FA18-OM18*(s.In(5,8)*OM28+s.In(6,8)*OM38)+OM28*(s.In(1,8)*OM18+s.In(2,8)*OM28+...
 s.In(3,8)*OM38));
  FB18_1 = s.m(8)*(AlM17_1*C8-AlM37_1*S8);
  FB28_1 = s.m(8)*AlM26_1;
  FB38_1 = s.m(8)*(AlM17_1*S8+AlM37_1*C8);
  CM18_1 = s.l(2,8)*FB38_1-FB28_1*s.l(3,8);
  CM28_1 = -(s.l(1,8)*FB38_1-FB18_1*s.l(3,8));
  CM38_1 = s.l(1,8)*FB28_1-s.l(2,8)*FB18_1;
  FB18_2 = s.m(8)*(AlM17_2*C8-AlM37_2*S8);
  FB28_2 = s.m(8)*AlM26_2;
  FB38_2 = s.m(8)*(AlM17_2*S8+AlM37_2*C8);
  CM18_2 = s.l(2,8)*FB38_2-FB28_2*s.l(3,8);
  CM28_2 = -(s.l(1,8)*FB38_2-FB18_2*s.l(3,8));
  CM38_2 = s.l(1,8)*FB28_2-s.l(2,8)*FB18_2;
  FB18_3 = s.m(8)*(AlM17_3*C8-AlM37_3*S8);
  FB28_3 = s.m(8)*AlM26_3;
  FB38_3 = s.m(8)*(AlM17_3*S8+AlM37_3*C8);
  CM18_3 = s.l(2,8)*FB38_3-FB28_3*s.l(3,8);
  CM28_3 = -(s.l(1,8)*FB38_3-FB18_3*s.l(3,8));
  CM38_3 = s.l(1,8)*FB28_3-s.l(2,8)*FB18_3;
  FB18_4 = s.m(8)*(C8*(AlM17_4-s.dpt(2,5)*OpM37_4+s.dpt(3,5)*OpM26_4)-S8*(AlM37_4+s.dpt(2,5)*OpM17_4-OpM26_4*s.dpt(1,5))-s.l(2,8)*OpM38_4+OpM26_4...
 *s.l(3,8));
  FB28_4 = s.m(8)*(AlM27_4-s.dpt(3,5)*OpM17_4+s.l(1,8)*OpM38_4-OpM18_4*s.l(3,8)+OpM37_4*s.dpt(1,5));
  FB38_4 = s.m(8)*(C8*(AlM37_4+s.dpt(2,5)*OpM17_4-OpM26_4*s.dpt(1,5))+S8*(AlM17_4-s.dpt(2,5)*OpM37_4+s.dpt(3,5)*OpM26_4)-s.l(1,8)*OpM26_4+...
 s.l(2,8)*OpM18_4);
  CM18_4 = s.In(1,8)*OpM18_4+s.In(2,8)*OpM26_4+s.In(3,8)*OpM38_4+s.l(2,8)*FB38_4-FB28_4*s.l(3,8);
  CM28_4 = s.In(5,8)*OpM26_4+s.In(6,8)*OpM38_4-s.l(1,8)*FB38_4+FB18_4*s.l(3,8);
  CM38_4 = s.In(9,8)*OpM38_4+s.l(1,8)*FB28_4-s.l(2,8)*FB18_4;
  FB18_5 = s.m(8)*(C8*(AlM17_5-s.dpt(2,5)*OpM37_5+s.dpt(3,5)*C6)-S8*(AlM37_5+s.dpt(2,5)*OpM17_5-s.dpt(1,5)*C6)-s.l(2,8)*OpM38_5+s.l(3,8)*C6);
  FB28_5 = s.m(8)*(AlM27_5-s.dpt(3,5)*OpM17_5+s.l(1,8)*OpM38_5-OpM18_5*s.l(3,8)+OpM37_5*s.dpt(1,5));
  FB38_5 = s.m(8)*(C8*(AlM37_5+s.dpt(2,5)*OpM17_5-s.dpt(1,5)*C6)+S8*(AlM17_5-s.dpt(2,5)*OpM37_5+s.dpt(3,5)*C6)-s.l(1,8)*C6+s.l(2,8)*OpM18_5);
  CM18_5 = s.In(1,8)*OpM18_5+s.In(2,8)*C6+s.In(3,8)*OpM38_5+s.l(2,8)*FB38_5-FB28_5*s.l(3,8);
  CM28_5 = s.In(5,8)*C6+s.In(6,8)*OpM38_5-s.l(1,8)*FB38_5+FB18_5*s.l(3,8);
  CM38_5 = s.In(9,8)*OpM38_5+s.l(1,8)*FB28_5-s.l(2,8)*FB18_5;
  FB18_6 = -s.m(8)*C7p8*(s.dpt(2,5)+s.l(2,8)+s.dpt(2,1));
  FB28_6 = s.m(8)*(s.dpt(1,1)+s.dpt(3,5)*S7+s.l(1,8)*C7p8+s.dpt(1,5)*C7+s.l(3,8)*S7p8);
  FB38_6 = -s.m(8)*S7p8*(s.dpt(2,5)+s.l(2,8)+s.dpt(2,1));
  CM18_6 = -(s.In(1,8)*S7p8-s.In(3,8)*C7p8-s.l(2,8)*FB38_6+FB28_6*s.l(3,8));
  CM28_6 = s.In(6,8)*C7p8-s.l(1,8)*FB38_6+FB18_6*s.l(3,8);
  CM38_6 = s.In(9,8)*C7p8+s.l(1,8)*FB28_6-s.l(2,8)*FB18_6;
  FB18_7 = s.m(8)*(s.l(3,8)+s.dpt(3,5)*C8+s.dpt(1,5)*S8);
  FB38_7 = -s.m(8)*(s.l(1,8)-s.dpt(3,5)*S8+s.dpt(1,5)*C8);
  CM28_7 = s.In(5,8)-s.l(1,8)*FB38_7+FB18_7*s.l(3,8);
  CM28_8 = s.In(5,8)+s.l(1,8)*s.l(1,8)*s.m(8)+s.m(8)*s.l(3,8)*s.l(3,8);
  FA17 = -(s.frc(1,7)-s.m(7)*(AlF17+s.l(1,7)*BS17+s.l(2,7)*BeF27+s.l(3,7)*BeF37));
  FA27 = -(s.frc(2,7)-s.m(7)*(AlF27+s.l(1,7)*BeF47+s.l(2,7)*BS57+s.l(3,7)*BeF67));
  FA37 = -(s.frc(3,7)-s.m(7)*(AlF37+s.l(1,7)*BeF77+s.l(2,7)*BeF87+s.l(3,7)*BS97));
  FF17 = FA17+FA18*C8+FA38*S8;
  FF27 = FA27+FA28;
  FF37 = FA37-FA18*S8+FA38*C8;
  CF17 = -(s.trq(1,7)-s.In(1,7)*OpF17-s.In(2,7)*OpF26-s.In(3,7)*OpF37+s.dpt(2,5)*(FA18*S8-FA38*C8)+s.dpt(3,5)*FA28-s.l(2,7)*FA37+s.l(3,7)*FA27-...
 CF18*C8-CF38*S8+OM37*(s.In(5,7)*OM27+s.In(6,7)*OM37-s.In(9,7)*OM27));
  CF27 = -(s.trq(2,7)-CF28-s.In(5,7)*OpF26-s.In(6,7)*OpF37-s.dpt(3,5)*(FA18*C8+FA38*S8)+s.l(1,7)*FA37-s.l(3,7)*FA17-OM37*(s.In(1,7)*OM17+...
 s.In(2,7)*OM27+s.In(3,7)*OM37-s.In(9,7)*OM17)-s.dpt(1,5)*(FA18*S8-FA38*C8));
  CF37 = -(s.trq(3,7)-s.In(9,7)*OpF37+s.dpt(2,5)*(FA18*C8+FA38*S8)-s.l(1,7)*FA27+s.l(2,7)*FA17+CF18*S8-CF38*C8-FA28*s.dpt(1,5)-OM17*(s.In(5,7)*...
 OM27+s.In(6,7)*OM37)+OM27*(s.In(1,7)*OM17+s.In(2,7)*OM27+s.In(3,7)*OM37));
  FB17_1 = s.m(7)*AlM17_1;
  FB27_1 = s.m(7)*AlM26_1;
  FB37_1 = s.m(7)*AlM37_1;
  FM17_1 = FB17_1+FB18_1*C8+FB38_1*S8;
  FM27_1 = FB27_1+FB28_1;
  FM37_1 = FB37_1-FB18_1*S8+FB38_1*C8;
  CM17_1 = -(s.dpt(2,5)*(FB18_1*S8-FB38_1*C8)+s.dpt(3,5)*FB28_1-s.l(2,7)*FB37_1+s.l(3,7)*FB27_1-CM18_1*C8-CM38_1*S8);
  CM27_1 = CM28_1+s.dpt(3,5)*(FB18_1*C8+FB38_1*S8)-s.l(1,7)*FB37_1+s.l(3,7)*FB17_1+s.dpt(1,5)*(FB18_1*S8-FB38_1*C8);
  CM37_1 = -(s.dpt(2,5)*(FB18_1*C8+FB38_1*S8)-s.l(1,7)*FB27_1+s.l(2,7)*FB17_1+CM18_1*S8-CM38_1*C8-FB28_1*s.dpt(1,5));
  FB17_2 = s.m(7)*AlM17_2;
  FB27_2 = s.m(7)*AlM26_2;
  FB37_2 = s.m(7)*AlM37_2;
  FM17_2 = FB17_2+FB18_2*C8+FB38_2*S8;
  FM27_2 = FB27_2+FB28_2;
  FM37_2 = FB37_2-FB18_2*S8+FB38_2*C8;
  CM17_2 = -(s.dpt(2,5)*(FB18_2*S8-FB38_2*C8)+s.dpt(3,5)*FB28_2-s.l(2,7)*FB37_2+s.l(3,7)*FB27_2-CM18_2*C8-CM38_2*S8);
  CM27_2 = CM28_2+s.dpt(3,5)*(FB18_2*C8+FB38_2*S8)-s.l(1,7)*FB37_2+s.l(3,7)*FB17_2+s.dpt(1,5)*(FB18_2*S8-FB38_2*C8);
  CM37_2 = -(s.dpt(2,5)*(FB18_2*C8+FB38_2*S8)-s.l(1,7)*FB27_2+s.l(2,7)*FB17_2+CM18_2*S8-CM38_2*C8-FB28_2*s.dpt(1,5));
  FB17_3 = s.m(7)*AlM17_3;
  FB27_3 = s.m(7)*AlM26_3;
  FB37_3 = s.m(7)*AlM37_3;
  FM17_3 = FB17_3+FB18_3*C8+FB38_3*S8;
  FM27_3 = FB27_3+FB28_3;
  FM37_3 = FB37_3-FB18_3*S8+FB38_3*C8;
  CM17_3 = -(s.dpt(2,5)*(FB18_3*S8-FB38_3*C8)+s.dpt(3,5)*FB28_3-s.l(2,7)*FB37_3+s.l(3,7)*FB27_3-CM18_3*C8-CM38_3*S8);
  CM27_3 = CM28_3+s.dpt(3,5)*(FB18_3*C8+FB38_3*S8)-s.l(1,7)*FB37_3+s.l(3,7)*FB17_3+s.dpt(1,5)*(FB18_3*S8-FB38_3*C8);
  CM37_3 = -(s.dpt(2,5)*(FB18_3*C8+FB38_3*S8)-s.l(1,7)*FB27_3+s.l(2,7)*FB17_3+CM18_3*S8-CM38_3*C8-FB28_3*s.dpt(1,5));
  FB17_4 = s.m(7)*(AlM17_4-s.l(2,7)*OpM37_4+s.l(3,7)*OpM26_4);
  FB27_4 = s.m(7)*(AlM27_4+s.l(1,7)*OpM37_4-s.l(3,7)*OpM17_4);
  FB37_4 = s.m(7)*(AlM37_4-s.l(1,7)*OpM26_4+s.l(2,7)*OpM17_4);
  FM17_4 = FB17_4+FB18_4*C8+FB38_4*S8;
  FM27_4 = FB27_4+FB28_4;
  FM37_4 = FB37_4-FB18_4*S8+FB38_4*C8;
  CM17_4 = s.In(1,7)*OpM17_4+s.In(2,7)*OpM26_4+s.In(3,7)*OpM37_4-s.dpt(2,5)*(FB18_4*S8-FB38_4*C8)-s.dpt(3,5)*FB28_4+s.l(2,7)*FB37_4-s.l(3,7)*...
 FB27_4+CM18_4*C8+CM38_4*S8;
  CM27_4 = CM28_4+s.In(5,7)*OpM26_4+s.In(6,7)*OpM37_4+s.dpt(3,5)*(FB18_4*C8+FB38_4*S8)-s.l(1,7)*FB37_4+s.l(3,7)*FB17_4+s.dpt(1,5)*(FB18_4*S8-...
 FB38_4*C8);
  CM37_4 = s.In(9,7)*OpM37_4-s.dpt(2,5)*(FB18_4*C8+FB38_4*S8)+s.l(1,7)*FB27_4-s.l(2,7)*FB17_4-CM18_4*S8+CM38_4*C8+FB28_4*s.dpt(1,5);
  FB17_5 = s.m(7)*(AlM17_5-s.l(2,7)*OpM37_5+s.l(3,7)*C6);
  FB27_5 = s.m(7)*(AlM27_5+s.l(1,7)*OpM37_5-s.l(3,7)*OpM17_5);
  FB37_5 = s.m(7)*(AlM37_5-s.l(1,7)*C6+s.l(2,7)*OpM17_5);
  FM17_5 = FB17_5+FB18_5*C8+FB38_5*S8;
  FM27_5 = FB27_5+FB28_5;
  FM37_5 = FB37_5-FB18_5*S8+FB38_5*C8;
  CM17_5 = s.In(1,7)*OpM17_5+s.In(2,7)*C6+s.In(3,7)*OpM37_5-s.dpt(2,5)*(FB18_5*S8-FB38_5*C8)-s.dpt(3,5)*FB28_5+s.l(2,7)*FB37_5-s.l(3,7)*FB27_5+...
 CM18_5*C8+CM38_5*S8;
  CM27_5 = CM28_5+s.In(5,7)*C6+s.In(6,7)*OpM37_5+s.dpt(3,5)*(FB18_5*C8+FB38_5*S8)-s.l(1,7)*FB37_5+s.l(3,7)*FB17_5+s.dpt(1,5)*(FB18_5*S8-FB38_5*C8...
 );
  CM37_5 = s.In(9,7)*OpM37_5-s.dpt(2,5)*(FB18_5*C8+FB38_5*S8)+s.l(1,7)*FB27_5-s.l(2,7)*FB17_5-CM18_5*S8+CM38_5*C8+FB28_5*s.dpt(1,5);
  FB17_6 = -s.m(7)*C7*(s.l(2,7)+s.dpt(2,1));
  FB27_6 = s.m(7)*(s.dpt(1,1)+s.l(1,7)*C7+s.l(3,7)*S7);
  FB37_6 = -s.m(7)*S7*(s.l(2,7)+s.dpt(2,1));
  CM27_6 = CM28_6+s.In(6,7)*C7+s.dpt(3,5)*(FB18_6*C8+FB38_6*S8)-s.l(1,7)*FB37_6+s.l(3,7)*FB17_6+s.dpt(1,5)*(FB18_6*S8-FB38_6*C8);
  CM27_7 = s.In(5,7)+CM28_7+s.dpt(3,5)*(FB18_7*C8+FB38_7*S8)+s.l(1,7)*s.l(1,7)*s.m(7)+s.l(3,7)*s.l(3,7)*s.m(7)+s.dpt(1,5)*(FB18_7*S8-FB38_7*C8);

% = = Block_0_2_0_1_0_3 = = 
 
% Backward Dynamics 

  FA110 = -(s.frc(1,10)+s.m(10)*(s.l(1,10)*(OM210*OM210+OM310*OM310)-s.l(2,10)*(BS210-OpF310)-s.l(3,10)*(BS310+OpF26)-C10*(AlF19+s.dpt(2,7)*BeF29...
 +s.dpt(3,7)*BeF39+BS19*s.dpt(1,7))+S10*(AlF39+s.dpt(2,7)*BeF89+s.dpt(3,7)*BS99+BeF79*s.dpt(1,7))));
  FA210 = -(s.frc(2,10)-s.m(10)*(AlF29+s.dpt(2,7)*BS59+s.dpt(3,7)*BeF69+s.l(1,10)*(BS210+OpF310)-s.l(2,10)*(OM110*OM110+OM310*OM310)+BeF49*...
 s.dpt(1,7)+s.l(3,10)*(BS610-OpF110)));
  FA310 = -(s.frc(3,10)-s.m(10)*(s.l(1,10)*(BS310-OpF26)+s.l(2,10)*(BS610+OpF110)-s.l(3,10)*(OM110*OM110+OM210*OM210)+C10*(AlF39+s.dpt(2,7)*BeF89...
 +s.dpt(3,7)*BS99+BeF79*s.dpt(1,7))+S10*(AlF19+s.dpt(2,7)*BeF29+s.dpt(3,7)*BeF39+BS19*s.dpt(1,7))));
  CF110 = -(s.trq(1,10)-s.In(1,10)*OpF110-s.In(2,10)*OpF26-s.In(3,10)*OpF310-s.l(2,10)*FA310+FA210*s.l(3,10)+OM310*(s.In(5,10)*OM210+s.In(6,10)*...
 OM310-s.In(9,10)*OM210));
  CF210 = -(s.trq(2,10)-s.In(5,10)*OpF26-s.In(6,10)*OpF310+s.l(1,10)*FA310-FA110*s.l(3,10)-OM310*(s.In(1,10)*OM110+s.In(2,10)*OM210+s.In(3,10)*...
 OM310-s.In(9,10)*OM110));
  CF310 = -(s.trq(3,10)-s.In(9,10)*OpF310-s.l(1,10)*FA210+s.l(2,10)*FA110-OM110*(s.In(5,10)*OM210+s.In(6,10)*OM310)+OM210*(s.In(1,10)*OM110+...
 s.In(2,10)*OM210+s.In(3,10)*OM310));
  FB110_1 = s.m(10)*(AlM19_1*C10-AlM39_1*S10);
  FB210_1 = s.m(10)*AlM26_1;
  FB310_1 = s.m(10)*(AlM19_1*S10+AlM39_1*C10);
  CM110_1 = s.l(2,10)*FB310_1-FB210_1*s.l(3,10);
  CM210_1 = -(s.l(1,10)*FB310_1-FB110_1*s.l(3,10));
  CM310_1 = s.l(1,10)*FB210_1-s.l(2,10)*FB110_1;
  FB110_2 = s.m(10)*(AlM19_2*C10-AlM39_2*S10);
  FB210_2 = s.m(10)*AlM26_2;
  FB310_2 = s.m(10)*(AlM19_2*S10+AlM39_2*C10);
  CM110_2 = s.l(2,10)*FB310_2-FB210_2*s.l(3,10);
  CM210_2 = -(s.l(1,10)*FB310_2-FB110_2*s.l(3,10));
  CM310_2 = s.l(1,10)*FB210_2-s.l(2,10)*FB110_2;
  FB110_3 = s.m(10)*(AlM19_3*C10-AlM39_3*S10);
  FB210_3 = s.m(10)*AlM26_3;
  FB310_3 = s.m(10)*(AlM19_3*S10+AlM39_3*C10);
  CM110_3 = s.l(2,10)*FB310_3-FB210_3*s.l(3,10);
  CM210_3 = -(s.l(1,10)*FB310_3-FB110_3*s.l(3,10));
  CM310_3 = s.l(1,10)*FB210_3-s.l(2,10)*FB110_3;
  FB110_4 = s.m(10)*(C10*(AlM19_4-s.dpt(2,7)*OpM39_4+s.dpt(3,7)*OpM26_4)-S10*(AlM39_4+s.dpt(2,7)*OpM19_4-OpM26_4*s.dpt(1,7))-s.l(2,10)*OpM310_4+...
 OpM26_4*s.l(3,10));
  FB210_4 = s.m(10)*(AlM29_4-s.dpt(3,7)*OpM19_4+s.l(1,10)*OpM310_4-OpM110_4*s.l(3,10)+OpM39_4*s.dpt(1,7));
  FB310_4 = s.m(10)*(C10*(AlM39_4+s.dpt(2,7)*OpM19_4-OpM26_4*s.dpt(1,7))+S10*(AlM19_4-s.dpt(2,7)*OpM39_4+s.dpt(3,7)*OpM26_4)-s.l(1,10)*OpM26_4+...
 s.l(2,10)*OpM110_4);
  CM110_4 = s.In(1,10)*OpM110_4+s.In(2,10)*OpM26_4+s.In(3,10)*OpM310_4+s.l(2,10)*FB310_4-FB210_4*s.l(3,10);
  CM210_4 = s.In(5,10)*OpM26_4+s.In(6,10)*OpM310_4-s.l(1,10)*FB310_4+FB110_4*s.l(3,10);
  CM310_4 = s.In(9,10)*OpM310_4+s.l(1,10)*FB210_4-s.l(2,10)*FB110_4;
  FB110_5 = s.m(10)*(C10*(AlM19_5-s.dpt(2,7)*OpM39_5+s.dpt(3,7)*C6)-S10*(AlM39_5+s.dpt(2,7)*OpM19_5-s.dpt(1,7)*C6)-s.l(2,10)*OpM310_5+s.l(3,10)*...
 C6);
  FB210_5 = s.m(10)*(AlM29_5-s.dpt(3,7)*OpM19_5+s.l(1,10)*OpM310_5-OpM110_5*s.l(3,10)+OpM39_5*s.dpt(1,7));
  FB310_5 = s.m(10)*(C10*(AlM39_5+s.dpt(2,7)*OpM19_5-s.dpt(1,7)*C6)+S10*(AlM19_5-s.dpt(2,7)*OpM39_5+s.dpt(3,7)*C6)-s.l(1,10)*C6+s.l(2,10)*...
 OpM110_5);
  CM110_5 = s.In(1,10)*OpM110_5+s.In(2,10)*C6+s.In(3,10)*OpM310_5+s.l(2,10)*FB310_5-FB210_5*s.l(3,10);
  CM210_5 = s.In(5,10)*C6+s.In(6,10)*OpM310_5-s.l(1,10)*FB310_5+FB110_5*s.l(3,10);
  CM310_5 = s.In(9,10)*OpM310_5+s.l(1,10)*FB210_5-s.l(2,10)*FB110_5;
  FB110_6 = -s.m(10)*C10p9*(s.dpt(2,7)+s.l(2,10)+s.dpt(2,2));
  FB210_6 = s.m(10)*(s.dpt(1,2)+s.dpt(3,7)*S9+s.l(1,10)*C10p9+s.dpt(1,7)*C9+s.l(3,10)*S10p9);
  FB310_6 = -s.m(10)*S10p9*(s.dpt(2,7)+s.l(2,10)+s.dpt(2,2));
  CM110_6 = -(s.In(1,10)*S10p9-s.In(3,10)*C10p9-s.l(2,10)*FB310_6+FB210_6*s.l(3,10));
  CM210_6 = s.In(6,10)*C10p9-s.l(1,10)*FB310_6+FB110_6*s.l(3,10);
  CM310_6 = s.In(9,10)*C10p9+s.l(1,10)*FB210_6-s.l(2,10)*FB110_6;
  FB110_9 = s.m(10)*(s.l(3,10)+s.dpt(3,7)*C10+s.dpt(1,7)*S10);
  FB310_9 = -s.m(10)*(s.l(1,10)-s.dpt(3,7)*S10+s.dpt(1,7)*C10);
  CM210_9 = s.In(5,10)-s.l(1,10)*FB310_9+FB110_9*s.l(3,10);
  CM210_10 = s.In(5,10)+s.l(1,10)*s.l(1,10)*s.m(10)+s.m(10)*s.l(3,10)*s.l(3,10);
  FA19 = -(s.frc(1,9)-s.m(9)*(AlF19+s.l(1,9)*BS19+s.l(2,9)*BeF29+s.l(3,9)*BeF39));
  FA29 = -(s.frc(2,9)-s.m(9)*(AlF29+s.l(1,9)*BeF49+s.l(2,9)*BS59+s.l(3,9)*BeF69));
  FA39 = -(s.frc(3,9)-s.m(9)*(AlF39+s.l(1,9)*BeF79+s.l(2,9)*BeF89+s.l(3,9)*BS99));
  FF19 = FA19+FA110*C10+FA310*S10;
  FF29 = FA210+FA29;
  FF39 = FA39-FA110*S10+FA310*C10;
  CF19 = -(s.trq(1,9)-s.In(1,9)*OpF19-s.In(2,9)*OpF26-s.In(3,9)*OpF39+s.dpt(2,7)*(FA110*S10-FA310*C10)+s.dpt(3,7)*FA210-s.l(2,9)*FA39+s.l(3,9)*...
 FA29-CF110*C10-CF310*S10+OM39*(s.In(5,9)*OM29+s.In(6,9)*OM39-s.In(9,9)*OM29));
  CF29 = -(s.trq(2,9)-CF210-s.In(5,9)*OpF26-s.In(6,9)*OpF39-s.dpt(3,7)*(FA110*C10+FA310*S10)+s.l(1,9)*FA39-s.l(3,9)*FA19-OM39*(s.In(1,9)*OM19+...
 s.In(2,9)*OM29+s.In(3,9)*OM39-s.In(9,9)*OM19)-s.dpt(1,7)*(FA110*S10-FA310*C10));
  CF39 = -(s.trq(3,9)-s.In(9,9)*OpF39+s.dpt(2,7)*(FA110*C10+FA310*S10)-s.l(1,9)*FA29+s.l(2,9)*FA19+CF110*S10-CF310*C10-FA210*s.dpt(1,7)-OM19*(...
 s.In(5,9)*OM29+s.In(6,9)*OM39)+OM29*(s.In(1,9)*OM19+s.In(2,9)*OM29+s.In(3,9)*OM39));
  FB19_1 = s.m(9)*AlM19_1;
  FB29_1 = s.m(9)*AlM26_1;
  FB39_1 = s.m(9)*AlM39_1;
  FM19_1 = FB19_1+FB110_1*C10+FB310_1*S10;
  FM29_1 = FB210_1+FB29_1;
  FM39_1 = FB39_1-FB110_1*S10+FB310_1*C10;
  CM19_1 = -(s.dpt(2,7)*(FB110_1*S10-FB310_1*C10)+s.dpt(3,7)*FB210_1-s.l(2,9)*FB39_1+s.l(3,9)*FB29_1-CM110_1*C10-CM310_1*S10);
  CM29_1 = CM210_1+s.dpt(3,7)*(FB110_1*C10+FB310_1*S10)-s.l(1,9)*FB39_1+s.l(3,9)*FB19_1+s.dpt(1,7)*(FB110_1*S10-FB310_1*C10);
  CM39_1 = -(s.dpt(2,7)*(FB110_1*C10+FB310_1*S10)-s.l(1,9)*FB29_1+s.l(2,9)*FB19_1+CM110_1*S10-CM310_1*C10-FB210_1*s.dpt(1,7));
  FB19_2 = s.m(9)*AlM19_2;
  FB29_2 = s.m(9)*AlM26_2;
  FB39_2 = s.m(9)*AlM39_2;
  FM19_2 = FB19_2+FB110_2*C10+FB310_2*S10;
  FM29_2 = FB210_2+FB29_2;
  FM39_2 = FB39_2-FB110_2*S10+FB310_2*C10;
  CM19_2 = -(s.dpt(2,7)*(FB110_2*S10-FB310_2*C10)+s.dpt(3,7)*FB210_2-s.l(2,9)*FB39_2+s.l(3,9)*FB29_2-CM110_2*C10-CM310_2*S10);
  CM29_2 = CM210_2+s.dpt(3,7)*(FB110_2*C10+FB310_2*S10)-s.l(1,9)*FB39_2+s.l(3,9)*FB19_2+s.dpt(1,7)*(FB110_2*S10-FB310_2*C10);
  CM39_2 = -(s.dpt(2,7)*(FB110_2*C10+FB310_2*S10)-s.l(1,9)*FB29_2+s.l(2,9)*FB19_2+CM110_2*S10-CM310_2*C10-FB210_2*s.dpt(1,7));
  FB19_3 = s.m(9)*AlM19_3;
  FB29_3 = s.m(9)*AlM26_3;
  FB39_3 = s.m(9)*AlM39_3;
  FM19_3 = FB19_3+FB110_3*C10+FB310_3*S10;
  FM29_3 = FB210_3+FB29_3;
  FM39_3 = FB39_3-FB110_3*S10+FB310_3*C10;
  CM19_3 = -(s.dpt(2,7)*(FB110_3*S10-FB310_3*C10)+s.dpt(3,7)*FB210_3-s.l(2,9)*FB39_3+s.l(3,9)*FB29_3-CM110_3*C10-CM310_3*S10);
  CM29_3 = CM210_3+s.dpt(3,7)*(FB110_3*C10+FB310_3*S10)-s.l(1,9)*FB39_3+s.l(3,9)*FB19_3+s.dpt(1,7)*(FB110_3*S10-FB310_3*C10);
  CM39_3 = -(s.dpt(2,7)*(FB110_3*C10+FB310_3*S10)-s.l(1,9)*FB29_3+s.l(2,9)*FB19_3+CM110_3*S10-CM310_3*C10-FB210_3*s.dpt(1,7));
  FB19_4 = s.m(9)*(AlM19_4-s.l(2,9)*OpM39_4+s.l(3,9)*OpM26_4);
  FB29_4 = s.m(9)*(AlM29_4+s.l(1,9)*OpM39_4-s.l(3,9)*OpM19_4);
  FB39_4 = s.m(9)*(AlM39_4-s.l(1,9)*OpM26_4+s.l(2,9)*OpM19_4);
  FM19_4 = FB19_4+FB110_4*C10+FB310_4*S10;
  FM29_4 = FB210_4+FB29_4;
  FM39_4 = FB39_4-FB110_4*S10+FB310_4*C10;
  CM19_4 = s.In(1,9)*OpM19_4+s.In(2,9)*OpM26_4+s.In(3,9)*OpM39_4-s.dpt(2,7)*(FB110_4*S10-FB310_4*C10)-s.dpt(3,7)*FB210_4+s.l(2,9)*FB39_4-s.l(3,9)...
 *FB29_4+CM110_4*C10+CM310_4*S10;
  CM29_4 = CM210_4+s.In(5,9)*OpM26_4+s.In(6,9)*OpM39_4+s.dpt(3,7)*(FB110_4*C10+FB310_4*S10)-s.l(1,9)*FB39_4+s.l(3,9)*FB19_4+s.dpt(1,7)*(FB110_4*...
 S10-FB310_4*C10);
  CM39_4 = s.In(9,9)*OpM39_4-s.dpt(2,7)*(FB110_4*C10+FB310_4*S10)+s.l(1,9)*FB29_4-s.l(2,9)*FB19_4-CM110_4*S10+CM310_4*C10+FB210_4*s.dpt(1,7);
  FB19_5 = s.m(9)*(AlM19_5-s.l(2,9)*OpM39_5+s.l(3,9)*C6);
  FB29_5 = s.m(9)*(AlM29_5+s.l(1,9)*OpM39_5-s.l(3,9)*OpM19_5);
  FB39_5 = s.m(9)*(AlM39_5-s.l(1,9)*C6+s.l(2,9)*OpM19_5);
  FM19_5 = FB19_5+FB110_5*C10+FB310_5*S10;
  FM29_5 = FB210_5+FB29_5;
  FM39_5 = FB39_5-FB110_5*S10+FB310_5*C10;
  CM19_5 = s.In(1,9)*OpM19_5+s.In(2,9)*C6+s.In(3,9)*OpM39_5-s.dpt(2,7)*(FB110_5*S10-FB310_5*C10)-s.dpt(3,7)*FB210_5+s.l(2,9)*FB39_5-s.l(3,9)*...
 FB29_5+CM110_5*C10+CM310_5*S10;
  CM29_5 = CM210_5+s.In(5,9)*C6+s.In(6,9)*OpM39_5+s.dpt(3,7)*(FB110_5*C10+FB310_5*S10)-s.l(1,9)*FB39_5+s.l(3,9)*FB19_5+s.dpt(1,7)*(FB110_5*S10-...
 FB310_5*C10);
  CM39_5 = s.In(9,9)*OpM39_5-s.dpt(2,7)*(FB110_5*C10+FB310_5*S10)+s.l(1,9)*FB29_5-s.l(2,9)*FB19_5-CM110_5*S10+CM310_5*C10+FB210_5*s.dpt(1,7);
  FB19_6 = -s.m(9)*C9*(s.l(2,9)+s.dpt(2,2));
  FB29_6 = s.m(9)*(s.dpt(1,2)+s.l(1,9)*C9+s.l(3,9)*S9);
  FB39_6 = -s.m(9)*S9*(s.l(2,9)+s.dpt(2,2));
  CM29_6 = CM210_6+s.In(6,9)*C9+s.dpt(3,7)*(FB110_6*C10+FB310_6*S10)-s.l(1,9)*FB39_6+s.l(3,9)*FB19_6+s.dpt(1,7)*(FB110_6*S10-FB310_6*C10);
  CM29_9 = s.In(5,9)+CM210_9+s.dpt(3,7)*(FB110_9*C10+FB310_9*S10)+s.l(1,9)*s.l(1,9)*s.m(9)+s.l(3,9)*s.l(3,9)*s.m(9)+s.dpt(1,7)*(FB110_9*S10-...
 FB310_9*C10);

% = = Block_0_2_0_1_0_4 = = 
 
% Backward Dynamics 

  FA112 = -(s.frc(1,12)+s.m(12)*(s.l(1,12)*(OM212*OM212+OM312*OM312)-s.l(2,12)*(BS212-OpF312)-s.l(3,12)*(BS312+OpF26)-C12*(AlF111+s.dpt(2,9)*...
 BeF211+s.dpt(3,9)*BeF311+BS111*s.dpt(1,9))+S12*(AlF311+s.dpt(2,9)*BeF811+s.dpt(3,9)*BS911+BeF711*s.dpt(1,9))));
  FA212 = -(s.frc(2,12)-s.m(12)*(AlF211+s.dpt(2,9)*BS511+s.dpt(3,9)*BeF611+s.l(1,12)*(BS212+OpF312)-s.l(2,12)*(OM112*OM112+OM312*OM312)+BeF411*...
 s.dpt(1,9)+s.l(3,12)*(BS612-OpF112)));
  FA312 = -(s.frc(3,12)-s.m(12)*(s.l(1,12)*(BS312-OpF26)+s.l(2,12)*(BS612+OpF112)-s.l(3,12)*(OM112*OM112+OM212*OM212)+C12*(AlF311+s.dpt(2,9)*...
 BeF811+s.dpt(3,9)*BS911+BeF711*s.dpt(1,9))+S12*(AlF111+s.dpt(2,9)*BeF211+s.dpt(3,9)*BeF311+BS111*s.dpt(1,9))));
  CF112 = -(s.trq(1,12)-s.In(1,12)*OpF112-s.In(2,12)*OpF26-s.In(3,12)*OpF312-s.l(2,12)*FA312+FA212*s.l(3,12)+OM312*(s.In(5,12)*OM212+s.In(6,12)*...
 OM312-s.In(9,12)*OM212));
  CF212 = -(s.trq(2,12)-s.In(5,12)*OpF26-s.In(6,12)*OpF312+s.l(1,12)*FA312-FA112*s.l(3,12)-OM312*(s.In(1,12)*OM112+s.In(2,12)*OM212+s.In(3,12)*...
 OM312-s.In(9,12)*OM112));
  CF312 = -(s.trq(3,12)-s.In(9,12)*OpF312-s.l(1,12)*FA212+s.l(2,12)*FA112-OM112*(s.In(5,12)*OM212+s.In(6,12)*OM312)+OM212*(s.In(1,12)*OM112+...
 s.In(2,12)*OM212+s.In(3,12)*OM312));
  FB112_1 = s.m(12)*(AlM111_1*C12-AlM311_1*S12);
  FB212_1 = s.m(12)*AlM26_1;
  FB312_1 = s.m(12)*(AlM111_1*S12+AlM311_1*C12);
  CM112_1 = s.l(2,12)*FB312_1-FB212_1*s.l(3,12);
  CM212_1 = -(s.l(1,12)*FB312_1-FB112_1*s.l(3,12));
  CM312_1 = s.l(1,12)*FB212_1-s.l(2,12)*FB112_1;
  FB112_2 = s.m(12)*(AlM111_2*C12-AlM311_2*S12);
  FB212_2 = s.m(12)*AlM26_2;
  FB312_2 = s.m(12)*(AlM111_2*S12+AlM311_2*C12);
  CM112_2 = s.l(2,12)*FB312_2-FB212_2*s.l(3,12);
  CM212_2 = -(s.l(1,12)*FB312_2-FB112_2*s.l(3,12));
  CM312_2 = s.l(1,12)*FB212_2-s.l(2,12)*FB112_2;
  FB112_3 = s.m(12)*(AlM111_3*C12-AlM311_3*S12);
  FB212_3 = s.m(12)*AlM26_3;
  FB312_3 = s.m(12)*(AlM111_3*S12+AlM311_3*C12);
  CM112_3 = s.l(2,12)*FB312_3-FB212_3*s.l(3,12);
  CM212_3 = -(s.l(1,12)*FB312_3-FB112_3*s.l(3,12));
  CM312_3 = s.l(1,12)*FB212_3-s.l(2,12)*FB112_3;
  FB112_4 = s.m(12)*(C12*(AlM111_4-s.dpt(2,9)*OpM311_4+s.dpt(3,9)*OpM26_4)-S12*(AlM311_4+s.dpt(2,9)*OpM111_4-OpM26_4*s.dpt(1,9))-s.l(2,12)*...
 OpM312_4+OpM26_4*s.l(3,12));
  FB212_4 = s.m(12)*(AlM211_4-s.dpt(3,9)*OpM111_4+s.l(1,12)*OpM312_4-OpM112_4*s.l(3,12)+OpM311_4*s.dpt(1,9));
  FB312_4 = s.m(12)*(C12*(AlM311_4+s.dpt(2,9)*OpM111_4-OpM26_4*s.dpt(1,9))+S12*(AlM111_4-s.dpt(2,9)*OpM311_4+s.dpt(3,9)*OpM26_4)-s.l(1,12)*...
 OpM26_4+s.l(2,12)*OpM112_4);
  CM112_4 = s.In(1,12)*OpM112_4+s.In(2,12)*OpM26_4+s.In(3,12)*OpM312_4+s.l(2,12)*FB312_4-FB212_4*s.l(3,12);
  CM212_4 = s.In(5,12)*OpM26_4+s.In(6,12)*OpM312_4-s.l(1,12)*FB312_4+FB112_4*s.l(3,12);
  CM312_4 = s.In(9,12)*OpM312_4+s.l(1,12)*FB212_4-s.l(2,12)*FB112_4;
  FB112_5 = s.m(12)*(C12*(AlM111_5-s.dpt(2,9)*OpM311_5+s.dpt(3,9)*C6)-S12*(AlM311_5+s.dpt(2,9)*OpM111_5-s.dpt(1,9)*C6)-s.l(2,12)*OpM312_5+...
 s.l(3,12)*C6);
  FB212_5 = s.m(12)*(AlM211_5-s.dpt(3,9)*OpM111_5+s.l(1,12)*OpM312_5-OpM112_5*s.l(3,12)+OpM311_5*s.dpt(1,9));
  FB312_5 = s.m(12)*(C12*(AlM311_5+s.dpt(2,9)*OpM111_5-s.dpt(1,9)*C6)+S12*(AlM111_5-s.dpt(2,9)*OpM311_5+s.dpt(3,9)*C6)-s.l(1,12)*C6+s.l(2,12)*...
 OpM112_5);
  CM112_5 = s.In(1,12)*OpM112_5+s.In(2,12)*C6+s.In(3,12)*OpM312_5+s.l(2,12)*FB312_5-FB212_5*s.l(3,12);
  CM212_5 = s.In(5,12)*C6+s.In(6,12)*OpM312_5-s.l(1,12)*FB312_5+FB112_5*s.l(3,12);
  CM312_5 = s.In(9,12)*OpM312_5+s.l(1,12)*FB212_5-s.l(2,12)*FB112_5;
  FB112_6 = -s.m(12)*C11p12*(s.dpt(2,9)+s.l(2,12)+s.dpt(2,3));
  FB212_6 = s.m(12)*(s.dpt(1,3)+s.dpt(3,9)*S11+s.l(1,12)*C11p12+s.dpt(1,9)*C11+s.l(3,12)*S11p12);
  FB312_6 = -s.m(12)*S11p12*(s.dpt(2,9)+s.l(2,12)+s.dpt(2,3));
  CM112_6 = -(s.In(1,12)*S11p12-s.In(3,12)*C11p12-s.l(2,12)*FB312_6+FB212_6*s.l(3,12));
  CM212_6 = s.In(6,12)*C11p12-s.l(1,12)*FB312_6+FB112_6*s.l(3,12);
  CM312_6 = s.In(9,12)*C11p12+s.l(1,12)*FB212_6-s.l(2,12)*FB112_6;
  FB112_11 = s.m(12)*(s.l(3,12)+s.dpt(3,9)*C12+s.dpt(1,9)*S12);
  FB312_11 = -s.m(12)*(s.l(1,12)-s.dpt(3,9)*S12+s.dpt(1,9)*C12);
  CM212_11 = s.In(5,12)-s.l(1,12)*FB312_11+FB112_11*s.l(3,12);
  CM212_12 = s.In(5,12)+s.l(1,12)*s.l(1,12)*s.m(12)+s.m(12)*s.l(3,12)*s.l(3,12);
  FA111 = -(s.frc(1,11)-s.m(11)*(AlF111+s.l(1,11)*BS111+s.l(2,11)*BeF211+s.l(3,11)*BeF311));
  FA211 = -(s.frc(2,11)-s.m(11)*(AlF211+s.l(1,11)*BeF411+s.l(2,11)*BS511+s.l(3,11)*BeF611));
  FA311 = -(s.frc(3,11)-s.m(11)*(AlF311+s.l(1,11)*BeF711+s.l(2,11)*BeF811+s.l(3,11)*BS911));
  FF111 = FA111+FA112*C12+FA312*S12;
  FF211 = FA211+FA212;
  FF311 = FA311-FA112*S12+FA312*C12;
  CF111 = -(s.trq(1,11)-s.In(1,11)*OpF111-s.In(2,11)*OpF26-s.In(3,11)*OpF311+s.dpt(2,9)*(FA112*S12-FA312*C12)+s.dpt(3,9)*FA212-s.l(2,11)*FA311+...
 s.l(3,11)*FA211-CF112*C12-CF312*S12+OM311*(s.In(5,11)*OM211+s.In(6,11)*OM311-s.In(9,11)*OM211));
  CF211 = -(s.trq(2,11)-CF212-s.In(5,11)*OpF26-s.In(6,11)*OpF311-s.dpt(3,9)*(FA112*C12+FA312*S12)+s.l(1,11)*FA311-s.l(3,11)*FA111-OM311*(...
 s.In(1,11)*OM111+s.In(2,11)*OM211+s.In(3,11)*OM311-s.In(9,11)*OM111)-s.dpt(1,9)*(FA112*S12-FA312*C12));
  CF311 = -(s.trq(3,11)-s.In(9,11)*OpF311+s.dpt(2,9)*(FA112*C12+FA312*S12)-s.l(1,11)*FA211+s.l(2,11)*FA111+CF112*S12-CF312*C12-FA212*s.dpt(1,9)-...
 OM111*(s.In(5,11)*OM211+s.In(6,11)*OM311)+OM211*(s.In(1,11)*OM111+s.In(2,11)*OM211+s.In(3,11)*OM311));
  FB111_1 = s.m(11)*AlM111_1;
  FB211_1 = s.m(11)*AlM26_1;
  FB311_1 = s.m(11)*AlM311_1;
  FM111_1 = FB111_1+FB112_1*C12+FB312_1*S12;
  FM211_1 = FB211_1+FB212_1;
  FM311_1 = FB311_1-FB112_1*S12+FB312_1*C12;
  CM111_1 = -(s.dpt(2,9)*(FB112_1*S12-FB312_1*C12)+s.dpt(3,9)*FB212_1-s.l(2,11)*FB311_1+s.l(3,11)*FB211_1-CM112_1*C12-CM312_1*S12);
  CM211_1 = CM212_1+s.dpt(3,9)*(FB112_1*C12+FB312_1*S12)-s.l(1,11)*FB311_1+s.l(3,11)*FB111_1+s.dpt(1,9)*(FB112_1*S12-FB312_1*C12);
  CM311_1 = -(s.dpt(2,9)*(FB112_1*C12+FB312_1*S12)-s.l(1,11)*FB211_1+s.l(2,11)*FB111_1+CM112_1*S12-CM312_1*C12-FB212_1*s.dpt(1,9));
  FB111_2 = s.m(11)*AlM111_2;
  FB211_2 = s.m(11)*AlM26_2;
  FB311_2 = s.m(11)*AlM311_2;
  FM111_2 = FB111_2+FB112_2*C12+FB312_2*S12;
  FM211_2 = FB211_2+FB212_2;
  FM311_2 = FB311_2-FB112_2*S12+FB312_2*C12;
  CM111_2 = -(s.dpt(2,9)*(FB112_2*S12-FB312_2*C12)+s.dpt(3,9)*FB212_2-s.l(2,11)*FB311_2+s.l(3,11)*FB211_2-CM112_2*C12-CM312_2*S12);
  CM211_2 = CM212_2+s.dpt(3,9)*(FB112_2*C12+FB312_2*S12)-s.l(1,11)*FB311_2+s.l(3,11)*FB111_2+s.dpt(1,9)*(FB112_2*S12-FB312_2*C12);
  CM311_2 = -(s.dpt(2,9)*(FB112_2*C12+FB312_2*S12)-s.l(1,11)*FB211_2+s.l(2,11)*FB111_2+CM112_2*S12-CM312_2*C12-FB212_2*s.dpt(1,9));
  FB111_3 = s.m(11)*AlM111_3;
  FB211_3 = s.m(11)*AlM26_3;
  FB311_3 = s.m(11)*AlM311_3;
  FM111_3 = FB111_3+FB112_3*C12+FB312_3*S12;
  FM211_3 = FB211_3+FB212_3;
  FM311_3 = FB311_3-FB112_3*S12+FB312_3*C12;
  CM111_3 = -(s.dpt(2,9)*(FB112_3*S12-FB312_3*C12)+s.dpt(3,9)*FB212_3-s.l(2,11)*FB311_3+s.l(3,11)*FB211_3-CM112_3*C12-CM312_3*S12);
  CM211_3 = CM212_3+s.dpt(3,9)*(FB112_3*C12+FB312_3*S12)-s.l(1,11)*FB311_3+s.l(3,11)*FB111_3+s.dpt(1,9)*(FB112_3*S12-FB312_3*C12);
  CM311_3 = -(s.dpt(2,9)*(FB112_3*C12+FB312_3*S12)-s.l(1,11)*FB211_3+s.l(2,11)*FB111_3+CM112_3*S12-CM312_3*C12-FB212_3*s.dpt(1,9));
  FB111_4 = s.m(11)*(AlM111_4-s.l(2,11)*OpM311_4+s.l(3,11)*OpM26_4);
  FB211_4 = s.m(11)*(AlM211_4+s.l(1,11)*OpM311_4-s.l(3,11)*OpM111_4);
  FB311_4 = s.m(11)*(AlM311_4-s.l(1,11)*OpM26_4+s.l(2,11)*OpM111_4);
  FM111_4 = FB111_4+FB112_4*C12+FB312_4*S12;
  FM211_4 = FB211_4+FB212_4;
  FM311_4 = FB311_4-FB112_4*S12+FB312_4*C12;
  CM111_4 = s.In(1,11)*OpM111_4+s.In(2,11)*OpM26_4+s.In(3,11)*OpM311_4-s.dpt(2,9)*(FB112_4*S12-FB312_4*C12)-s.dpt(3,9)*FB212_4+s.l(2,11)*FB311_4-...
 s.l(3,11)*FB211_4+CM112_4*C12+CM312_4*S12;
  CM211_4 = CM212_4+s.In(5,11)*OpM26_4+s.In(6,11)*OpM311_4+s.dpt(3,9)*(FB112_4*C12+FB312_4*S12)-s.l(1,11)*FB311_4+s.l(3,11)*FB111_4+s.dpt(1,9)*(...
 FB112_4*S12-FB312_4*C12);
  CM311_4 = s.In(9,11)*OpM311_4-s.dpt(2,9)*(FB112_4*C12+FB312_4*S12)+s.l(1,11)*FB211_4-s.l(2,11)*FB111_4-CM112_4*S12+CM312_4*C12+FB212_4*...
 s.dpt(1,9);
  FB111_5 = s.m(11)*(AlM111_5-s.l(2,11)*OpM311_5+s.l(3,11)*C6);
  FB211_5 = s.m(11)*(AlM211_5+s.l(1,11)*OpM311_5-s.l(3,11)*OpM111_5);
  FB311_5 = s.m(11)*(AlM311_5-s.l(1,11)*C6+s.l(2,11)*OpM111_5);
  FM111_5 = FB111_5+FB112_5*C12+FB312_5*S12;
  FM211_5 = FB211_5+FB212_5;
  FM311_5 = FB311_5-FB112_5*S12+FB312_5*C12;
  CM111_5 = s.In(1,11)*OpM111_5+s.In(2,11)*C6+s.In(3,11)*OpM311_5-s.dpt(2,9)*(FB112_5*S12-FB312_5*C12)-s.dpt(3,9)*FB212_5+s.l(2,11)*FB311_5-...
 s.l(3,11)*FB211_5+CM112_5*C12+CM312_5*S12;
  CM211_5 = CM212_5+s.In(5,11)*C6+s.In(6,11)*OpM311_5+s.dpt(3,9)*(FB112_5*C12+FB312_5*S12)-s.l(1,11)*FB311_5+s.l(3,11)*FB111_5+s.dpt(1,9)*(...
 FB112_5*S12-FB312_5*C12);
  CM311_5 = s.In(9,11)*OpM311_5-s.dpt(2,9)*(FB112_5*C12+FB312_5*S12)+s.l(1,11)*FB211_5-s.l(2,11)*FB111_5-CM112_5*S12+CM312_5*C12+FB212_5*...
 s.dpt(1,9);
  FB111_6 = -s.m(11)*C11*(s.l(2,11)+s.dpt(2,3));
  FB211_6 = s.m(11)*(s.dpt(1,3)+s.l(1,11)*C11+s.l(3,11)*S11);
  FB311_6 = -s.m(11)*S11*(s.l(2,11)+s.dpt(2,3));
  CM211_6 = CM212_6+s.In(6,11)*C11+s.dpt(3,9)*(FB112_6*C12+FB312_6*S12)-s.l(1,11)*FB311_6+s.l(3,11)*FB111_6+s.dpt(1,9)*(FB112_6*S12-FB312_6*C12);
  CM211_11 = s.In(5,11)+CM212_11+s.dpt(3,9)*(FB112_11*C12+FB312_11*S12)+s.l(1,11)*s.l(1,11)*s.m(11)+s.l(3,11)*s.l(3,11)*s.m(11)+s.dpt(1,9)*(...
 FB112_11*S12-FB312_11*C12);

% = = Block_0_2_0_1_0_5 = = 
 
% Backward Dynamics 

  FA114 = -(s.frc(1,14)+s.m(14)*(s.l(1,14)*(OM214*OM214+OM314*OM314)-s.l(2,14)*(BS214-OpF314)-s.l(3,14)*(BS314+OpF26)-C14*(AlF113+s.dpt(2,11)*...
 BeF213+s.dpt(3,11)*BeF313+BS113*s.dpt(1,11))+S14*(AlF313+s.dpt(2,11)*BeF813+s.dpt(3,11)*BS913+BeF713*s.dpt(1,11))));
  FA214 = -(s.frc(2,14)-s.m(14)*(AlF213+s.dpt(2,11)*BS513+s.dpt(3,11)*BeF613+s.l(1,14)*(BS214+OpF314)-s.l(2,14)*(OM114*OM114+OM314*OM314)+BeF413*...
 s.dpt(1,11)+s.l(3,14)*(BS614-OpF114)));
  FA314 = -(s.frc(3,14)-s.m(14)*(s.l(1,14)*(BS314-OpF26)+s.l(2,14)*(BS614+OpF114)-s.l(3,14)*(OM114*OM114+OM214*OM214)+C14*(AlF313+s.dpt(2,11)*...
 BeF813+s.dpt(3,11)*BS913+BeF713*s.dpt(1,11))+S14*(AlF113+s.dpt(2,11)*BeF213+s.dpt(3,11)*BeF313+BS113*s.dpt(1,11))));
  CF114 = -(s.trq(1,14)-s.In(1,14)*OpF114-s.In(2,14)*OpF26-s.In(3,14)*OpF314-s.l(2,14)*FA314+FA214*s.l(3,14)+OM314*(s.In(5,14)*OM214+s.In(6,14)*...
 OM314-s.In(9,14)*OM214));
  CF214 = -(s.trq(2,14)-s.In(5,14)*OpF26-s.In(6,14)*OpF314+s.l(1,14)*FA314-FA114*s.l(3,14)-OM314*(s.In(1,14)*OM114+s.In(2,14)*OM214+s.In(3,14)*...
 OM314-s.In(9,14)*OM114));
  CF314 = -(s.trq(3,14)-s.In(9,14)*OpF314-s.l(1,14)*FA214+s.l(2,14)*FA114-OM114*(s.In(5,14)*OM214+s.In(6,14)*OM314)+OM214*(s.In(1,14)*OM114+...
 s.In(2,14)*OM214+s.In(3,14)*OM314));
  FB114_1 = s.m(14)*(AlM113_1*C14-AlM313_1*S14);
  FB214_1 = s.m(14)*AlM26_1;
  FB314_1 = s.m(14)*(AlM113_1*S14+AlM313_1*C14);
  CM114_1 = s.l(2,14)*FB314_1-FB214_1*s.l(3,14);
  CM214_1 = -(s.l(1,14)*FB314_1-FB114_1*s.l(3,14));
  CM314_1 = s.l(1,14)*FB214_1-s.l(2,14)*FB114_1;
  FB114_2 = s.m(14)*(AlM113_2*C14-AlM313_2*S14);
  FB214_2 = s.m(14)*AlM26_2;
  FB314_2 = s.m(14)*(AlM113_2*S14+AlM313_2*C14);
  CM114_2 = s.l(2,14)*FB314_2-FB214_2*s.l(3,14);
  CM214_2 = -(s.l(1,14)*FB314_2-FB114_2*s.l(3,14));
  CM314_2 = s.l(1,14)*FB214_2-s.l(2,14)*FB114_2;
  FB114_3 = s.m(14)*(AlM113_3*C14-AlM313_3*S14);
  FB214_3 = s.m(14)*AlM26_3;
  FB314_3 = s.m(14)*(AlM113_3*S14+AlM313_3*C14);
  CM114_3 = s.l(2,14)*FB314_3-FB214_3*s.l(3,14);
  CM214_3 = -(s.l(1,14)*FB314_3-FB114_3*s.l(3,14));
  CM314_3 = s.l(1,14)*FB214_3-s.l(2,14)*FB114_3;
  FB114_4 = s.m(14)*(C14*(AlM113_4-s.dpt(2,11)*OpM313_4+s.dpt(3,11)*OpM26_4)-S14*(AlM313_4+s.dpt(2,11)*OpM113_4-OpM26_4*s.dpt(1,11))-s.l(2,14)*...
 OpM314_4+OpM26_4*s.l(3,14));
  FB214_4 = s.m(14)*(AlM213_4-s.dpt(3,11)*OpM113_4+s.l(1,14)*OpM314_4-OpM114_4*s.l(3,14)+OpM313_4*s.dpt(1,11));
  FB314_4 = s.m(14)*(C14*(AlM313_4+s.dpt(2,11)*OpM113_4-OpM26_4*s.dpt(1,11))+S14*(AlM113_4-s.dpt(2,11)*OpM313_4+s.dpt(3,11)*OpM26_4)-s.l(1,14)*...
 OpM26_4+s.l(2,14)*OpM114_4);
  CM114_4 = s.In(1,14)*OpM114_4+s.In(2,14)*OpM26_4+s.In(3,14)*OpM314_4+s.l(2,14)*FB314_4-FB214_4*s.l(3,14);
  CM214_4 = s.In(5,14)*OpM26_4+s.In(6,14)*OpM314_4-s.l(1,14)*FB314_4+FB114_4*s.l(3,14);
  CM314_4 = s.In(9,14)*OpM314_4+s.l(1,14)*FB214_4-s.l(2,14)*FB114_4;
  FB114_5 = s.m(14)*(C14*(AlM113_5-s.dpt(2,11)*OpM313_5+s.dpt(3,11)*C6)-S14*(AlM313_5+s.dpt(2,11)*OpM113_5-s.dpt(1,11)*C6)-s.l(2,14)*OpM314_5+...
 s.l(3,14)*C6);
  FB214_5 = s.m(14)*(AlM213_5-s.dpt(3,11)*OpM113_5+s.l(1,14)*OpM314_5-OpM114_5*s.l(3,14)+OpM313_5*s.dpt(1,11));
  FB314_5 = s.m(14)*(C14*(AlM313_5+s.dpt(2,11)*OpM113_5-s.dpt(1,11)*C6)+S14*(AlM113_5-s.dpt(2,11)*OpM313_5+s.dpt(3,11)*C6)-s.l(1,14)*C6+s.l(2,14)...
 *OpM114_5);
  CM114_5 = s.In(1,14)*OpM114_5+s.In(2,14)*C6+s.In(3,14)*OpM314_5+s.l(2,14)*FB314_5-FB214_5*s.l(3,14);
  CM214_5 = s.In(5,14)*C6+s.In(6,14)*OpM314_5-s.l(1,14)*FB314_5+FB114_5*s.l(3,14);
  CM314_5 = s.In(9,14)*OpM314_5+s.l(1,14)*FB214_5-s.l(2,14)*FB114_5;
  FB114_6 = -s.m(14)*C13p14*(s.dpt(2,11)+s.l(2,14)+s.dpt(2,4));
  FB214_6 = s.m(14)*(s.dpt(1,4)+s.dpt(3,11)*S13+s.l(1,14)*C13p14+s.dpt(1,11)*C13+s.l(3,14)*S13p14);
  FB314_6 = -s.m(14)*S13p14*(s.dpt(2,11)+s.l(2,14)+s.dpt(2,4));
  CM114_6 = -(s.In(1,14)*S13p14-s.In(3,14)*C13p14-s.l(2,14)*FB314_6+FB214_6*s.l(3,14));
  CM214_6 = s.In(6,14)*C13p14-s.l(1,14)*FB314_6+FB114_6*s.l(3,14);
  CM314_6 = s.In(9,14)*C13p14+s.l(1,14)*FB214_6-s.l(2,14)*FB114_6;
  FB114_13 = s.m(14)*(s.l(3,14)+s.dpt(3,11)*C14+s.dpt(1,11)*S14);
  FB314_13 = -s.m(14)*(s.l(1,14)-s.dpt(3,11)*S14+s.dpt(1,11)*C14);
  CM214_13 = s.In(5,14)-s.l(1,14)*FB314_13+FB114_13*s.l(3,14);
  CM214_14 = s.In(5,14)+s.l(1,14)*s.l(1,14)*s.m(14)+s.m(14)*s.l(3,14)*s.l(3,14);
  FA113 = -(s.frc(1,13)-s.m(13)*(AlF113+s.l(1,13)*BS113+s.l(2,13)*BeF213+s.l(3,13)*BeF313));
  FA213 = -(s.frc(2,13)-s.m(13)*(AlF213+s.l(1,13)*BeF413+s.l(2,13)*BS513+s.l(3,13)*BeF613));
  FA313 = -(s.frc(3,13)-s.m(13)*(AlF313+s.l(1,13)*BeF713+s.l(2,13)*BeF813+s.l(3,13)*BS913));
  FF113 = FA113+FA114*C14+FA314*S14;
  FF213 = FA213+FA214;
  FF313 = FA313-FA114*S14+FA314*C14;
  CF113 = -(s.trq(1,13)-s.In(1,13)*OpF113-s.In(2,13)*OpF26-s.In(3,13)*OpF313+s.dpt(2,11)*(FA114*S14-FA314*C14)+s.dpt(3,11)*FA214-s.l(2,13)*FA313+...
 s.l(3,13)*FA213-CF114*C14-CF314*S14+OM313*(s.In(5,13)*OM213+s.In(6,13)*OM313-s.In(9,13)*OM213));
  CF213 = -(s.trq(2,13)-CF214-s.In(5,13)*OpF26-s.In(6,13)*OpF313-s.dpt(3,11)*(FA114*C14+FA314*S14)+s.l(1,13)*FA313-s.l(3,13)*FA113-OM313*(...
 s.In(1,13)*OM113+s.In(2,13)*OM213+s.In(3,13)*OM313-s.In(9,13)*OM113)-s.dpt(1,11)*(FA114*S14-FA314*C14));
  CF313 = -(s.trq(3,13)-s.In(9,13)*OpF313+s.dpt(2,11)*(FA114*C14+FA314*S14)-s.l(1,13)*FA213+s.l(2,13)*FA113+CF114*S14-CF314*C14-FA214*s.dpt(1,11)...
 -OM113*(s.In(5,13)*OM213+s.In(6,13)*OM313)+OM213*(s.In(1,13)*OM113+s.In(2,13)*OM213+s.In(3,13)*OM313));
  FB113_1 = s.m(13)*AlM113_1;
  FB213_1 = s.m(13)*AlM26_1;
  FB313_1 = s.m(13)*AlM313_1;
  FM113_1 = FB113_1+FB114_1*C14+FB314_1*S14;
  FM213_1 = FB213_1+FB214_1;
  FM313_1 = FB313_1-FB114_1*S14+FB314_1*C14;
  CM113_1 = -(s.dpt(2,11)*(FB114_1*S14-FB314_1*C14)+s.dpt(3,11)*FB214_1-s.l(2,13)*FB313_1+s.l(3,13)*FB213_1-CM114_1*C14-CM314_1*S14);
  CM213_1 = CM214_1+s.dpt(3,11)*(FB114_1*C14+FB314_1*S14)-s.l(1,13)*FB313_1+s.l(3,13)*FB113_1+s.dpt(1,11)*(FB114_1*S14-FB314_1*C14);
  CM313_1 = -(s.dpt(2,11)*(FB114_1*C14+FB314_1*S14)-s.l(1,13)*FB213_1+s.l(2,13)*FB113_1+CM114_1*S14-CM314_1*C14-FB214_1*s.dpt(1,11));
  FB113_2 = s.m(13)*AlM113_2;
  FB213_2 = s.m(13)*AlM26_2;
  FB313_2 = s.m(13)*AlM313_2;
  FM113_2 = FB113_2+FB114_2*C14+FB314_2*S14;
  FM213_2 = FB213_2+FB214_2;
  FM313_2 = FB313_2-FB114_2*S14+FB314_2*C14;
  CM113_2 = -(s.dpt(2,11)*(FB114_2*S14-FB314_2*C14)+s.dpt(3,11)*FB214_2-s.l(2,13)*FB313_2+s.l(3,13)*FB213_2-CM114_2*C14-CM314_2*S14);
  CM213_2 = CM214_2+s.dpt(3,11)*(FB114_2*C14+FB314_2*S14)-s.l(1,13)*FB313_2+s.l(3,13)*FB113_2+s.dpt(1,11)*(FB114_2*S14-FB314_2*C14);
  CM313_2 = -(s.dpt(2,11)*(FB114_2*C14+FB314_2*S14)-s.l(1,13)*FB213_2+s.l(2,13)*FB113_2+CM114_2*S14-CM314_2*C14-FB214_2*s.dpt(1,11));
  FB113_3 = s.m(13)*AlM113_3;
  FB213_3 = s.m(13)*AlM26_3;
  FB313_3 = s.m(13)*AlM313_3;
  FM113_3 = FB113_3+FB114_3*C14+FB314_3*S14;
  FM213_3 = FB213_3+FB214_3;
  FM313_3 = FB313_3-FB114_3*S14+FB314_3*C14;
  CM113_3 = -(s.dpt(2,11)*(FB114_3*S14-FB314_3*C14)+s.dpt(3,11)*FB214_3-s.l(2,13)*FB313_3+s.l(3,13)*FB213_3-CM114_3*C14-CM314_3*S14);
  CM213_3 = CM214_3+s.dpt(3,11)*(FB114_3*C14+FB314_3*S14)-s.l(1,13)*FB313_3+s.l(3,13)*FB113_3+s.dpt(1,11)*(FB114_3*S14-FB314_3*C14);
  CM313_3 = -(s.dpt(2,11)*(FB114_3*C14+FB314_3*S14)-s.l(1,13)*FB213_3+s.l(2,13)*FB113_3+CM114_3*S14-CM314_3*C14-FB214_3*s.dpt(1,11));
  FB113_4 = s.m(13)*(AlM113_4-s.l(2,13)*OpM313_4+s.l(3,13)*OpM26_4);
  FB213_4 = s.m(13)*(AlM213_4+s.l(1,13)*OpM313_4-s.l(3,13)*OpM113_4);
  FB313_4 = s.m(13)*(AlM313_4-s.l(1,13)*OpM26_4+s.l(2,13)*OpM113_4);
  FM113_4 = FB113_4+FB114_4*C14+FB314_4*S14;
  FM213_4 = FB213_4+FB214_4;
  FM313_4 = FB313_4-FB114_4*S14+FB314_4*C14;
  CM113_4 = s.In(1,13)*OpM113_4+s.In(2,13)*OpM26_4+s.In(3,13)*OpM313_4-s.dpt(2,11)*(FB114_4*S14-FB314_4*C14)-s.dpt(3,11)*FB214_4+s.l(2,13)*...
 FB313_4-s.l(3,13)*FB213_4+CM114_4*C14+CM314_4*S14;
  CM213_4 = CM214_4+s.In(5,13)*OpM26_4+s.In(6,13)*OpM313_4+s.dpt(3,11)*(FB114_4*C14+FB314_4*S14)-s.l(1,13)*FB313_4+s.l(3,13)*FB113_4+s.dpt(1,11)*...
 (FB114_4*S14-FB314_4*C14);
  CM313_4 = s.In(9,13)*OpM313_4-s.dpt(2,11)*(FB114_4*C14+FB314_4*S14)+s.l(1,13)*FB213_4-s.l(2,13)*FB113_4-CM114_4*S14+CM314_4*C14+FB214_4*...
 s.dpt(1,11);
  FB113_5 = s.m(13)*(AlM113_5-s.l(2,13)*OpM313_5+s.l(3,13)*C6);
  FB213_5 = s.m(13)*(AlM213_5+s.l(1,13)*OpM313_5-s.l(3,13)*OpM113_5);
  FB313_5 = s.m(13)*(AlM313_5-s.l(1,13)*C6+s.l(2,13)*OpM113_5);
  FM113_5 = FB113_5+FB114_5*C14+FB314_5*S14;
  FM213_5 = FB213_5+FB214_5;
  FM313_5 = FB313_5-FB114_5*S14+FB314_5*C14;
  CM113_5 = s.In(1,13)*OpM113_5+s.In(2,13)*C6+s.In(3,13)*OpM313_5-s.dpt(2,11)*(FB114_5*S14-FB314_5*C14)-s.dpt(3,11)*FB214_5+s.l(2,13)*FB313_5-...
 s.l(3,13)*FB213_5+CM114_5*C14+CM314_5*S14;
  CM213_5 = CM214_5+s.In(5,13)*C6+s.In(6,13)*OpM313_5+s.dpt(3,11)*(FB114_5*C14+FB314_5*S14)-s.l(1,13)*FB313_5+s.l(3,13)*FB113_5+s.dpt(1,11)*(...
 FB114_5*S14-FB314_5*C14);
  CM313_5 = s.In(9,13)*OpM313_5-s.dpt(2,11)*(FB114_5*C14+FB314_5*S14)+s.l(1,13)*FB213_5-s.l(2,13)*FB113_5-CM114_5*S14+CM314_5*C14+FB214_5*...
 s.dpt(1,11);
  FB113_6 = -s.m(13)*C13*(s.l(2,13)+s.dpt(2,4));
  FB213_6 = s.m(13)*(s.dpt(1,4)+s.l(1,13)*C13+s.l(3,13)*S13);
  FB313_6 = -s.m(13)*S13*(s.l(2,13)+s.dpt(2,4));
  CM213_6 = CM214_6+s.In(6,13)*C13+s.dpt(3,11)*(FB114_6*C14+FB314_6*S14)-s.l(1,13)*FB313_6+s.l(3,13)*FB113_6+s.dpt(1,11)*(FB114_6*S14-FB314_6*C14...
 );
  CM213_13 = s.In(5,13)+CM214_13+s.dpt(3,11)*(FB114_13*C14+FB314_13*S14)+s.l(1,13)*s.l(1,13)*s.m(13)+s.l(3,13)*s.l(3,13)*s.m(13)+s.dpt(1,11)*(...
 FB114_13*S14-FB314_13*C14);

% = = Block_0_2_0_2_0_1 = = 
 
% Backward Dynamics 

  FA16 = -(s.frc(1,6)-s.m(6)*(AlF16+s.l(1,6)*BS16+s.l(2,6)*BeF26+s.l(3,6)*BeF36));
  FA26 = -(s.frc(2,6)-s.m(6)*(AlF26+s.l(1,6)*BeF46+s.l(2,6)*BS56+s.l(3,6)*BeF66));
  FA36 = -(s.frc(3,6)-s.m(6)*(AlF35+s.l(1,6)*BeF76+s.l(2,6)*BeF86+s.l(3,6)*BS96));
  FF16 = FA16+FF111*C11+FF113*C13+FF17*C7+FF19*C9+FF311*S11+FF313*S13+FF37*S7+FF39*S9;
  FF26 = FA26+FF211+FF213+FF27+FF29;
  CF16 = -(s.trq(1,6)-s.In(1,6)*OpF16-s.In(2,6)*OpF26-s.In(3,6)*OpF35-s.l(2,6)*FA36+s.l(3,6)*FA26-CF111*C11-CF113*C13-CF17*C7-CF19*C9-CF311*S11-...
 CF313*S13-CF37*S7-CF39*S9+FF211*s.dpt(3,3)+FF213*s.dpt(3,4)+FF27*s.dpt(3,1)+FF29*s.dpt(3,2)+OM36*(s.In(5,6)*OM26+s.In(6,6)*OM36-s.In(9,6)*OM26)+...
 s.dpt(2,1)*(FF17*S7-FF37*C7)+s.dpt(2,2)*(FF19*S9-FF39*C9)+s.dpt(2,3)*(FF111*S11-FF311*C11)+s.dpt(2,4)*(FF113*S13-FF313*C13));
  CF26 = -(s.trq(2,6)-CF211-CF213-CF27-CF29-s.In(5,6)*OpF26-s.In(6,6)*OpF35+s.l(1,6)*FA36-s.l(3,6)*FA16-OM36*(s.In(1,6)*OM16+s.In(2,6)*OM26+...
 s.In(3,6)*OM36-s.In(9,6)*OM16)-s.dpt(1,1)*(FF17*S7-FF37*C7)-s.dpt(1,2)*(FF19*S9-FF39*C9)-s.dpt(1,3)*(FF111*S11-FF311*C11)-s.dpt(1,4)*(FF113*S13-FF313...
 *C13)-s.dpt(3,1)*(FF17*C7+FF37*S7)-s.dpt(3,2)*(FF19*C9+FF39*S9)-s.dpt(3,3)*(FF111*C11+FF311*S11)-s.dpt(3,4)*(FF113*C13+FF313*S13));
  CF36 = -(s.trq(3,6)-s.In(9,6)*OpF35-s.l(1,6)*FA26+s.l(2,6)*FA16+CF111*S11+CF113*S13+CF17*S7+CF19*S9-CF311*C11-CF313*C13-CF37*C7-CF39*C9-FF211*...
 s.dpt(1,3)-FF213*s.dpt(1,4)-FF27*s.dpt(1,1)-FF29*s.dpt(1,2)-OM16*(s.In(5,6)*OM26+s.In(6,6)*OM36)+OM26*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)+...
 s.dpt(2,1)*(FF17*C7+FF37*S7)+s.dpt(2,2)*(FF19*C9+FF39*S9)+s.dpt(2,3)*(FF111*C11+FF311*S11)+s.dpt(2,4)*(FF113*C13+FF313*S13));
  FB16_1 = s.m(6)*AlM16_1;
  FB26_1 = s.m(6)*AlM26_1;
  FB36_1 = s.m(6)*S5;
  FM16_1 = FB16_1+FM111_1*C11+FM113_1*C13+FM17_1*C7+FM19_1*C9+FM311_1*S11+FM313_1*S13+FM37_1*S7+FM39_1*S9;
  FM26_1 = FB26_1+FM211_1+FM213_1+FM27_1+FM29_1;
  CM16_1 = s.l(2,6)*FB36_1-s.l(3,6)*FB26_1+CM111_1*C11+CM113_1*C13+CM17_1*C7+CM19_1*C9+CM311_1*S11+CM313_1*S13+CM37_1*S7+CM39_1*S9-FM211_1*...
 s.dpt(3,3)-FM213_1*s.dpt(3,4)-FM27_1*s.dpt(3,1)-FM29_1*s.dpt(3,2)-s.dpt(2,1)*(FM17_1*S7-FM37_1*C7)-s.dpt(2,2)*(FM19_1*S9-FM39_1*C9)-s.dpt(2,3)*(...
 FM111_1*S11-FM311_1*C11)-s.dpt(2,4)*(FM113_1*S13-FM313_1*C13);
  CM26_1 = CM211_1+CM213_1+CM27_1+CM29_1-s.l(1,6)*FB36_1+s.l(3,6)*FB16_1+s.dpt(1,1)*(FM17_1*S7-FM37_1*C7)+s.dpt(1,2)*(FM19_1*S9-FM39_1*C9)+...
 s.dpt(1,3)*(FM111_1*S11-FM311_1*C11)+s.dpt(1,4)*(FM113_1*S13-FM313_1*C13)+s.dpt(3,1)*(FM17_1*C7+FM37_1*S7)+s.dpt(3,2)*(FM19_1*C9+FM39_1*S9)+...
 s.dpt(3,3)*(FM111_1*C11+FM311_1*S11)+s.dpt(3,4)*(FM113_1*C13+FM313_1*S13);
  CM36_1 = s.l(1,6)*FB26_1-s.l(2,6)*FB16_1-CM111_1*S11-CM113_1*S13-CM17_1*S7-CM19_1*S9+CM311_1*C11+CM313_1*C13+CM37_1*C7+CM39_1*C9+FM211_1*...
 s.dpt(1,3)+FM213_1*s.dpt(1,4)+FM27_1*s.dpt(1,1)+FM29_1*s.dpt(1,2)-s.dpt(2,1)*(FM17_1*C7+FM37_1*S7)-s.dpt(2,2)*(FM19_1*C9+FM39_1*S9)-s.dpt(2,3)*(...
 FM111_1*C11+FM311_1*S11)-s.dpt(2,4)*(FM113_1*C13+FM313_1*S13);
  FB16_2 = s.m(6)*AlM16_2;
  FB26_2 = s.m(6)*AlM26_2;
  FB36_2 = s.m(6)*AlM35_2;
  FM16_2 = FB16_2+FM111_2*C11+FM113_2*C13+FM17_2*C7+FM19_2*C9+FM311_2*S11+FM313_2*S13+FM37_2*S7+FM39_2*S9;
  FM26_2 = FB26_2+FM211_2+FM213_2+FM27_2+FM29_2;
  CM16_2 = s.l(2,6)*FB36_2-s.l(3,6)*FB26_2+CM111_2*C11+CM113_2*C13+CM17_2*C7+CM19_2*C9+CM311_2*S11+CM313_2*S13+CM37_2*S7+CM39_2*S9-FM211_2*...
 s.dpt(3,3)-FM213_2*s.dpt(3,4)-FM27_2*s.dpt(3,1)-FM29_2*s.dpt(3,2)-s.dpt(2,1)*(FM17_2*S7-FM37_2*C7)-s.dpt(2,2)*(FM19_2*S9-FM39_2*C9)-s.dpt(2,3)*(...
 FM111_2*S11-FM311_2*C11)-s.dpt(2,4)*(FM113_2*S13-FM313_2*C13);
  CM26_2 = CM211_2+CM213_2+CM27_2+CM29_2-s.l(1,6)*FB36_2+s.l(3,6)*FB16_2+s.dpt(1,1)*(FM17_2*S7-FM37_2*C7)+s.dpt(1,2)*(FM19_2*S9-FM39_2*C9)+...
 s.dpt(1,3)*(FM111_2*S11-FM311_2*C11)+s.dpt(1,4)*(FM113_2*S13-FM313_2*C13)+s.dpt(3,1)*(FM17_2*C7+FM37_2*S7)+s.dpt(3,2)*(FM19_2*C9+FM39_2*S9)+...
 s.dpt(3,3)*(FM111_2*C11+FM311_2*S11)+s.dpt(3,4)*(FM113_2*C13+FM313_2*S13);
  CM36_2 = s.l(1,6)*FB26_2-s.l(2,6)*FB16_2-CM111_2*S11-CM113_2*S13-CM17_2*S7-CM19_2*S9+CM311_2*C11+CM313_2*C13+CM37_2*C7+CM39_2*C9+FM211_2*...
 s.dpt(1,3)+FM213_2*s.dpt(1,4)+FM27_2*s.dpt(1,1)+FM29_2*s.dpt(1,2)-s.dpt(2,1)*(FM17_2*C7+FM37_2*S7)-s.dpt(2,2)*(FM19_2*C9+FM39_2*S9)-s.dpt(2,3)*(...
 FM111_2*C11+FM311_2*S11)-s.dpt(2,4)*(FM113_2*C13+FM313_2*S13);
  FB16_3 = s.m(6)*AlM16_3;
  FB26_3 = s.m(6)*AlM26_3;
  FB36_3 = s.m(6)*AlM35_3;
  FM16_3 = FB16_3+FM111_3*C11+FM113_3*C13+FM17_3*C7+FM19_3*C9+FM311_3*S11+FM313_3*S13+FM37_3*S7+FM39_3*S9;
  FM26_3 = FB26_3+FM211_3+FM213_3+FM27_3+FM29_3;
  CM16_3 = s.l(2,6)*FB36_3-s.l(3,6)*FB26_3+CM111_3*C11+CM113_3*C13+CM17_3*C7+CM19_3*C9+CM311_3*S11+CM313_3*S13+CM37_3*S7+CM39_3*S9-FM211_3*...
 s.dpt(3,3)-FM213_3*s.dpt(3,4)-FM27_3*s.dpt(3,1)-FM29_3*s.dpt(3,2)-s.dpt(2,1)*(FM17_3*S7-FM37_3*C7)-s.dpt(2,2)*(FM19_3*S9-FM39_3*C9)-s.dpt(2,3)*(...
 FM111_3*S11-FM311_3*C11)-s.dpt(2,4)*(FM113_3*S13-FM313_3*C13);
  CM26_3 = CM211_3+CM213_3+CM27_3+CM29_3-s.l(1,6)*FB36_3+s.l(3,6)*FB16_3+s.dpt(1,1)*(FM17_3*S7-FM37_3*C7)+s.dpt(1,2)*(FM19_3*S9-FM39_3*C9)+...
 s.dpt(1,3)*(FM111_3*S11-FM311_3*C11)+s.dpt(1,4)*(FM113_3*S13-FM313_3*C13)+s.dpt(3,1)*(FM17_3*C7+FM37_3*S7)+s.dpt(3,2)*(FM19_3*C9+FM39_3*S9)+...
 s.dpt(3,3)*(FM111_3*C11+FM311_3*S11)+s.dpt(3,4)*(FM113_3*C13+FM313_3*S13);
  CM36_3 = s.l(1,6)*FB26_3-s.l(2,6)*FB16_3-CM111_3*S11-CM113_3*S13-CM17_3*S7-CM19_3*S9+CM311_3*C11+CM313_3*C13+CM37_3*C7+CM39_3*C9+FM211_3*...
 s.dpt(1,3)+FM213_3*s.dpt(1,4)+FM27_3*s.dpt(1,1)+FM29_3*s.dpt(1,2)-s.dpt(2,1)*(FM17_3*C7+FM37_3*S7)-s.dpt(2,2)*(FM19_3*C9+FM39_3*S9)-s.dpt(2,3)*(...
 FM111_3*C11+FM311_3*S11)-s.dpt(2,4)*(FM113_3*C13+FM313_3*S13);
  FB16_4 = -s.m(6)*(s.l(2,6)*S5-s.l(3,6)*OpM26_4);
  FB26_4 = s.m(6)*(s.l(1,6)*S5-s.l(3,6)*OpM16_4);
  FB36_4 = -s.m(6)*(s.l(1,6)*OpM26_4-s.l(2,6)*OpM16_4);
  CM16_4 = s.In(1,6)*OpM16_4+s.In(2,6)*OpM26_4+s.In(3,6)*S5+s.l(2,6)*FB36_4-s.l(3,6)*FB26_4+CM111_4*C11+CM113_4*C13+CM17_4*C7+CM19_4*C9+CM311_4*...
 S11+CM313_4*S13+CM37_4*S7+CM39_4*S9-FM211_4*s.dpt(3,3)-FM213_4*s.dpt(3,4)-FM27_4*s.dpt(3,1)-FM29_4*s.dpt(3,2)-s.dpt(2,1)*(FM17_4*S7-FM37_4*C7)-...
 s.dpt(2,2)*(FM19_4*S9-FM39_4*C9)-s.dpt(2,3)*(FM111_4*S11-FM311_4*C11)-s.dpt(2,4)*(FM113_4*S13-FM313_4*C13);
  CM26_4 = CM211_4+CM213_4+CM27_4+CM29_4+s.In(5,6)*OpM26_4+s.In(6,6)*S5-s.l(1,6)*FB36_4+s.l(3,6)*FB16_4+s.dpt(1,1)*(FM17_4*S7-FM37_4*C7)+...
 s.dpt(1,2)*(FM19_4*S9-FM39_4*C9)+s.dpt(1,3)*(FM111_4*S11-FM311_4*C11)+s.dpt(1,4)*(FM113_4*S13-FM313_4*C13)+s.dpt(3,1)*(FM17_4*C7+FM37_4*S7)+...
 s.dpt(3,2)*(FM19_4*C9+FM39_4*S9)+s.dpt(3,3)*(FM111_4*C11+FM311_4*S11)+s.dpt(3,4)*(FM113_4*C13+FM313_4*S13);
  CM36_4 = s.In(9,6)*S5+s.l(1,6)*FB26_4-s.l(2,6)*FB16_4-CM111_4*S11-CM113_4*S13-CM17_4*S7-CM19_4*S9+CM311_4*C11+CM313_4*C13+CM37_4*C7+CM39_4*C9+...
 FM211_4*s.dpt(1,3)+FM213_4*s.dpt(1,4)+FM27_4*s.dpt(1,1)+FM29_4*s.dpt(1,2)-s.dpt(2,1)*(FM17_4*C7+FM37_4*S7)-s.dpt(2,2)*(FM19_4*C9+FM39_4*S9)-...
 s.dpt(2,3)*(FM111_4*C11+FM311_4*S11)-s.dpt(2,4)*(FM113_4*C13+FM313_4*S13);
  FB16_5 = s.l(3,6)*s.m(6)*C6;
  FB26_5 = -s.l(3,6)*s.m(6)*S6;
  FB36_5 = -s.m(6)*(s.l(1,6)*C6-s.l(2,6)*S6);
  CM36_5 = s.l(1,6)*FB26_5-s.l(2,6)*FB16_5-CM111_5*S11-CM113_5*S13-CM17_5*S7-CM19_5*S9+CM311_5*C11+CM313_5*C13+CM37_5*C7+CM39_5*C9+FM211_5*...
 s.dpt(1,3)+FM213_5*s.dpt(1,4)+FM27_5*s.dpt(1,1)+FM29_5*s.dpt(1,2)-s.dpt(2,1)*(FM17_5*C7+FM37_5*S7)-s.dpt(2,2)*(FM19_5*C9+FM39_5*S9)-s.dpt(2,3)*(...
 FM111_5*C11+FM311_5*S11)-s.dpt(2,4)*(FM113_5*C13+FM313_5*S13);
  CM36_6 = s.In(9,6)+s.l(1,6)*s.l(1,6)*s.m(6)+s.l(2,6)*s.l(2,6)*s.m(6)+s.dpt(1,1)*(FB27_6+FB28_6)+s.dpt(1,2)*(FB210_6+FB29_6)+s.dpt(1,3)*(FB211_6...
 +FB212_6)+s.dpt(1,4)*(FB213_6+FB214_6)-s.dpt(2,1)*(C7*(FB17_6+FB18_6*C8+FB38_6*S8)+S7*(FB37_6-FB18_6*S8+FB38_6*C8))-s.dpt(2,2)*(C9*(FB19_6+FB110_6*...
 C10+FB310_6*S10)+S9*(FB39_6-FB110_6*S10+FB310_6*C10))-s.dpt(2,3)*(C11*(FB111_6+FB112_6*C12+FB312_6*S12)+S11*(FB311_6-FB112_6*S12+FB312_6*C12))-...
 s.dpt(2,4)*(C13*(FB113_6+FB114_6*C14+FB314_6*S14)+S13*(FB313_6-FB114_6*S14+FB314_6*C14))+C11*(s.In(9,11)*C11-s.dpt(2,9)*(FB112_6*C12+FB312_6*S12)+...
 s.l(1,11)*FB211_6-s.l(2,11)*FB111_6-CM112_6*S12+CM312_6*C12+FB212_6*s.dpt(1,9))+S11*(s.In(1,11)*S11-s.In(3,11)*C11+s.dpt(2,9)*(FB112_6*S12-FB312_6*...
 C12)+s.dpt(3,9)*FB212_6-s.l(2,11)*FB311_6+s.l(3,11)*FB211_6-CM112_6*C12-CM312_6*S12)+C13*(s.In(9,13)*C13-s.dpt(2,11)*(FB114_6*C14+FB314_6*S14)+...
 s.l(1,13)*FB213_6-s.l(2,13)*FB113_6-CM114_6*S14+CM314_6*C14+FB214_6*s.dpt(1,11))+S13*(s.In(1,13)*S13-s.In(3,13)*C13+s.dpt(2,11)*(FB114_6*S14-FB314_6*...
 C14)+s.dpt(3,11)*FB214_6-s.l(2,13)*FB313_6+s.l(3,13)*FB213_6-CM114_6*C14-CM314_6*S14)+C7*(s.In(9,7)*C7-s.dpt(2,5)*(FB18_6*C8+FB38_6*S8)+s.l(1,7)*...
 FB27_6-s.l(2,7)*FB17_6-CM18_6*S8+CM38_6*C8+FB28_6*s.dpt(1,5))+S7*(s.In(1,7)*S7-s.In(3,7)*C7+s.dpt(2,5)*(FB18_6*S8-FB38_6*C8)+s.dpt(3,5)*FB28_6-...
 s.l(2,7)*FB37_6+s.l(3,7)*FB27_6-CM18_6*C8-CM38_6*S8)+C9*(s.In(9,9)*C9-s.dpt(2,7)*(FB110_6*C10+FB310_6*S10)+s.l(1,9)*FB29_6-s.l(2,9)*FB19_6-CM110_6*...
 S10+CM310_6*C10+FB210_6*s.dpt(1,7))+S9*(s.In(1,9)*S9-s.In(3,9)*C9+s.dpt(2,7)*(FB110_6*S10-FB310_6*C10)+s.dpt(3,7)*FB210_6-s.l(2,9)*FB39_6+s.l(3,9)*...
 FB29_6-CM110_6*C10-CM310_6*S10);
  FA15 = -(s.frc(1,5)-s.m(5)*(AlF15-s.l(1,5)*(qd(5)*qd(5)+OM35*OM35)+s.l(3,5)*BS35));
  FA25 = -(s.frc(2,5)-s.m(5)*(AlF24-qd(4)*qd(4)*s.l(2,5)+s.l(1,5)*(OpF35+qd(5)*OM15)-s.l(3,5)*(OpF15-qd(5)*OM35)));
  FA35 = -(s.frc(3,5)-s.m(5)*(AlF35+s.l(1,5)*BS35-s.l(3,5)*(qd(5)*qd(5)+OM15*OM15)));
  FF15 = FA15+FF16*C6-FF26*S6;
  FF35 = FA35+FA36-FF111*S11-FF113*S13-FF17*S7-FF19*S9+FF311*C11+FF313*C13+FF37*C7+FF39*C9;
  CF25 = -(s.trq(2,5)-s.In(6,5)*OpF35+s.l(1,5)*FA35-s.l(3,5)*FA15-CF16*S6-CF26*C6-OM35*(qd(5)*s.In(2,5)+s.In(1,5)*OM15+s.In(3,5)*OM35-s.In(9,5)*...
 OM15));
  FB15_1 = s.m(5)*C5;
  FB35_1 = s.m(5)*S5;
  FM51_26 = FM16_1*S6+FM26_1*C6;
  FM15_1 = FB15_1+FM16_1*C6-FM26_1*S6;
  FM35_1 = FB35_1+FB36_1-FM111_1*S11-FM113_1*S13-FM17_1*S7-FM19_1*S9+FM311_1*C11+FM313_1*C13+FM37_1*C7+FM39_1*C9;
  CM25_1 = CM16_1*S6+CM26_1*C6-s.l(1,5)*FB35_1+s.l(3,5)*FB15_1;
  FB15_2 = s.m(5)*AlM15_2;
  FB25_2 = s.m(5)*C4;
  FB35_2 = s.m(5)*AlM35_2;
  CM25_2 = CM16_2*S6+CM26_2*C6-s.l(1,5)*FB35_2+s.l(3,5)*FB15_2;
  FB15_3 = s.m(5)*AlM15_3;
  FB25_3 = s.m(5)*S4;
  FB35_3 = s.m(5)*AlM35_3;
  CM25_3 = CM16_3*S6+CM26_3*C6-s.l(1,5)*FB35_3+s.l(3,5)*FB15_3;
  FB15_4 = -s.l(2,5)*s.m(5)*S5;
  FB25_4 = s.m(5)*(s.l(1,5)*S5-s.l(3,5)*C5);
  FB35_4 = s.l(2,5)*s.m(5)*C5;
  CM25_4 = s.In(6,5)*S5-s.l(1,5)*FB35_4+s.l(3,5)*FB15_4+CM16_4*S6+CM26_4*C6;
  CM25_5 = s.In(5,5)+s.l(1,5)*s.l(1,5)*s.m(5)+s.l(3,5)*s.l(3,5)*s.m(5)+C6*(CM211_5+CM213_5+CM27_5+CM29_5+s.In(5,6)*C6-s.l(1,6)*FB36_5+s.l(3,6)*...
 FB16_5+s.dpt(1,1)*(FM17_5*S7-FM37_5*C7)+s.dpt(1,2)*(FM19_5*S9-FM39_5*C9)+s.dpt(1,3)*(FM111_5*S11-FM311_5*C11)+s.dpt(1,4)*(FM113_5*S13-FM313_5*C13)+...
 s.dpt(3,1)*(FM17_5*C7+FM37_5*S7)+s.dpt(3,2)*(FM19_5*C9+FM39_5*S9)+s.dpt(3,3)*(FM111_5*C11+FM311_5*S11)+s.dpt(3,4)*(FM113_5*C13+FM313_5*S13))+S6*(...
 s.In(1,6)*S6+s.In(2,6)*C6+s.l(2,6)*FB36_5-s.l(3,6)*FB26_5+CM111_5*C11+CM113_5*C13+CM17_5*C7+CM19_5*C9+CM311_5*S11+CM313_5*S13+CM37_5*S7+CM39_5*S9-...
 FM211_5*s.dpt(3,3)-FM213_5*s.dpt(3,4)-FM27_5*s.dpt(3,1)-FM29_5*s.dpt(3,2)-s.dpt(2,1)*(FM17_5*S7-FM37_5*C7)-s.dpt(2,2)*(FM19_5*S9-FM39_5*C9)-...
 s.dpt(2,3)*(FM111_5*S11-FM311_5*C11)-s.dpt(2,4)*(FM113_5*S13-FM313_5*C13));
  FA24 = -(s.frc(2,4)-s.m(4)*(AlF24-qd(4)*qd(4)*s.l(2,4)));
  FA34 = -(s.frc(3,4)-s.m(4)*(AlF34-qd(4)*qd(4)*s.l(3,4)));
  FF24 = FA24+FA25+FF16*S6+FF26*C6;
  FF34 = FA34-FF15*S5+FF35*C5;
  CF14 = -(s.trq(1,4)-s.l(2,4)*FA34+s.l(3,4)*FA24+C5*(s.trq(1,5)-s.In(1,5)*OpF15-s.In(3,5)*OpF35-s.l(2,5)*FA35+s.l(3,5)*FA25-CF16*C6+CF26*S6+OM35...
 *(qd(5)*(s.In(5,5)-s.In(9,5))+s.In(6,5)*OM35))+S5*(s.trq(3,5)-CF36+qd(5)*(qd(5)*s.In(2,5)+s.In(1,5)*OM15+s.In(3,5)*OM35)-s.In(9,5)*OpF35-s.l(1,5)*...
 FA25+s.l(2,5)*FA15-OM15*(qd(5)*s.In(5,5)+s.In(6,5)*OM35)));
  FM41_35 = -(FM15_1*S5-FM35_1*C5);
  CM41_15 = C5*(s.l(2,5)*FB35_1+CM16_1*C6-CM26_1*S6)+S5*(CM36_1-s.l(2,5)*FB15_1);
  FB24_2 = s.m(4)*C4;
  FB34_2 = -s.m(4)*S4;
  FM24_2 = FB24_2+FB25_2+FM16_2*S6+FM26_2*C6;
  FM34_2 = FB34_2+C5*(FB35_2+FB36_2-FM111_2*S11-FM113_2*S13-FM17_2*S7-FM19_2*S9+FM311_2*C11+FM313_2*C13+FM37_2*C7+FM39_2*C9)-S5*(FB15_2+FM16_2*C6...
 -FM26_2*S6);
  CM14_2 = s.l(2,4)*FB34_2-s.l(3,4)*FB24_2+C5*(s.l(2,5)*FB35_2-s.l(3,5)*FB25_2+CM16_2*C6-CM26_2*S6)+S5*(CM36_2+s.l(1,5)*FB25_2-s.l(2,5)*FB15_2);
  FB24_3 = s.m(4)*S4;
  FB34_3 = s.m(4)*C4;
  CM14_3 = s.l(2,4)*FB34_3-s.l(3,4)*FB24_3+C5*(s.l(2,5)*FB35_3-s.l(3,5)*FB25_3+CM16_3*C6-CM26_3*S6)+S5*(CM36_3+s.l(1,5)*FB25_3-s.l(2,5)*FB15_3);
  CM14_4 = s.In(1,4)+s.l(2,4)*s.l(2,4)*s.m(4)+s.l(3,4)*s.l(3,4)*s.m(4)+C5*(s.In(1,5)*C5+s.In(3,5)*S5+s.l(2,5)*FB35_4-s.l(3,5)*FB25_4+CM16_4*C6-...
 CM26_4*S6)+S5*(CM36_4+s.In(9,5)*S5+s.l(1,5)*FB25_4-s.l(2,5)*FB15_4);
  FF33 = -(s.frc(3,3)+s.m(3)*s.g(3)-FF24*S4-FF34*C4);
  FM31_24 = -(FM41_35*S4-FM51_26*C4);
  FM31_34 = FM41_35*C4+FM51_26*S4;
  FM32_34 = FM24_2*S4+FM34_2*C4;
  FM33_3 = s.m(3)+C4*(FB34_3+C5*(FB35_3+FB36_3-FM111_3*S11-FM113_3*S13-FM17_3*S7-FM19_3*S9+FM311_3*C11+FM313_3*C13+FM37_3*C7+FM39_3*C9)-S5*(...
 FB15_3+FM16_3*C6-FM26_3*S6))+S4*(FB24_3+FB25_3+FM16_3*S6+FM26_3*C6);
  FF22 = -(s.frc(2,2)+s.frc(2,3)+s.g(2)*s.m(2)+s.g(2)*s.m(3)-FF24*C4+FF34*S4);
  FM22_2 = s.m(2)+s.m(3)+FM24_2*C4-FM34_2*S4;
  FF11 = -(s.frc(1,1)+s.frc(1,2)+s.frc(1,3)+s.frc(1,4)+s.g(1)*s.m(1)+s.g(1)*s.m(2)+s.g(1)*s.m(3)+s.g(1)*s.m(4)-FF15*C5-FF35*S5);
  FM11_1 = s.m(1)+s.m(2)+s.m(3)+s.m(4)+FM15_1*C5+FM35_1*S5;

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  M(1,1) = FM11_1;
  M(1,2) = FM31_24;
  M(1,3) = FM31_34;
  M(1,4) = CM41_15;
  M(1,5) = CM25_1;
  M(1,6) = CM36_1;
  M(1,7) = CM27_1;
  M(1,8) = CM28_1;
  M(1,9) = CM29_1;
  M(1,10) = CM210_1;
  M(1,11) = CM211_1;
  M(1,12) = CM212_1;
  M(1,13) = CM213_1;
  M(1,14) = CM214_1;
  M(2,1) = FM31_24;
  M(2,2) = FM22_2;
  M(2,3) = FM32_34;
  M(2,4) = CM14_2;
  M(2,5) = CM25_2;
  M(2,6) = CM36_2;
  M(2,7) = CM27_2;
  M(2,8) = CM28_2;
  M(2,9) = CM29_2;
  M(2,10) = CM210_2;
  M(2,11) = CM211_2;
  M(2,12) = CM212_2;
  M(2,13) = CM213_2;
  M(2,14) = CM214_2;
  M(3,1) = FM31_34;
  M(3,2) = FM32_34;
  M(3,3) = FM33_3;
  M(3,4) = CM14_3;
  M(3,5) = CM25_3;
  M(3,6) = CM36_3;
  M(3,7) = CM27_3;
  M(3,8) = CM28_3;
  M(3,9) = CM29_3;
  M(3,10) = CM210_3;
  M(3,11) = CM211_3;
  M(3,12) = CM212_3;
  M(3,13) = CM213_3;
  M(3,14) = CM214_3;
  M(4,1) = CM41_15;
  M(4,2) = CM14_2;
  M(4,3) = CM14_3;
  M(4,4) = CM14_4;
  M(4,5) = CM25_4;
  M(4,6) = CM36_4;
  M(4,7) = CM27_4;
  M(4,8) = CM28_4;
  M(4,9) = CM29_4;
  M(4,10) = CM210_4;
  M(4,11) = CM211_4;
  M(4,12) = CM212_4;
  M(4,13) = CM213_4;
  M(4,14) = CM214_4;
  M(5,1) = CM25_1;
  M(5,2) = CM25_2;
  M(5,3) = CM25_3;
  M(5,4) = CM25_4;
  M(5,5) = CM25_5;
  M(5,6) = CM36_5;
  M(5,7) = CM27_5;
  M(5,8) = CM28_5;
  M(5,9) = CM29_5;
  M(5,10) = CM210_5;
  M(5,11) = CM211_5;
  M(5,12) = CM212_5;
  M(5,13) = CM213_5;
  M(5,14) = CM214_5;
  M(6,1) = CM36_1;
  M(6,2) = CM36_2;
  M(6,3) = CM36_3;
  M(6,4) = CM36_4;
  M(6,5) = CM36_5;
  M(6,6) = CM36_6;
  M(6,7) = CM27_6;
  M(6,8) = CM28_6;
  M(6,9) = CM29_6;
  M(6,10) = CM210_6;
  M(6,11) = CM211_6;
  M(6,12) = CM212_6;
  M(6,13) = CM213_6;
  M(6,14) = CM214_6;
  M(7,1) = CM27_1;
  M(7,2) = CM27_2;
  M(7,3) = CM27_3;
  M(7,4) = CM27_4;
  M(7,5) = CM27_5;
  M(7,6) = CM27_6;
  M(7,7) = CM27_7;
  M(7,8) = CM28_7;
  M(8,1) = CM28_1;
  M(8,2) = CM28_2;
  M(8,3) = CM28_3;
  M(8,4) = CM28_4;
  M(8,5) = CM28_5;
  M(8,6) = CM28_6;
  M(8,7) = CM28_7;
  M(8,8) = CM28_8;
  M(9,1) = CM29_1;
  M(9,2) = CM29_2;
  M(9,3) = CM29_3;
  M(9,4) = CM29_4;
  M(9,5) = CM29_5;
  M(9,6) = CM29_6;
  M(9,9) = CM29_9;
  M(9,10) = CM210_9;
  M(10,1) = CM210_1;
  M(10,2) = CM210_2;
  M(10,3) = CM210_3;
  M(10,4) = CM210_4;
  M(10,5) = CM210_5;
  M(10,6) = CM210_6;
  M(10,9) = CM210_9;
  M(10,10) = CM210_10;
  M(11,1) = CM211_1;
  M(11,2) = CM211_2;
  M(11,3) = CM211_3;
  M(11,4) = CM211_4;
  M(11,5) = CM211_5;
  M(11,6) = CM211_6;
  M(11,11) = CM211_11;
  M(11,12) = CM212_11;
  M(12,1) = CM212_1;
  M(12,2) = CM212_2;
  M(12,3) = CM212_3;
  M(12,4) = CM212_4;
  M(12,5) = CM212_5;
  M(12,6) = CM212_6;
  M(12,11) = CM212_11;
  M(12,12) = CM212_12;
  M(13,1) = CM213_1;
  M(13,2) = CM213_2;
  M(13,3) = CM213_3;
  M(13,4) = CM213_4;
  M(13,5) = CM213_5;
  M(13,6) = CM213_6;
  M(13,13) = CM213_13;
  M(13,14) = CM214_13;
  M(14,1) = CM214_1;
  M(14,2) = CM214_2;
  M(14,3) = CM214_3;
  M(14,4) = CM214_4;
  M(14,5) = CM214_5;
  M(14,6) = CM214_6;
  M(14,13) = CM214_13;
  M(14,14) = CM214_14;
  c(1) = FF11;
  c(2) = FF22;
  c(3) = FF33;
  c(4) = CF14;
  c(5) = CF25;
  c(6) = CF36;
  c(7) = CF27;
  c(8) = CF28;
  c(9) = CF29;
  c(10) = CF210;
  c(11) = CF211;
  c(12) = CF212;
  c(13) = CF213;
  c(14) = CF214;

% ====== END Task 0 ====== 

  

