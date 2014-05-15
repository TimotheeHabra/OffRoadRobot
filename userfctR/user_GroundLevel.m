function [level]= user_GroundLevel(px,py,mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[level]= user_GroundLevel(px,py,mbs_data,tsim,iwhl)
%
% Computes the ground level at position px,py
%
% mbs_data : multibody data structure
% tsim : current time step

level=0.0; % flat ground
return