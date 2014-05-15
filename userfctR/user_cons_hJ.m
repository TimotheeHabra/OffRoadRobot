function [h,Jac] = user_cons_hJ(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[h,Jac] = user_cons_hJ(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim     : current time
%
% h   : vector of user constraints residue 
% Jac : Jacobian matrix of the user constraints
%   note :
%   d(h)/dt = hd(q,qd) = Jac*qd
%   d2(h)/dt2 = hdd(q,qd,qdd) = J*qdd + Jdqd

global MBS_user MBS_info

q   = mbs_data.q;

Nuc = mbs_data.Nuserc;
Njoint = mbs_data.Njoint;

h   = zeros(Nuc,1);
Jac = zeros(Nuc,Njoint);

%/*-- Begin of user code --*/

% h(1) = ...;     % 
% Jac(1,1) = ...;
% Jac(1,2) = ...;
% ...

% h(2) = 
% Jac(2,1) =
% Jac(2,2) =
% ...

%/*-- End of user code --*/

return