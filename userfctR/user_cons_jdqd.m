function [Jdqd] = user_cons_jdqd(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[Jdqd] = user_cons_jdqd(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim     : current time
%
% Jdqd(q,qd) : vector of the user constraints second derivative cross terms
%
%   d(h)/dt = hd(q,qd) = Jac*qd
%   d2(h)/dt2 = hdd(q,qd,qdd) = J*qdd + Jdqd

global MBS_user MBS_info

q   = mbs_data.q;
qd  = mbs_data.qd;

Nuc = mbs_data.Nuserc;

Jdqd = zeros(Nuc,1);

%/*-- Begin of user code --*/

% Jdqd(1) = 0;
% Jdqd(2) = 
% ...

%/*-- End of user code --*/

return