function [] = user_InvDyn_io(mbs_data,tsim,step,flag)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 01/08/2012
%
% -------------------------
%
% user_InvDyn_io(mbs_data,tsim,step,flag)
%
% mbs_data : multibody data structure
% tsim : current step time
% step :
%   - before the process : 
%           . flag = 'init'
%           . step contains the exact number of requested time steps
%   - during the process : 
%           . flag : non-existent 
%           . step : contains the current process step (1,2,3,...)
%
% no return value
% this function may use a global structure called MBS_user

% MBS_user : global user structure
%
% field "curvar" : to compute and store in any user function
%                  the current value of a variable
%                  ex. "curvar.myangle"
% field "resinvdyn" : to store the corresponding variable at each step "step"
%                     corresponding to the step time "tsim".

global MBS_user MBS_info

if (nargin > 3)         %   process starting
    switch flag
        case 'init'     
            MBS_user.resinvdyn.tsim = zeros(step,1);
%             MBS_user.resinvdyn.myangle = zeros(step,1);

        otherwise       %   unused
            ;
    end
else                    %   process running
    MBS_user.resinvdyn.tsim(step) = tsim;
%     MBS_user.resinvdyn.myangle(step) = MBS_user.curvar.myangle;

end

return