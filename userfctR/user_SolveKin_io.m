function [] = user_SolveKin_io(mbs_data,tsim,step,flag)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
% user_SolveKin_io(mbs_data,tsim,step,flag)
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
% MBS_user : global user structure
%
% field "curvar" : to compute and store in any user function
%                  the current value of a variable
%                  ex. "curvar.myangle"
% field "res_solvekin" : to store the corresponding variable at each step "step"
%                     corresponding to the step time "tsim".
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

if (nargin > 3)         %   process starting
    switch flag
        case 'init'
            MBS_user.res_solvekin.tsim = zeros(step,1);
%             MBS_user.res_solvekin.myangle = zeros(step,1);
        otherwise       %   unused
            ;
    end
else                    %   process running
    MBS_user.res_solvekin.tsim(step) = tsim;
%     MBS_user.res_solvekin.myangle(step) = MBS_user.curvar.myangle;
end

return