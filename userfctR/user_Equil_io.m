function [] = user_Equil_io(mbs_data,step,flag)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
% user_Equil_io(mbs_data,step,flag)
%
% mbs_data : multibody data structure
% step :
%   - before the process : 
%           . flag = 'init'
%           . step contains the exact number of max. of allowed N.R. iterations
%   - during the process : 
%           . flag : non-existent 
%           . step : contains the current process step (1,2,3,...)
%
% no return value
% this function uses a global structure called MBS_user

% MBS_user : global user structure
%
% field "curvar" : to compute and store in any user function
%                  the current value of a variable
%                  ex. "curvar.myangle"
% field "resequil" : to store the corresponding variable at each step "step"

global MBS_user MBS_info

if (nargin > 2)         %   process starting
    switch flag
        case 'init'     
            MBS_user.resequil.iter = [];
%             MBS_user.resequil.myangle = [];
        otherwise       %   unused
            ;
    end
else                    %   process running
    MBS_user.resequil.iter(step) = step;
%     MBS_user.resequil.myangle(step) = MBS_user.curvar.myangle;
end
 
return