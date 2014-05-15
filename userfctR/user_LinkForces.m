function Flink = user_LinkForces(Z,Zd,mbs_data,tsim,ilnk)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
% Flink = user_LinkForces(Z,Zd,mbs_data,tsim,ilnk)
%
% Z : position of link-body 2 with respect to link-body 1
% Zd : velocity of link-body 2 with respect to link-body 1
% NB :  Z and Zd are automatically computed in the symbolic file
%       associated with the links that the user has introduced in its MBS
% mbs_data : multibody data structure
% tsim : current time
% ilnk : link index (can be obtained via the 'mbs_get_link_id' function)
%
% Flink : force applied to link-body 1 from link-body 2 (link "ilnk")
%   NB :
%     - For a spring/damper system, the Flink force has the SAME sign as Z, Zd
%       (contrary to joint forces) and thus : Flink = + ... Z + ... Zd
%     - The reaction "-Flink" is automatically taken into accounbt by MBsyslab
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Flink = 0;

%/*-- Begin of user code --*/
% Use the 'mbs_get_link_id' function to get easily the link indices, e.g. :
% L1 = mbs_get_link_id(MBS_info,'myLink_1');
% [L2,L3] = mbs_get_link_id(MBS_info,'myLink_2','myLink_3');
%
switch(ilnk)
    case L1
        % instructions for case 1
        %         e.g. for the user model 'mysuspension' in MBsysPad :
        %         STIFF = mbs_data.user_model.mysuspension.K;
        %         DAMP  = mbs_data.user_model.mysuspension.C;
        %         Z0    = mbs_data.user_model.mysuspension.z0;
        %         % user model constitutive equation
        %         Fspring = STIFF*(Z-Z0);
        %         Fdamper = DAMP*Zd;
        %
        %         Flink=Fspring+Fdamper;
        %
    case L2
        % instructions for case 2, if any
        %
    case L3
        % instructions for case 3, if any
end

%/*-- End of user code --*/

return