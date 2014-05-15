function Swr = user_Link3DForces(Pos,Rot,Vit,OM,Acc,OMP,mbs_data,tsim,index)
%Swr = user_Link3DForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,il3D)
%
% Pos(3,1) : relative position vector 
% Rot(3,3) : relative rotation matrix
% Vit(3,1) : relative velocity vector 
% OM(3,1)  : relative angular velocity vector
% Acc(3,1) : relative acceleration vector
% OMP(3,1) : relative angular acceleration vector
%
% => All above relative quantities are expressed in the first anchor point frame !
%
% mbs_data : general multibody data structure
% tsim : current time
% index : link index
%
% Swr(6,1) = [Fx; Fy; Fz; Mx; My; Mz];
%   - Fx, Fy, Fz : Force components (expressed in the first anchor point's frame) 
%   - Mx, My, Mz : Pure torque components (expressed in the first anchor point's frame) 
%
% this function may use a global structure called MBS_user
%
%	support@mbsyslab.be

%	MBsysLab - PRM - 2006
%   last revision : 28/08/2006 by Nicolas Docquier function model (beta
%   version)
%   
%
global MBS_user;

Fx=0.0; Fy=0.0; Fz=0.0;
Mx=0.0; My=0.0; Mz=0.0;

%/*-- Begin of user code --*/

    
switch(index)
    case 1  
        % instructions for case 1
        ;  
        
    case 2
        % instructions for case 2
        ;
        
    case 3
        % instructions for case 3
        ; 
    
    %...    

end


%/*-- End of user code --*/

Swr = [Fx; Fy; Fz; Mx; My; Mz];

return
%========================================
