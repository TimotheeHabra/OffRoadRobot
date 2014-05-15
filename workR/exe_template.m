%--------------------------------------------------------------------------
%   Universit� catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be  
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   MBsysLab main script template:
%      - featuring default options
%      - to be adapted by the user
%
%   Project : $project_name$
%   Author : Team Robotran
%   Date : $Date$ 
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"

% Project loading
prjname = 'OffRoadRobot';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure
                                                                            % Have a look at the content of the mbs_data structure on www.robotran.be

%% 2. Coordinate partitioning [mbs_exe_part]                                % For constrained MBS only
%--------------------------------------------------------------------------
% MBS_user.process = 'part';
% 
% qu_id = mbs_get_joint_id(mbs_info,{'joint1' 'joint4' 'joint6'});            % Get joint indices from joint names
% mbs_data = mbs_set_qu(mbs_data,qu_id);                                      % Set variables [qu_id] as independent 
% 
% qv_id = mbs_get_joint_id(mbs_info,{'joint3' 'joint5'});                     % Get joint indices from joint names
% mbs_data = mbs_set_qv(mbs_data,qv_id);                                      % Set variables [qv_id] as dependent
% 
% qc_id = mbs_get_joint_id(mbs_info,{'joint2'});                              % Get joint indices from joint names
% mbs_data = mbs_set_qc(mbs_data,qc_id);                                      % Set variables [qc_id] as driven
% 
% opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
% % other options : 'visualize', 'clearmbsglobal'                             % Help about options on www.robotran.be
% 
% [mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);                      % Coordinate partitioning process
% 
% % Coordinate partitioning results
% disp('Coordinate partitioning results');
% disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
% disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
% disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
% disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);
% 
% %% 3. Equilibrium [mbs_exe_equil]
% %--------------------------------------------------------------------------
% MBS_user.process = 'equil';
% 
% opt.equil = {'solvemethod','fsolvepk',...
%     'relax',1.0,'itermax',30,'verbose','yes'};
% % other options : 'smooth', 'xeqchoice', 'visualize', 'clearmbsglobal'      % Help about options on www.robotran.be
% %                 'senstol', 'equitol', 'static'
% 
% [mbs_equil,mbs_data] = mbs_exe_equil(mbs_data,opt.equil);                   % Equilibrium process
% 
% % Equilibrium results
% disp('Equilibrium results');
% disp(['XXX1 = ', num2str(mbs_data.q(1))]);                                  % Units : translation [m]
% disp(['XXX2 = ', num2str(mbs_data.q(2))]);                                  %         rotation    [rad]
% 
% %% Saving of the coordinate partitioning and of the equilibrium [mbs_save]
% mbs_save(mbs_info,mbs_data);                                                % Overwrite the original *.mbs (it is recommended to make a copy of the original *.mbs)
%                                                                             % Future equilibrium will be faster (only 1 check iteration)
% mbs_data_ini = mbs_data;                                                    % Backup of the new multibody data structure

%% 4. Kinematics [mbs_exe_solvekin]
%--------------------------------------------------------------------------
% MBS_user.process = 'solvekin';
% 
% mbs_data = mbs_set_qc(mbs_data,mbs_data.qu);                                % Set all independent variables as driven
% 
% opt.solvekin = {'motion','trajectory','time',0:0.01:5,'verbose','yes','framerate',1000};
% % other options : 'visualize', 'save2file', renamefile', clearmbsglobal'    % Help about options on www.robotran.be
% [mbs_solvekin,mbs_data] = mbs_exe_solvekin(mbs_data,opt.solvekin);
% 
% % Kinematics results (sensor kinematics example) [mbs_gensensor_$project_name$]
% Joint_id = mbs_get_joint_id(mbs_info,{'joint3'});
% mbs_data = mbs_set_qa(mbs_data,Joint_id);                                   % Set variables [Joint_id] as actuated 
% gen_sens = [];
% for i=1:length(mbs_solvekin.tsim)                                           % Kinematics of the generic sensor located on joint 'joint3' 
%     t = mbs_solvekin.tsim(i);
%     s.q = mbs_solvekin.q(i,:)'; 
%     s.qd = mbs_solvekin.qd(i,:)'; 
%     s.qdd = mbs_solvekin.qdd(i,:)'; 
%     [sens] = mbs_gensensor_$project_name$(s,t,[],Joint_id);                 % Sensor kinematics process (generic sensor on joint 'joint3')
%     gen_sens = [gen_sens; t sens.P(1) sens.P(2) sens.V(1)];                 % - position: sens.P(i); velocity: sens.V(i); acceleration: sens.A(i)
%                                                                             % - rotation matrix: sens.R(i,j); angular velocity: sens.OM(i)
%                                                                             % - angular acceleration: sens.OMP(i); Jacobian matrix: sens.J(m,n)
% end 
% 
% % Graphical Results
% figure(1);
% plot(mbs_solvekin.tsim,mbs_solvekin.q(:,Joint_id));                         % Joint motion time history : joint 'joint3' motion (example)
% figure(2);
% plot(gen_sens(:,1), gens_sens(:,2));                                        % Sensor motion time history : sensor 'joint3' - x motion (example) 
% pause; 
% close(1); close(2);
% clear gen_sens sens;

%% 5. Direct dynamics [mbs_exe_dirdyn]
%--------------------------------------------------------------------------
MBS_user.process = 'dirdyn';
mbs_data = mbs_set_qu(mbs_data,mbs_data_ini.qu);                            % Retrieving of the initial set of independent variables

opt.dirdyn = {'time',0:0.01:6,'motion','simulation',...
    'odemethod','ode15s','save2file','yes','framerate',1000,...
    'renamefile','no','verbose','yes'};
% other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
%                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be

[mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)

% Graphical Results
% figure(1);
% plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,1));                                    % Joint motion time history : joint n� 1 motion (example)
% pause; 
% close(1);

%% 6. Inverse dynamics [mbs_exe_invdyn]
%--------------------------------------------------------------------------
% MBS_user.process = 'invdyn';
% Joint_id = mbs_get_joint_id(mbs_info,{'joint1' 'joint2' 'joint3'});
% mbs_data = mbs_set_qa(mbs_data,Joint_id);                                   % Set variables [Joint_id] as actuated 
% 
% opt.invdyn = {'motion','trajectory','framerate',1000,...
%     'save2file','yes','renamefile','no','verbose','yes'};
% % other options : 'time', 'visualize', 'clearmbsglobal'                     % Help about options on www.robotran.be
% 
% [mbs_invdyn,mbs_data] = mbs_exe_invdyn(mbs_data,opt.invdyn);                % Inverse dynamics process
% 
% % Graphical Results
% mycolor = ['r' 'b' 'g' 'k'];                                                % Vector of color
% figure(1)
% hold on
% for i=1:mbs_data.nqa
%     plot(mbs_invdyn.tsim,mbs_invdyn.Qact(mbs_data.qa(i)),mycolor(i));       % Actuator force/torque time history
% end
% title('Force or torque XXX'); 
% xlabel('tsim'); ylabel('force [N] or torque [Nm]');
% pause;
% close(1);

%% 7. Modal analysis [mbs_exe_equil ==> mbs_exe_modal]
%--------------------------------------------------------------------------
% MBS_user.process = 'equil';                                                 % Premilinary equilibrium 
% 
% opt.equil = {'solvemethod','fsolvepk',...
%     'relax',1.0,'equitol',1e-7,'itermax',30,'verbose','yes'};
% % other options : 'smooth', 'xeqchoice', 'visualize', 'clearmbsglobal'      % Help about options on www.robotran.be
% %                 'senstol', 'static'
% 
% [mbs_equil,mbs_data] = mbs_exe_equil(mbs_data,opt.equil);                   % Equilibrium process
% %
% 
% MBS_user.process = 'modal';                                                 % Modal analysis
% 
% opt.modal = {'time',0.0,'mode_ampl',0.2...
%     'save2file','yes','renamefile','no','verbose','yes'};
% % other options : 'lintol', 'relincr', 'absincr', 'clearmbsglobal'          % Help about options on www.robotran.be
% %                 'senstol', 'equitol', 'OptionDjief'
% 
% [mbs_modal,mbs_data] = mbs_exe_modal(mbs_data,opt.modal);                   % Modal analysis process
% 
% % Results available in \results and \animation folders
% % ...

%% 8. Closing operations (optional)
%--------------------------------------------------------------------------
mbs_rm_allprjpath;                                                          % Cleaning of the Matlab project paths
mbs_del_glob('MBS_user','MBS_info','MBS_data');                             % Cleaning of the global MBS variables
clc;                                                                        % Cleaning of the Matlab command window