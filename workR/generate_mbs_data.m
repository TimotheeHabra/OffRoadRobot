function [ mbs_data ] = generate_mbs_data()

% Generates the mbs_data structure 
% used in 'call_simulink' and in the xml file generation

%% Project loading

global MBS_user;       % Declaration of the global user structure
MBS_user.process = ''; 

prjname = 'OffRoadRobot';      % project name
[mbs_data, mbs_info] = mbs_load(prjname,'default');

% number of constraints 
% -> if there is no constraint, you must comment these lines 
%     ( 'mbs_data.Nuserc = 0;' might not work !!!)
mbs_data.Nuserc = 0;
mbs_data.Ncons  = 0;


%% 2. Coordinate partitioning [mbs_exe_part]             % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';

% driven joints
id_FR = mbs_get_joint_id(mbs_info,'R2_FR');
mbs_data = mbs_set_qdriven(mbs_data,id_FR);

id_FL = mbs_get_joint_id(mbs_info,'R2_FL');
mbs_data = mbs_set_qdriven(mbs_data,id_FL);

id_RR = mbs_get_joint_id(mbs_info,'R2_RR');
mbs_data = mbs_set_qdriven(mbs_data,id_RR);

id_RL = mbs_get_joint_id(mbs_info,'R2_RL');
mbs_data = mbs_set_qdriven(mbs_data,id_RL);

opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);

% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);
disp(' ');

end

