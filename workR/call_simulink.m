%% Initialization
clc;
close all;
clear all;

%% Generate mbs_data

mbs_data = generate_mbs_data();
mbs_data_closed = mbs_data;

%% Parameters to check

% -- Simulation model parameters [s] -- %
start_time  = 0.0;
finish_time = 10.0;
step_size   = 5.0e-5;

nb_joints = 14;

% q_ini
if(length(mbs_data.q) ~= nb_joints), error('Wrong size for q_ini (expected %d) !!',nb_joints); end

% qd_ini
if(length(mbs_data.qd) ~= nb_joints), error('Wrong size for qd_ini (expected %d) !!',nb_joints); end

% state_in
if(length(mbs_data.qdd) ~= nb_joints), error('Wrong size for qdd_ini (expected %d) !!', nb_joints); end


%% Simulink

model_name = 'dirdyna_OffRoadRobot';

full_model_name = sprintf('%s.mdl', model_name);

open_system(full_model_name);

set_param(model_name, 'Solver','ode4',...
         'StartTime',num2str(start_time),...
         'StopTime',num2str(finish_time),...
         'FixedStep',num2str(step_size));
     
% mbs_data.DonePart=1;

ud =  get_param([model_name '/S-Function_dirdynared'], 'UserData');
ud.mbs_data = mbs_data;
set_param([model_name '/S-Function_dirdynared'], 'UserData', ud);

tic
sim(full_model_name); % launch simulation
toc

%% Outputs

