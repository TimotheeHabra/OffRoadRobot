%
% Synchronize other versions of the code with the Simulink one
% (files are copied from the Simulink environment)
%
% Developed by Nicolas Van der Noot - 2014
%

clc;
clear all;
close all;

%% Flags

ctrl_flag        = 0; % 1 to synchronize the controller files for the real robot
standalone_flag  = 0; % 1 to synchronize the files of the Standalone version 
                      %(except the xml file and the .txt about the state variables)
stand_state_flag = 0; % 1 to synchronize 'simu_variables.m' and 'control_variables.m'
                      % with the Standalone version
xml_flag         = 1; % 1 to generate the xml file for the Standalone version


%% Path of the original version (Simulink in this script)

controller_files_init = '../SfunctionsR/src_user/controller_files';
simulation_files_init = '../SfunctionsR/src_user/simulation_files';
user_files_init       = '../SfunctionsR/src_user/user_files';
symbolicR_init        = '../symbolicR';

%% Generate the real robot version (controller)

if ctrl_flag
    
    fprintf('\n>>> Copying files for the real robot...\n');

    % copy-paste the folder
    if isdir('../Real_robot_controller');
        rmdir('../Real_robot_controller', 's');
    end

    mkdir('../Real_robot_controller');

    copyfile(controller_files_init, '../Real_robot_controller');

    fprintf('The files have been copied for the real robot\n');
end

%% Synchronize the Standalone version 

if standalone_flag

    % Copying files for the Standalone version
    fprintf('\n>>> Copying files for the Standalone version...\n');

%     if isdir('../Standalone/src/project/controller_files');
%         rmdir('../Standalone/src/project/controller_files', 's');
%     end
% 
%     if isdir('../Standalone/src/project/simulation_files');
%         rmdir('../Standalone/src/project/simulation_files', 's');
%     end
% 
%     if isdir('../Standalone/src/project/user_files');
%         rmdir('../Standalone/src/project/user_files', 's');
%     end
    
    if isdir('../Standalone/src/project/symbolicR');
        rmdir('../Standalone/src/project/symbolicR', 's');
    end

%     mkdir('../Standalone/src/project/controller_files');
%     mkdir('../Standalone/src/project/simulation_files');
%     mkdir('../Standalone/src/project/user_files');
    mkdir('../Standalone/src/project/symbolicR');

%     copyfile(controller_files_init, '../Standalone/src/project/controller_files');
%     copyfile(simulation_files_init, '../Standalone/src/project/simulation_files');
%     copyfile(user_files_init      , '../Standalone/src/project/user_files');
    copyfile(symbolicR_init       , '../Standalone/src/project/symbolicR');
    fprintf('The files have been copied for the Standalone version\n');
end


%% Generate the .txt files used in the Standalone version for the state variables

if stand_state_flag
    
    [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables();  
    [controllers_struct] = control_variables();
    
    addpath('make_generate/');
    
    gen_control_variables_txt(controllers_struct);
    gen_simu_variables_txt(simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct);
    
    rmpath('make_generate/');

end

%% Generate the xml file (.mbsdata) for the Standalone version

if xml_flag
    
    fprintf('\n>>> Creating the xml file for the Standalone version...\n');
   
    addpath('standalone/');

    generate_xml();

    rmpath('standalone/');
end

