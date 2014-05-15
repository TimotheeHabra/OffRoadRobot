%
% Automates the compilation procedure of the model
%
% Developed by Nicolas Van der Noot and Allan Barrea - 2013
%

%% Initialization
clc;
clear all;
close all;

% compile all files (1) or not (0)
compile_all = 1;
    
% project name
prjname = 'OffRoadRobot';
    
% debug (1 to debug, 0 otherwise)
debug = 0;

%% Gen user_sf_IO.c & user_sf_IO.h if needed

genUserIO = 0;

userIOName = '../SfunctionsR/src_user/user_files/user_sf_IO.c';
simuVarName = './simu_variables.m';

if (compile_all) || (~exist(userIOName, 'file')) || (~exist(simuVarName, 'file'))
    genUserIO = 1;
else
    user_sf_IO_c = dir(userIOName);
    simu_variables_m = dir(simuVarName);

    if (simu_variables_m.datenum > user_sf_IO_c.datenum)   
        genUserIO = 1;
    end
end

if genUserIO   
    fprintf('\n>>> Generating user_IO files...\n');
    
    [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables();
    
    [nb_none, ~] = size(simu_vars_none);
    if nb_none > 0
        simu_vars_none = [simu_vars_none , repmat({'none'},nb_none,1)];
    end

    [nb_in, ~] = size(simu_vars_in);
    if nb_in > 0
        simu_vars_in = [simu_vars_in , repmat({'in'},nb_in,1)];
    end

    [nb_out, ~] = size(simu_vars_out);
    if nb_out > 0
        simu_vars_out = [simu_vars_out , repmat({'out'},nb_out,1)];
    end

    [nb_struct, ~] = size(simu_vars_struct);
    if nb_struct > 0
        simu_vars_struct = [simu_vars_struct , repmat({'structure'},nb_struct,1)];
    end
    
    simu_vars = cat(1,simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct);    
    addpath('make_generate/');
    gen_user_IO(simu_vars, prjname);
    rmpath('make_generate/');
else
    fprintf('\n>>> No need to generate user_IO files...\n');
end

%% Gen ControllerStruct.c & ControllerStruct.h if needed

genCont = 0;

contStructName = '../SfunctionsR/src_user/controller_files/ControllersStruct.c';
contVarName = './control_variables.m';

if (compile_all) || (~exist(contStructName, 'file')) || (~exist(contVarName, 'file'))
    genCont = 1;
else
    ControllersStruct_c = dir(contStructName);
    control_variables_m = dir(contVarName);

    if (control_variables_m.datenum > ControllersStruct_c.datenum)  
        genCont = 1;
    end
end

if genCont   
    fprintf('\n>>> Generating ModelStruct files...\n');
    
    controllers_struct = control_variables();
    addpath('make_generate/');
    gen_controller_struct(controllers_struct);
    rmpath('make_generate/');
else
    fprintf('\n>>> No need to generate ModelStruct files...\n');
end

%% Building MEX file
fprintf('\n>>> Building MEX file...\n');

if genUserIO || genCont
    compile_all = 1;
end

addpath('make_generate/');
mbs_make_sf(compile_all, prjname, debug);
rmpath('make_generate/');

%% Loading project
fprintf('\n>>> Loading project...\n');
[mbs_data, mbs_info] = mbs_load(prjname,'default');
fprintf('\n');

