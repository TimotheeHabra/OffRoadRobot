% -------------------------------------------------------------------------
% Author: Allan Barrea and Nicolas Van der Noot
%
% Created: 08-11-2012
% Last update: 29-10-2013
%
% Compiles the C files to a MEX-file for the project
% Based on 'mbs_make_sf.m' (c) 2008 CEREM, UCL
%
% This file should be executed on the workR repertory of the project !
% -------------------------------------------------------------------------

function mbs_make_sf(compile_all, prjname, debug)

% --- Initialization ---

clear functions; % to free the MEX-file if needed

% OS_type:
%     Windows : 1
%     Linux   : 2
%     Mac OS  : 3
%     Other   : 4 (Solaris...)
if(strcmp(computer,'PCWIN') || strcmp(computer,'PCWIN64'))
    OS_type = 1;
elseif(strcmp(computer,'GLNX86') || strcmp(computer,'GLNXA64'))
    OS_type = 2;
elseif(strcmp(computer,'MACI') || strcmp(computer,'MACI64'))
    OS_type = 3;
else
    OS_type = 4;
end

if(OS_type == 1)
    disp('MBS>> Running MEX-file creation on Windows...');
elseif(OS_type == 2)
    disp('MBS>> Running MEX-file creation on GNU Linux 64 bits...');
elseif(OS_type == 3)
    disp('MBS>> Running MEX-file creation on MacOS X 64 bits...');
else
    disp('MBS>> ERROR - Unsupported OS !!');
    return;
end

% MBSPATHDEF defines the paths for use with MBsysLab routines
%   mbsprjpath : Path to the directory containing your multibody systems projects
%   mbspath    : Path to the directory containing MBsysLab files and directories
mbspathdef;

%define = 'DIRDYNARED';
define = 'ACCELRED';

% --- To avoid overwritting a correct file ---

fname = strcat('mbs_sf_dirdynared_', prjname);

overwrite = 1;
if exist(fname,'file') == 3 && ~overwrite % if there is already a 'fname' MEX-file
    button = questdlg(['Would you like to replace ' fname ' ?'],'File already exist');
    switch button
        case {'No', 'Cancel'}
            disp(['MBS>> SFunction file: ' fname ' not created']);
            return
    end
end

% --- Files and directories definitions ---

common_dir = fullfile(mbspath,'MBsysLab','mbs_simulink','mbs_sourceC');
user_dir = fullfile(mbsprjpath,prjname,'SfunctionsR','src_user','user_files');
simulation_dir = fullfile(mbsprjpath,prjname,'SfunctionsR','src_user','simulation_files');
controller_dir = fullfile(mbsprjpath,prjname,'SfunctionsR','src_user','controller_files');
symbolic_dir = fullfile(mbsprjpath,prjname,'symbolicR');

[simulation_headers, controller_headers, simulation_files, controller_files] = simu_ctrl_files();

project_files = {...
    'LocalDataStruct.c'...
    'mbs_close_loops.c'...
    'mbs_compute_model.c'...
    'mbs_dirdynared.c'...
    'mbs_sf_main.c'...
    'MBSdataStruct.c'...
    'MBSsensorStruct.c'...
    'sf_InitCond.c'...
    'sf_IOPort.c'...
    };

symbolic_files = {...
    ['mbs_cons_hJ_' prjname '.c']...
    ['mbs_cons_jdqd_' prjname '.c']...
    ['mbs_dirdyna_' prjname '.c']...
    ['mbs_extforces_' prjname '.c']...
    ['mbs_gensensor_' prjname '.c']...
    ['mbs_link_' prjname '.c']...
    ['mbs_link3D_' prjname '.c']...
    ['mbs_sensor_' prjname '.c']...
    };

tool_files = {...    
    'choldc.c'...
    'cholsl.c'...
    'lubksb.c'...
    'ludcmp.c'...
    'lut.c'...
    'mbs_matrix.c'...
    'mbs_tool.c'...
    'norm.c'...
    'nrutil.c'...
    'svbksb.c'...
    'svdcmp.c'...
    };

user_headers = {...
    'user_sf_IO.h'...
    'UserModelStruct.h'...
    'userDef.h'...
    };

user_files = {...
    'user_compute_output.c'...
    'user_cons_hJ.c'...
    'user_cons_jdqd.c'...
    'user_Derivative.c'...
    'user_DrivenJoints.c'...
    'user_ExtForces.c'...
    'user_initialization.c'...
    'user_JointForces.c'...
    'user_Link3Dforces.c'...
    'user_Linkforces.c'...
    'user_sf_IO.c'...
    'UserModelStruct.c'...
    };
    
% Checks if global compilation is needed
compile_ctrl = 0;

if compile_all
    new_save_compile();
else
    for i=1:length(controller_headers),    
        if ~strcmp('controller_def.h',controller_headers{i})
            compile_all = need_to_compile_all(fullfile(controller_dir,controller_headers{i}),prjname);
        end
    end

    for i=1:length(simulation_headers),    
        compile_all = need_to_compile_all(fullfile(simulation_dir,simulation_headers{i}),prjname);
    end

    for i=1:length(user_headers),    
        compile_all = need_to_compile_all(fullfile(user_dir,user_headers{i}),prjname);
    end 
end

if ~compile_all
    compile_ctrl = need_to_compile_ctrl(fullfile(controller_dir,'controller_def.h'),prjname);
end

if compile_all && exist('object_files','dir'),
   rmdir('./object_files', 's');
end

if ~exist('object_files','dir'),
    mkdir('object_files');
end

if ~exist('save_compile.mat','file')
    new_save_compile_ok();
end

s = load('save_compile.mat');
files_compiled = s.files_compiled;

% --- Compiling ---

files_compiled = compile_folder( common_dir,     project_files,    OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, 0);
files_compiled = compile_folder( symbolic_dir,   symbolic_files,   OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, 0);
files_compiled = compile_folder( common_dir,     tool_files,       OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, 0);
files_compiled = compile_folder( controller_dir, controller_files, OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, compile_ctrl);
files_compiled = compile_folder( simulation_dir, simulation_files, OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, 0);
compile_folder( user_dir,       user_files,       OS_type, files_compiled, ...
    debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, 0);

% --- Linking ---
disp('MBS>> Linking dirdynared...');

if(OS_type == 1) % Windows
    % Linker command for Windows
    if debug
        eval(['mex -g -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' object_files/*.obj']);
    else
        eval(['mex -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' object_files/*.obj']);
    end
elseif(OS_type == 3) % Mac OS
    % Adapting the linker command to MacOS X 64 bits
    % Beware that the object files have a '.o' extension instead of '.obj'
    % Beware that the 'object_files/*.o' expression doesn't work on Mac
    % so, all the file names have to be given explicitely.
    objfiles = dir('object_files/*.o');
    objfiles = {objfiles.name};
    objfiles = sprintf('object_files/%s ', objfiles{:});
    if debug
        eval(['mex -g -largeArrayDims -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' ' objfiles]);
    else
        eval(['mex -largeArrayDims -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' ' objfiles]);
    end
elseif(OS_type == 2) % Linux
    % Adapting the linker command to GNU Linux 64 bits
    % Beware that the object files have a '.o' extension instead of '.obj'
    % Beware that the 'object_files/*.o' expression doesn't work on Mac
    % so, all the file names have to be given explicitely.
    objfiles = dir('object_files/*.o');
    objfiles = {objfiles.name};
    objfiles = sprintf('object_files/%s ', objfiles{:});
    if debug
        eval(['mex -g -largeArrayDims -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' ' objfiles]);
    else
        eval(['mex -largeArrayDims -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' ' objfiles]);
    end
end

% --- Cleaning ---
%disp('MBS>> Cleaning ''object_files'' directory...');
%rmdir('object_files', 's');

% --- Closing message ---
disp(['MBS>> SFunction file: ' fname ' successfully created']);

if exist('save_compile.mat','file')
    delete('save_compile.mat');
end

end

% Checks if the whole project has to be recompiled
function result = need_to_compile_all(h_file_name, prjname)

result = 0;

mex_file_name = ['./mbs_sf_dirdynared_' prjname '.' mexext];

% If the executable file is not present, then recompile the whole project
if ~exist(mex_file_name, 'file')
    result = 1;
    new_save_compile();
    return;
end

h_file = dir(h_file_name);
mex_file = dir(mex_file_name);

% If one header file is more recent than the executable and if this is not the first
% compilation trial, then recompile the whole project
if h_file.datenum > mex_file.datenum
    if ~exist('save_compile.mat','file')
        result = 1;
        new_save_compile();
        return;
    else 
        s = load('save_compile.mat');
        files_compiled = s.files_compiled;
        if h_file.datenum > files_compiled{1}
            result = 1;
            new_save_compile();
            return;
        end
    end
end

end

% Checks if the whole project has to be recompiled
function result = need_to_compile_ctrl(h_file_name, prjname)

result = 0;

mex_file_name = ['./mbs_sf_dirdynared_' prjname '.' mexext];

% If the executable file is not present, then recompile the whole project
if ~exist(mex_file_name, 'file')
    result = 1;
    new_save_compile();
    return;
end

h_file = dir(h_file_name);
mex_file = dir(mex_file_name);

% If one header file is more recent than the executable and if this is not the first
% compilation trial, then recompile the whole project
if h_file.datenum > mex_file.datenum
    new_save_compile_ok();
    result = 1;
end

end

% This function takes as input the C file name and checks
% whether or not the file has to be recompiled
function result = need_to_be_recompiled(c_file_name, files_compiled, OS_type)

result = 0;

% Generate the correct extension for object files depending on the platform
if(OS_type == 1)
    fext = '.obj';
elseif(OS_type == 3)
    fext = '.o';
elseif(OS_type == 2)
    fext = '.o';
end

% Generate the object file name from the C file name,
% assuming all object files are located in ./object_files
[pathstr, fname, ext] = fileparts(c_file_name);
obj_file_name = fullfile('./object_files', [fname fext]);

% If the object file doesn't exist, then the C file has to be recompiled
if ~exist(obj_file_name, 'file')
    result = 1;
    return;
end

% If the object file already exists, it has to be recompiled iff the C file
% is more recent than the object file.
c_file = dir(c_file_name);
obj_file = dir(obj_file_name);

if c_file.datenum > obj_file.datenum
    result = 1;
    return
end

if files_compiled{1}
    
    found_in_list = 0;
    length_files_compiled = length(files_compiled);
    for i = 1:length_files_compiled
        this_name_list = files_compiled{i};
        if strcmp(this_name_list,c_file_name)
            found_in_list = 1;
            break;
        end
    end

    if found_in_list
        result = 0;
    else
        result = 1;
    end
end

end

function [files_compiled] = compile_folder(dir, files, OS_type, files_compiled, debug, define, symbolic_dir, common_dir, controller_dir, simulation_dir, user_dir, compile_ctrl)

    for i=1:length(files),
        c_file_name = fullfile(dir,files{i});
        if (compile_ctrl) || need_to_be_recompiled(c_file_name, files_compiled, OS_type)
            disp(['MBS>> Compiling ' files{i} '...']);
            
            if(OS_type == 2) % Linux
                if debug
                    eval(['mex -g CFLAGS="-std=c99 -D_GNU_SOURCE  -fexceptions -fPIC -fno-omit-frame-pointer -pthread" -c -D' ...
                        define ' -I''' symbolic_dir ''' -I''' common_dir ''' -I''' controller_dir ''' -I''' simulation_dir ...
                        ''' -I''' user_dir ''' -outdir ''object_files'' ''' fullfile(dir,files{i}) '''']);
                else
                    eval(['mex CFLAGS="-std=c99 -D_GNU_SOURCE  -fexceptions -fPIC -fno-omit-frame-pointer -pthread" -c -D' ...
                        define ' -I''' symbolic_dir ''' -I''' common_dir ''' -I''' controller_dir ''' -I''' simulation_dir ...
                        ''' -I''' user_dir ''' -outdir ''object_files'' ''' fullfile(dir,files{i}) '''']);
                end
            else % Mac OS or Windows
                if debug
                    eval(['mex -g -c -D' define ' -I''' symbolic_dir ''' -I''' common_dir ...
                        ''' -I''' controller_dir ''' -I''' simulation_dir ''' -I''' user_dir ...
                        ''' -outdir ''object_files'' ''' fullfile(dir,files{i}) '''']);
                else
                    eval(['mex -c -D' define ' -I''' symbolic_dir ''' -I''' common_dir ...
                        ''' -I''' controller_dir ''' -I''' simulation_dir ''' -I''' user_dir ...
                        ''' -outdir ''object_files'' ''' fullfile(dir,files{i}) '''']);
                end
            end
        end
        
        found_in_list = 0;
        length_files_compiled = length(files_compiled);
        for j = 1:length_files_compiled
            this_name_list = files_compiled{j};
            if strcmp(this_name_list,c_file_name)
                found_in_list = 1;
                break;
            end
        end

        if ~found_in_list
            files_compiled{length_files_compiled+1} = c_file_name;
            save('save_compile.mat','files_compiled');
        end
    end
end

function new_save_compile

    files_compiled = {};
    save('save_compile.mat','files_compiled');
    save_compile_file = dir('./save_compile.mat');
    files_compiled{1} = save_compile_file.datenum;
    save('save_compile.mat','files_compiled');
end

function new_save_compile_ok

    files_compiled = {};
    files_compiled{1} = 0;
    save('save_compile.mat','files_compiled'); 
end
