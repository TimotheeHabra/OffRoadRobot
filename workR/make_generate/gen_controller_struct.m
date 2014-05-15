function gen_controller_struct(controllers_struct)

[nb_controllers, ~] = size(controllers_struct);


%% ControllerStruct.c

filename = 'ControllersStruct.c';
pathname = '../SfunctionsR/src_user/controller_files';
fname = fullfile(pathname,filename);

fid = fopen(fname,'w');
fprintf(fid,[...
'//---------------------------\r\n' ...
'// Nicolas Van der Noot\r\n' ...
'//\r\n' ...
'// Creation : 19-Sep-2013\r\n' ...
'// Last update : ' date '\r\n' ...
'//---------------------------\r\n\r\n' ...
'#include <stdlib.h>\r\n\r\n' ...
'#include "ControllersStruct.h"\r\n' ...
'\r\n\r\n']);

fprintf(fid,[...
'// ---- Controlleres initialization ---- //\r\n' ...
'\r\n']);

% init
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name 'Struc\r\n' ...
    '' control_name 'Struct * init_' control_name 'Struct(void)\r\n' ...
    '{\r\n' ...
    '    ' control_name 'Struct *cvs;\r\n\r\n']);
    
    found_sigle_tab = 0;
    found_multi_tab = 0;
    for i = 1:nb_variables        
        if ~strcmp(control_vars{i,2},'structure')
            if length(control_vars{i,3}) > 1
                found_multi_tab = 1;
                break;
            elseif (length(control_vars{i,3}) == 1) && (control_vars{i,3} > 1)
                found_sigle_tab = 1;
            end
            
            
        end
    end

    if found_multi_tab
        fprintf(fid,'    int i, j;\r\n\r\n');
    elseif found_sigle_tab
        fprintf(fid,'    int i;\r\n\r\n');
    end
    
    fprintf(fid,[...
    '    cvs = (' control_name 'Struct*) malloc(sizeof(' control_name 'Struct));\r\n' ... 
    '\r\n']);

    for i = 1:nb_variables
        
        if strcmp(control_vars{i,2},'structure')
            this_struct_instance_name = control_vars{i,1};
            this_struct_name          = control_vars{i,3};       
            
            fprintf(fid,['    cvs->' this_struct_instance_name ' = init_' this_struct_name 'Struct();\r\n\r\n']); 
                
        else
        
            switch length(control_vars{i,3})
                case 1
                    if control_vars{i,3} == 1
                        if strcmp(control_vars{i,2},'int')
                            fprintf(fid,['    cvs->' control_vars{i,1} ' = 0;\r\n\r\n']); 
                        else
                            fprintf(fid,['    cvs->' control_vars{i,1} ' = 0.0;\r\n\r\n']);
                        end
                    else
                        if strcmp(control_vars{i,2},'int')
                            fprintf(fid,['    for (i=0;i<' num2str(control_vars{i,3}) ';i++)\r\n' ...
                                         '    {\r\n' ...
                                         '		cvs->' control_vars{i,1} '[i] = 0;\r\n' ...
                                         '    }\r\n\r\n']);
                        else
                            fprintf(fid,['    for (i=0;i<' num2str(control_vars{i,3}) ';i++)\r\n' ...
                                         '    {\r\n' ...
                                         '		cvs->' control_vars{i,1} '[i] = 0.0;\r\n' ...
                                         '    }\r\n\r\n']);
                        end
                    end

                case 2
                    tab_size = control_vars{i,3};
                    if strcmp(control_vars{i,2},'int')
                        fprintf(fid,['    for (i=0;i<' num2str(tab_size(1)) ';i++)\r\n' ...
                                     '    {\r\n' ...
                                     '      for (j=0;j<' num2str(tab_size(2)) ';j++)\r\n' ...
                                     '      {\r\n' ...
                                     '		  cvs->' control_vars{i,1} '[i][j] = 0;\r\n' ...
                                     '      }\r\n'...
                                     '    }\r\n\r\n']);
                    else
                        fprintf(fid,['    for (i=0;i<' num2str(tab_size(1)) ';i++)\r\n' ...
                                     '    {\r\n' ...
                                     '      for (j=0;j<' num2str(tab_size(2)) ';j++)\r\n' ...
                                     '      {\r\n' ...
                                     '		  cvs->' control_vars{i,1} '[i][j] = 0.0;\r\n' ...
                                     '      }\r\n'...
                                     '    }\r\n\r\n']);
                    end

                otherwise
            end
        end
    end

    fprintf(fid,[...
    '    return cvs;\r\n' ...
    '}\r\n\r\n']);

end

fprintf(fid,[...
'// ---- Controllers: free ---- //\r\n' ...
'\r\n']);

% free
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name 'Struc\r\n' ...
    'void free_' control_name 'Struct(' control_name 'Struct *cvs)\r\n' ...
    '{\r\n']);

    for i = 1:nb_variables
        if strcmp(control_vars{i,2},'structure')
            this_struct_instance_name = control_vars{i,1};
            this_struct_name          = control_vars{i,3};
            fprintf(fid,['    free_' this_struct_name 'Struct(cvs->' this_struct_instance_name ');\r\n\r\n']);            
        end        
    end

    fprintf(fid,[...
    '    free(cvs);\r\n' ...
    '}\r\n']);
end

fclose(fid);

fprintf('ControllerStruct.c created\r\n');

%% ControllerStruct.h

filename = 'ControllersStruct.h';
pathname = '../SfunctionsR/src_user/controller_files';
fname = fullfile(pathname,filename);

fid = fopen(fname,'w');
fprintf(fid,[...
'//---------------------------\r\n' ...
'// Nicolas Van der Noot\r\n' ...
'//\r\n' ...
'// Creation : 19-Sep-2013\r\n' ...
'// Last update : ' date '\r\n' ...
'//---------------------------\r\n\r\n' ...
'#ifndef ControllerStruct_h\r\n' ...
'#define ControllerStruct_h\r\n' ...
'\r\n\r\n']);

fprintf(fid,[...
'// ---- Structures definitions (typedef) ---- //\r\n' ...
'\r\n']);

% typedef
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name 'Struc\r\n' ...
    'typedef struct ' control_name 'Struct\r\n' ...
    '{\r\n']);

    for i = 1:nb_variables
        
        if strcmp(control_vars{i,2},'structure')            
            this_struct_instance_name = control_vars{i,1};
            this_struct_name          = control_vars{i,3};            
            fprintf(fid,['    ' this_struct_name 'Struct *' this_struct_instance_name ';\r\n']);
            
        else            
            switch length(control_vars{i,3})
                case 1
                    if control_vars{i,3} == 1
                        fprintf(fid,['    ' control_vars{i,2} ' ' control_vars{i,1} ';\r\n']);
                    else
                        fprintf(fid,['    ' control_vars{i,2} ' ' control_vars{i,1} '[' num2str(control_vars{i,3}) '];\r\n']);
                    end
                case 2
                    tab_size = control_vars{i,3};
                    fprintf(fid,['    ' control_vars{i,2} ' ' control_vars{i,1} '[' num2str(tab_size(1)) '][' num2str(tab_size(2)) '];\r\n']);
                otherwise
            end
        end
    end

    fprintf(fid,[...
    '\r\n} ' control_name 'Struct;\r\n\r\n' ...
    '\r\n']);
end

fprintf(fid,[...
'// ---- Init and free functions: declarations ---- //\r\n' ...
'\r\n']);

% functions declaration
for k = 1:nb_controllers 
    control_name = controllers_struct{k,1};
    fprintf(fid,[control_name 'Struct * init_' control_name 'Struct(void);\r\n']);
    fprintf(fid,['void free_' control_name 'Struct(' control_name 'Struct *cvs);\r\n\r\n']);
end

fprintf(fid,[...
'/*--------------------*/\r\n' ...
'#endif\r\n' ...
'\r\n']);

fclose(fid);

fprintf('ControllerStruct.h created\r\n');

end