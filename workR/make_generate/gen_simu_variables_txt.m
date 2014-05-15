function gen_simu_variables_txt(simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct)

    filename = 'simu_variables.txt';
    pathname = '../Standalone/src/project/varState/';
    fname = fullfile(pathname,filename);

    fid = fopen(fname,'w');
    fprintf(fid,[...
                '%% Definition of the simulation variables and I/O ports\r\n'...
                '%% For each line: varname , type , size/str_type\r\n'...
                '%% 4 types of simulation variables (simu_vars_none,...)\r\n'...
                '%%       . NONE   : internal variable (type: int/double)\r\n'...
                '%%       . IN     : input (type: int/double)\r\n'...
                '%%       . OUT    : output (type: int/double)\r\n'...
                '%%       . STRUCT : structure variable (type: structure name without '''' or "")\r\n'...
                '%%   - varname = name of the variable (without '''' or "")\r\n'...
                '%%   - size = number of elements in the vector\r\n'...
                '%%               (put 1 for STRUCT)\r\n'...
                '%%           (1: simple varibale,  >1: vector -> do not use the 0 index -> different from ''control_variables'')\r\n'...
                '%%\r\n'...
                '%% Lines starting with %% or // are not taken into account (comments)\r\n'...
                '%% Write the corresponding variables under # NONE, # IN, # OUT, # STRUCT\r\n'...
                '%% Do modify the lines # NONE, # IN, # OUT, # STRUCT or add another line starting with #\r\n'...
                '%% Keep at least one space between different variables, do not use other signs (, . :)\r\n'...
                '%%\r\n\r\n']);
            
    fprintf(fid,'# NONE\r\n');
    
    size_1 = size(simu_vars_none, 1);
    
    for i = 1:size_1
        fprintf(fid,'%s   %s   %d\r\n', simu_vars_none{i}, simu_vars_none{size_1+i}, simu_vars_none{2*size_1+i});        
    end
    
    fprintf(fid,'\r\n# IN\r\n');
    
    size_1 = size(simu_vars_in, 1);
    
    for i = 1:size_1
        fprintf(fid,'%s   %s   %d\r\n', simu_vars_in{i}, simu_vars_in{size_1+i}, simu_vars_in{2*size_1+i});        
    end
    
    fprintf(fid,'\r\n# OUT\r\n');
    
    size_1 = size(simu_vars_out, 1);
    
    for i = 1:size_1
        fprintf(fid,'%s   %s   %d\r\n', simu_vars_out{i}, simu_vars_out{size_1+i}, simu_vars_out{2*size_1+i});        
    end
    
    fprintf(fid,'\r\n# STRUCT\r\n');
    
    size_1 = size(simu_vars_struct, 1);
    
    for i = 1:size_1
        fprintf(fid,'%s   %s   %d\r\n', simu_vars_struct{i}, simu_vars_struct{size_1+i}, simu_vars_struct{2*size_1+i});        
    end
    
    fprintf(fid,'\r\n');
        
    fclose(fid);

    fprintf('simu_variables.txt created\r\n');

end