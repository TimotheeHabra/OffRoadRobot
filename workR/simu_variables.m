function [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables()

% Definition of the simulation variables and I/O ports
% For each line: varname , type , size/str_type
% 4 types of simulation variables (simu_vars_none,...)
%       . none = internal variable (type: 'int'/'double')
%       . in = input (type: 'int'/'double')
%       . out = output (type: 'int'/'double')
%       . struct = structure variable (type: structure name with '')
%   - varname = name of the port or of the variable (with '')
%   - size = number of elements in the vector
%               (put '1' for 'simu_vars_struct')
%           (1: simple varibale,  >1: vector -> do not use the 0 index)
            
simu_vars_none = {    
};

simu_vars_in = {
};

simu_vars_out = {
    'tsim_out','double',1
    'output1','double',10
    'output2','double',10
};
    
simu_vars_struct = {
     'cvs','Controller',1
};

end

