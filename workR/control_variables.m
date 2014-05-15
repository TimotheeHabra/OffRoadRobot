function [controllers_struct] = control_variables()

% Definition of the control variables
% For each line: varname , type , size/str_type
%  - varname: name of the variable (with '')
%  - type: 'int' / 'double' / 'structure'
%      if type == 'int' / 'double'
%          - size/str_type: number of elements in the vector
%               1    :  simple variable
%               n    :  vector of n (n>1) elements
%               [m n]:  tabular of 2 entries with a size m*n 
%      if type == 'structure' 
%          - size/str_type: the type of the structure (its name with '')

%% controller_1

controller_1_name = 'Controller';
controller_1_vars = {
    % time input
    't' , 'double' ,1
    
    % control of the motors
    'Control' , 'double' ,4
};


%% controllers_vars

controllers_struct = {
    controller_1_name, controller_1_vars;
};

end
