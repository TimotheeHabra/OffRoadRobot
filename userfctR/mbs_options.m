function [myopt] = mbs_options();

% Coordinate partitioning

myopt.part =                {'verbose','yes','snapshot','no','threshold',1e-8,'rowperm','yes'};

% Direct dynamics - one shot

myopt.dirdyn_oneshot =       {'motion','oneshot','verbose','yes'};

% Direct dynamics - time simulation

myopt.dirdyn_simulation =         {'motion','simulation','verbose','yes','save2file','yes','renamefile','no',...
                            'depinteg','no','odemethod','ode45','time',[0:0.01:1]};

% Inverse dynamics - one shot

myopt.invdyn_oneshot =      {'verbose','yes','motion','oneshot'};

% Inverse dynamics - time simulation

myopt.invdyn_trajectory =   {'verbose','yes','save2file','yes','renamefile','no','snapshot','no',...
                            'motion','trajectory','time',[0:0.01:1]};

% Equilibrium

myopt.equil =               {'verbose','yes','solvemethod','fsolvepk','smooth','yes','itermax',100};
