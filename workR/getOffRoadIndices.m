%% Initialization
clc;
close all;
clear all;

%% Generate mbs_data and mbs_info

prjname = 'OffRoadRobot';      % project name
[mbs_data, mbs_info] = mbs_load(prjname,'default');
fprintf('\n');

%% S-Sensors


id_S_FR_Sensor = mbs_get_S_sensor_id(mbs_info,'FR_Sensor');
id_S_FL_Sensor = mbs_get_S_sensor_id(mbs_info,'FL_Sensor');
id_S_RL_Sensor = mbs_get_S_sensor_id(mbs_info,'RL_Sensor');
id_S_RR_Sensor = mbs_get_S_sensor_id(mbs_info,'RR_Sensor');


fprintf('// S sensors\n');
fprintf('#define S_FR_Sensor %d\n', id_S_FR_Sensor);
fprintf('#define S_FL_Sensor %d\n', id_S_FL_Sensor);
fprintf('#define S_RL_Sensor %d\n', id_S_RL_Sensor);
fprintf('#define S_RR_Sensor %d\n', id_S_RR_Sensor);

fprintf('\n');
%% F-Sensors


id_F_FR_Sensor = mbs_get_F_sensor_id(mbs_info,'FR_Leg_Force');
id_F_FL_Sensor = mbs_get_F_sensor_id(mbs_info,'FL_Leg_Force');
id_F_RL_Sensor = mbs_get_F_sensor_id(mbs_info,'RL_Leg_Force');
id_F_RR_Sensor = mbs_get_F_sensor_id(mbs_info,'RR_Leg_Force');


fprintf('// F sensors\n');
fprintf('#define F_FR_Sensor %d\n', id_F_FR_Sensor);
fprintf('#define F_FL_Sensor %d\n', id_F_FL_Sensor);
fprintf('#define F_RL_Sensor %d\n', id_F_RL_Sensor);
fprintf('#define F_RR_Sensor %d\n', id_F_RR_Sensor);

fprintf('\n');
