%
% Nicolas Van der Noot & Allan Barrea - 09/11/2013
%
% Function to generate an intermediate file containing the elements of the
% mbs_data structure. This intermediate file is then red by the 
% C standalone model to initialize the MBSdata struct.

function [] = MBSdata_extract(mbs_data, filename, pathname)

% -----------------------
% --- Initializations ---
% -----------------------

% Parameters
overwrite = 1;

% ---------------------------
% --- mbs_data extraction ---
% ---------------------------

xmlstr = xml_format(mbs_data, 'on', 'mbsdata');

% ------------------------
% --- Writing xml file ---
% ------------------------

fname = fullfile(pathname,filename);

if exist(fname,'file') && ~overwrite
    button = questdlg(['Would you like to replace ' filename ' ?'],'File already exists');
    switch button
        case {'No', 'Cancel'}
            disp(['File: ' filename ' not created']);
            return
    end
end

fid = fopen(fname,'w');

fprintf(fid, '%s', xmlstr);
    
fclose(fid);

disp(['File: ' filename ' successfully created']);

end

