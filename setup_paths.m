disp('Initializing air traffic control')
cur = fileparts(mfilename('fullpath'));

if isempty(cur)
    cur = pwd;
end
disp('adding third-party packages');
addpath(genpath(fullfile(cur,'third_party','ipopt')));

disp('Initialized')