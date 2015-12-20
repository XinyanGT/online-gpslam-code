% Benchmarking 
% Please comment out "clear", and some parameters setting lines in the
% corresponding matlab script that are called

clear;

%% Dataset
synthetic_dataset = true;
% plaza_dataset = true;
% victoria_dataset = true;

if exist('synthetic_dataset', 'var') && synthetic_dataset
    folderName = '../Experiment/synthetic/timing';
    exec_name_isam2 = 'synthetic isam2';
    exec_name_periodic = 'synthetic_periodic';
elseif exist('plaza_dataset', 'var') && plaza_dataset
    folderName = '../Experiment/plaza/timing';
    exec_name_isam2 = 'plaza isam2';
    exec_name_periodic = 'plaza_periodic';    
elseif exist('victoria_park_dataset', 'var') && victoria_park_dataset
    folderName = '../Experiment/victoria_park/timing';
    exec_name_isam2 = 'victoria_park isam2';
    exec_name_periodic = 'victoria_park_periodic';    
end

% Create folder if necessary
if ~exist(folderName, 'dir')
    mkdir(foldername);
end


%% Run
incK_arr = [10, 1]; % minimum number of range measurements to process after each update
% incK_arr = [20, 10, 1]; 
for incK_ind = 1:size(incK_arr,2)
    incK = incK_arr(incK_ind);
    
    % BTGP
    eval(exec_name_bt);
    fileName = sprintf('%s/BTGP_per_%d_landmark', folderName, incK);
    save(fileName, 'bufferedVelProj_size', 'XYT', 'XYT_dot', 'XY', 'step_t', 'nEstStateInds', 'estStateInds', 'diff');
    figName = sprintf('%s/figs/BTGP_per_%d_landmark', folderName, incK);
    savefig(figName);
    
    % XL_ordering
    XL_ordering = true;
    eval(exec_name_periodic);
    fileName = sprintf('%s/PB_per_%d_landmark', folderName, incK);
    save(fileName, 'bufferedVelProj_size', 'XYT', 'XYT_dot', 'XY', 'step_t', 'nEstStateInds', 'estStateInds', 'diff');
    figName = sprintf('%s/figs/PB_per_%d_landmark', folderName, incK);
    savefig(figName);
    
    % reordering
    XL_ordering = false;
    eval(exec_name_periodic);
    fileName = sprintf('%s/PBVR_per_%d_landmark', folderName, incK);
    save(fileName, 'bufferedVelProj_size', 'XYT', 'XYT_dot', 'XY', 'XYT_dead', 'step_t', 'nEstStateInds', 'estStateInds', 'diff');
    figName = sprintf('%s/figs/PBVR_per_%d_landmark', folderName, incK);
    savefig(figName); 
end

