% Benchmarking
% Please comment out "clear", and some parameters setting lines in the
% corresponding matlab script that are called

clear;

%% Dataset
synthetic_dataset = true;
% plaza_dataset = true;

runs = 1;
stateMarginal = 0;
landmarkMarginal = 0;
nNonInterpRangeMeasurements = 'N/A';

if exist('synthetic_dataset', 'var') && synthetic_dataset
    skip = [20,10,4,2,1] - 1;
    folder = '../Experiment/synthetic/ratio';
    exec_name = 'synthetic_periodic';
elseif exist('plaza_dataset', 'var') && plaza_dataset
    skip = [20,10,4,2,1] - 1;    
    folder = 'Experiment/plaza/ratio';
    exec_name = 'plaza_periodic';
end
    
% Create folder if necessary
if ~exist(folderName, 'dir')
    mkdir(foldername);
end

%% Run
rnum = size(skip,2);
incK = 1;
for outer_i = 1:runs
    for inner_i = 1:rnum
        bufferedVelProj_size = skip(inner_i);
        ratio = bufferedVelProj_size + 1;

        %% With interpolation
        to_interpolate = true;
        eval(exec_name); 

        fileName = sprintf('%s/data/ratio_%d_with_interp', folder, ratio);
        figName = sprintf('%s/figs/ratio_%d_with_interp', folder, ratio);
        save(fileName, 'bufferedVelProj_size', 'XYT', 'XYT_dot', 'stateMarginal', ...
            'landmarkMarginal', 'XY', 'step_t', 'nEstStateInds', 'estStateInds', 'diff', ...
            'nNonInterpRangeMeasurements');
        savefig(figName);
        close all;        

        %% Without interpolation
        to_interpolate = false;
        eval(exec_name); 

        fileName = sprintf('%s/data/ratio_%d_wout_interp', folder, ratio);
        figName = sprintf('%s/figs/ratio_%d_wout_interp', folder, ratio);
        save(fileName, 'bufferedVelProj_size', 'XYT', 'XYT_dot', 'stateMarginal', ...
              'landmarkMarginal', 'XY', 'step_t', 'nEstStateInds', 'estStateInds', 'diff', ...
              'nNonInterpRangeMeasurements');
        savefig(figName);
        close all;
    end
end

