% Plot the results generated from bench2.m
clear;

%% Dataset
synthetic_dataset = true;
% plaza_dataset = true;

runs = 1;
if exist('synthetic_dataset', 'var') && synthetic_dataset
    skip = [1,2,4,10,20]-1;    
    folder = '../Experiment/synthetic/ratio';
    gtFile = '../Data/Syn_theta_Plaza.mat'; % file with ground truth
elseif exist('plaza_dataset', 'var') && plaza_dataset
    skip = [1,2,4,10,20]-1;        
    folder = 'Experiment/plaza/ratio';
    gtFile = '../Data/Plaza1_thrown_away_some_range.mat';
end
rnum = size(skip,2);


%% Get data
e_wout = zeros(rnum, 1);
n_wout = zeros(rnum, 1);
t_wout = zeros(rnum, 1);
e_with = zeros(rnum, 1);
n_with = zeros(rnum, 1);
t_with = zeros(rnum, 1);

e_fine = zeros(rnum, 1);
fileName = sprintf('%s/data/ratio_%d_wout_interp', folder, 1);
load(fileName);
XYT_fine = XYT;
load(gtFile);

for i = 1:rnum
    ratio = skip(i) + 1;
    fileName = sprintf('%s/data/ratio_%d_wout_interp', folder, ratio);
    load(fileName);
    err_dist = sqrt(diff(:,1).^2 + diff(:,2).^2);
    e_wout(i) = sqrt(mean((err_dist).^2)); 
    t_wout(i) = step_t(end,2);
    n_wout(i) = nEstStateInds;
    
    fileName = sprintf('%s/data/ratio_%d_with_interp', folder, ratio);
    load(fileName);
    err_dist = sqrt(diff(:,1).^2 + diff(:,2).^2);
    e_with(i) = sqrt(mean((err_dist).^2)); 
    t_with(i) = step_t(end,2);    
    n_with(i) = nEstStateInds;
    
    % RMSE error of fine estimate at the estimated states of current ratio
    estStateInds = estStateInds(1:nEstStateInds);
    diff = XYT_fine(estStateInds+1, 1:2) - GT(estStateInds+1, 2:3);
    err_dist = sqrt(diff(:,1).^2 + diff(:,2).^2);
    e_fine(i) = sqrt(mean((err_dist).^2)); 
end

%% Plot
%% RMSE
figure; hold on;
ht1 = title('RMSE of distance errors of estimated states');
hx1 = xlabel('time step difference between two estimated states');    
hp1 = plot(skip+1, e_wout, 'gs-', skip+1, e_with, 'rd--', skip+1, e_fine, 'bo-.');
grid;
hl1 = legend('without interpolation', 'with interpolation', 'finest est. at estimated states');
if exist('plaza_dataset', 'var') && plaza_dataset
    ylim([0, 5]);
end
hy1 = ylabel('RMSE (m)');
set(hp1, 'LineWidth', 2);
set(gca, 'FontSize', 13);
set([hl1, hx1, hy1, ht1], 'FontSize', 13);


%% Computation time
figure; hold on;
hx2 = xlabel('time step difference between two estimated states');
t_with(1) = t_wout(1);
t_fine = t_wout(1) * ones(size(skip,2),1);
hp2 = plot(skip+1, t_wout, 'gs-', skip+1, t_with, 'rd--', skip+1, t_fine, 'b-.');
hl2 = legend('without interpolation', 'with interpolation', 'finest est.');
hy2 = ylabel('time (sec)');
ht2 = title('Computation Time');  
grid;
set(hp2, 'LineWidth', 2);
set([hl2, hx2, hy2, ht2], 'FontSize', 13);
set(gca, 'FontSize', 13);



