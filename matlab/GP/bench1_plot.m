% Plot the results generated from bench1.m
clear;

%% Dataset
synthetic_dataset = true;
% plaza_dataset = true;
% victoria_dataset = true;

if exist('synthetic_dataset', 'var') && synthetic_dataset
    folderName = '../Experiment/synthetic/timing';
elseif exist('plaza_dataset', 'var') && plaza_dataset
    folderName = '../Experiment/plaza/timing';
elseif exist('victoria_park_dataset', 'var') && victoria_park_dataset
    folderName = '../Experiment/victoria_park/timing';
end

%% GP with Bayes tree (iSAM2)
fileName = sprintf('%s/BTGP_per_1_landmark', folderName);
load(fileName);
btgp_1 = step_t;

fileName = sprintf('%s/BTGP_per_10_landmark', folderName);
load(fileName);
btgp_10 = step_t;

% fileName = sprintf('%s/BTGP_per_20_landmark', folderName);
% load(fileName);
% btgp_20 = step_t;


%% Periodic batch with variable reordering
fileName = sprintf('%s/PBVR_per_1_landmark', folderName);
load(fileName);
reorder_1 = step_t;

fileName = sprintf('%s/PBVR_per_10_landmark', folderName);
load(fileName);
reorder_10 = step_t;

% fileName = sprintf('%s/PBVR_per_20_landmark', folderName);
% load(fileName);
% reorder_20 = step_t;

%% Periodic batch
fileName = sprintf('%s/PB_per_1_landmark', folderName);
load(fileName);
xlorder_1 = step_t;

fileName = sprintf('%s/PB_per_10_landmark', folderName);
load(fileName);
xlorder_10 = step_t;

% fileName = sprintf('%s/PB_per_20_landmark', folderName);
% load(fileName);
% xlorder_20 = step_t;

%% Plot
temp = 1:size(step_t,1);
figure;
hp = plot(temp, xlorder_1(:,1), '-.', temp, reorder_1(:,1), ':', temp, btgp_1(:,1), '-');
%           temp, btgp_20(:,1),'-.',temp, reorder_20(:,1),'--', temp, xlorder_20(:,1),':');
      
set(hp,'LineWidth',2);
hx = xlabel('time step'); hy = ylabel('time (sec)');
ht = title('Computation Time of Each Step');
hl = legend('PB /1', 'PBVR /1', 'BTGP /1');
grid;
if exist('synthetic_dataset', 'var') && synthetic_dataset
    ylim([0 0.2])    
elseif exist('plaza_dataset', 'var') && plaza_dataset
    ylim([0 0.6])    
elseif exist('victoria_park_dataset', 'var') && victoria_park_dataset
    ylim([0 1.5])    
end

set([hx, hy, ht, hl], 'FontSize',13);
set(gca,'FontSize',13);


figure;
hp = plot(temp, xlorder_1(:,2), '-.', temp, reorder_1(:,2), ':', temp, btgp_1(:,2), '-', ...
          temp, xlorder_10(:,2),':',  temp, reorder_10(:,2),'--', temp, btgp_10(:,2),'-');
set(hp,'LineWidth',2);
hx = xlabel('time step'); hy = ylabel('time (sec)');
ht = title('Accumulated Computation Time');
hl = legend('PB /1', 'PBVR /1', 'BTGP /1', 'PB /10', 'PBVR /10', 'BTGP /10');
grid;
if exist('synthetic_dataset', 'var') && synthetic_dataset
    ylim([0 20])
elseif exist('plaza_dataset', 'var') && plaza_dataset
    ylim([0 150])
elseif exist('victoria_park_dataset', 'var') && victoria_park_dataset
    ylim([0 350])
end

set([hx, hy, ht, hl], 'FontSize',13);
set(gca,'FontSize',13);


