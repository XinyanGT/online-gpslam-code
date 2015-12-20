% XYT_dead
datafile = '../Data/victoria_park_deadreckoning.mat';
load(datafile);

% Estimate
file = '../Data/victoria_park_full_data_run';
load(file);

% Plot final result
figure; hold on;
sz = nEstStateInds;
P = XYT;

% Estimated states
hestimated_trajectory = plot(P(:,1), P(:,2), 'r-.', 'LineWidth', 2);
% Dead reckoning
hdead = plot(XYT_dead(:,1), XYT_dead(:,2), 'm-.');

% Estimated landmarks
hestimated_landmarks = plot(XY(:,1), XY(:,2), 'b+', 'LineWidth', 2);

hmap = legend([hdead, hestimated_trajectory, ...
    hestimated_landmarks], 'dead reck. Path', 'est. Path', ...
    'est. Landmarks');
set(hmap,'FontSize',14);
set(gca,'FontSize',13)
xlim([-240, 200]);
