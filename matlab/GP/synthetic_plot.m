% GT, and TL
datafile = '../Data/Syn_theta_Plaza.mat';
load(datafile);

% Estimate
file = '../Data/Syn_full_data_run.mat';
load(file);

% Plot final result
figure;
hold on;
sz = nEstStateInds;
P = XYT;

% Ground truth
htrue_trajectory = plot(GT(:,2), GT(:,3), 'g', 'LineWidth', 2);
% Estimated states
hestimated_trajectory = plot(P(:,1), P(:,2), 'r-.', 'LineWidth', 2);
% Dead reckoning
hdead = plot(DRp(:,2), DRp(:,3), 'm-.');

% Truth landmarks
htrue_landmarks = plot(TL(:,2), TL(:,3), 'gx', 'LineWidth', 2);
% Estimated landmarks
hestimated_landmarks = plot(XY(:,1), XY(:,2), 'b+', 'LineWidth', 2);

hmap = legend([hdead, htrue_trajectory, hestimated_trajectory, ...
    htrue_landmarks, hestimated_landmarks], 'dead reck. Path', 'true Path', 'est. Path', ...
    'true Landmarks', 'est. Landmarks');
set(hmap,'FontSize',15);
set(gca,'FontSize',13)

xlim([-15 65]);
ylim([-70 10]);
