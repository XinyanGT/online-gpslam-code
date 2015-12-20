figure; hold on;
estStateInds = estStateInds(1:nEstStateInds);

% Calculate error rate
sz = nEstStateInds;
XYT = zeros(sz, 3);
XYT_dot = zeros(sz, 3);
for ii=1:sz
    XYT(ii,:) = result.at(symbol('P', estStateInds(ii))).vector()';
    XYT_dot(ii,:) = result.at(symbol('V', estStateInds(ii))).vector()';
end

diff = GT(estStateInds(1:sz)+1,2:3) - XYT(:,1:2); % estStateInds start from 0
diff2 = sqrt(diff(:,1).^2 + diff(:,2).^2);
if to_interpolate
    fprintf('With Interpo, skipped states: %d, ME: %f, # estimated state: %d, total time: %f\n', bufferedVelProj_size, mean(diff2), sz, step_t(M,2));
else
    fprintf('Wout Interpo, skipped states: %d, ME: %f, # estimated state: %d, total time: %f\n', bufferedVelProj_size, mean(diff2), sz, step_t(M,2));
end

P = zeros(sz, 3);
Pdot = zeros(sz, 3);
for i=1:sz
    P(i,:) = result.at(symbol('P', estStateInds(i))).vector()';
    Pdot(i,:) = result.at(symbol('V', estStateInds(i))).vector()';
end

% Interpolation or query between estimated states
XYT_itp = zeros(M+1, 6);
currInd = 1;
for i = 1:sz-1
    X1 = [P(i,:)'; Pdot(i,:)'];
    X2 = [P(i+1,:)'; Pdot(i+1,:)'];
    XYT_itp(currInd, :) = X1';      % first set the start one
    delta_t = estStateInds(i+1) - estStateInds(i);  % the time in between the two estimated states
    for j = 1:delta_t-1             % interpolate from the first state in between to the state before the end state
        tao = j * dt;
        Lambda = calc_Phi(tao) - calc_Q(Qc, tao) * calc_Phi(delta_t-tao)' * calc_Q_inv(Qc, delta_t) * calc_Phi(delta_t);
        Psi = calc_Q(Qc, tao) * calc_Phi(delta_t-tao)' * calc_Q_inv(Qc, delta_t);
        XYT_itp(currInd+j, :) = (Lambda*X1 + Psi*X2)';
    end
    currInd = currInd + delta_t;
end
XYT_itp(end, :) = [P(end,:), Pdot(end,:)];      % set the last one

% Ground truth
htrue_trajectory = plot(GT(:,2), GT(:,3), 'g', 'LineWidth', 2);
% Estimated states
hestimated_trajectory = plot(P(:,1), P(:,2), 'r-.', 'LineWidth', 3);
% Interpolated
hinterpolated = plot(XYT_itp(:,1), XYT_itp(:,2), 'k', 'LineWidth', 2);
% Dead reckoning
hdead = plot(DRp(:,2), DRp(:,3), 'm-.');

% Truth landmarks
if exist('seen_lmkey', 'var')
    ind = find(seen_lmkey);
    htrue_landmarks = plot(TL(ind,2), TL(ind,3), 'mx', 'LineWidth', 2);
else
    htrue_landmarks = plot(TL(:,2), TL(:,3), 'mx', 'LineWidth', 2);
end

% Estimated landmarks
XY = utilities.extractPoint2(result);
hestimated_landmarks = plot(XY(:,1), XY(:,2), 'b+', 'LineWidth', 2);

% Landmarks
% if (~plaza1_dataset)
%     for i = 1:4
%         l = TL(i,:);
%         key = symbol('L', l(1));
%         cov = isam.marginalCovariance(key);
%         plot_ellipse(XY(i,:)', cov, 'm', 3);
%     end
% end

hmap = legend([hdead, htrue_trajectory, hestimated_trajectory, hinterpolated, ...
    htrue_landmarks, hestimated_landmarks], 'dead reckoning Path', 'true Path', 'est. Path', 'interp. Path', ...
    'true Landmarks', 'est. Landmarks');
set(hmap,'FontSize',14);
set(gca,'FontSize',13)


