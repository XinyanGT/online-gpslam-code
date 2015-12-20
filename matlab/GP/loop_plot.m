% Odometry
XYT = utilities.extractPose2(odo);
plot(XYT(:,1),XYT(:,2),'y-');


if ~periodic_script
    % linearization point
    lin = isam.getLinearizationPoint();
    XY = utilities.extractPoint2(lin);
    plot(XY(:,1),XY(:,2),'r*');
    sz = nEstStateInds;
    XYT = zeros(sz, 3);
    for ii=1:sz
        XYT(ii,:) = lin.at(symbol('P', estStateInds(ii))).vector()';
    end
    plot(XYT(:,1),XYT(:,2),'r.');
end

% Result
XY = utilities.extractPoint2(result);
plot(XY(:,1),XY(:,2),'k*');
sz = nEstStateInds;
XYT = zeros(sz, 3);
for ii=1:sz
    XYT(ii,:) = result.at(symbol('P', estStateInds(ii))).vector()';
end
plot(XYT(:,1),XYT(:,2),'k-'); 
axis equal