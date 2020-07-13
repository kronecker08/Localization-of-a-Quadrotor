clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
%% How do you want to do this version
Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
for i = 1:length(sampledTime)
    dt = sampledData(i).t -  prevTime;
    prevTime = sampledData(i).t;
    acc = sampledData(i).acc;
    angVel = sampledData(i).omg;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    z_t = Z(:,i);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(1:15,i) = uCurr;
    uPrev = uCurr;
    covarPrev = covar_curr;
end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);