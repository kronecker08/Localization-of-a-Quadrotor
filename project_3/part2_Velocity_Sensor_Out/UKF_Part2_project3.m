
% Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
%% How do you want to do this version
camLinVel = proj2Data.linearVel;
camAngVel = proj2Data.angVel;

% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime));
prevTime = 0; 


%% Calculate Kalmann Filter
for  i = 1:length(sampledTime)
    dt = sampledData(i).t -  prevTime;
    prevTime = sampledData(i).t;
    acc = sampledData(i).acc;
    angVel = sampledData(i).omg;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    z_t = [camLinVel(i,:),camAngVel(i,:)]';
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(1:15,i) = uCurr;
    uPrev = uCurr;
    covarPrev = covar_curr;
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);