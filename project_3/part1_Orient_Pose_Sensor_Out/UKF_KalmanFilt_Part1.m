clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
position = proj2Data.position;
angle = proj2Data.angle;
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
for i = 1:length(sampledTime)
    %% Fill in the FOR LOOP
    dt = sampledData(i).t -  prevTime;
    prevTime = sampledData(i).t;
    acc = sampledData(i).acc;
    angVel = sampledData(i).omg;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    z_t = [position(i,:),angle(i,:)]';
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(1:15,i) = uCurr;
    uPrev = uCurr;
    covarPrev = covar_curr;
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);