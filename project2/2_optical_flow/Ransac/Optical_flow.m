
close all;
clear all;
clc;

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1 ;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

K = [311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];
H_cr = [0.7071 -0.7071 0.0000 -0.0400;-0.7071 -0.7071 0.0000 0;0.0000 0.0000 -1.0000 -0.0300;0 0 0 1.0000];
%vEstimated =zeros(6,length(sampledTime));
%% intializing the tracker
p_ts = detectFASTFeatures(sampledData(1).img);
p_ts = p_ts.selectStrongest(50);
p_ts = p_ts.Location;
PointTracker = vision.PointTracker;
%% Low pass on time 
sTime = lowpasstime(datasetNum);
initialize(PointTracker,p_ts,sampledData(1).img);
t1 = sTime(1);
%% Initalize Loop load images
vEstimated = zeros(6,length(sampledData));
for n = 2:length(sampledData)
   % disp(n)
    t_prev = t1;
    t_diff = sTime(n)-t_prev;
    t1 = sTime(n);
    points_prev = p_ts;
    [points,~,~] = PointTracker(sampledData(n).img);
    %% new tracker
    p_ts = detectFASTFeatures(sampledData(n).img);
    p_ts = p_ts.selectStrongest(50);
    p_ts = p_ts.Location;
    PointTracker = vision.PointTracker;
    initialize(PointTracker,p_ts,sampledData(n).img);
    %% calculating velocity
    % normalizing the coordinates
    n_p_p = K\[points_prev(:,1)';points_prev(:,2)';ones(1,length(points_prev))];
    n_p = K\[points(:,1)';points(:,2)';ones(1,length(points))];
    v = (n_p - n_p_p)./t_diff;
    [position, orientation, R_c2w] = estimatePose(sampledData, n-1);
    e = 0.6;
    v_w = velocityRANSAC(v,n_p_p,position,R_c2w,e);
    X_quadrotor = [(H_cr(1:3,1:3))' -(H_cr(1:3,1:3))'*[0 -H_cr(3,4) H_cr(2,4);H_cr(3,4) 0 -H_cr(1,4);-H_cr(2,4) H_cr(1,4) 0]; zeros(3,3) (H_cr(1:3,1:3))']*v_w;
   % convert to world frame
    ozyx = orientation(end:-1:1);
    R_w_r = eul2rotm(ozyx','ZYX');
    vel0 = R_w_r*X_quadrotor(1:3);
    omg0 = R_w_r*X_quadrotor(4:6);
    vEstimated(:,n) = [vel0;omg0];  
end
vEstFilt = filter_v(vEstimated,datasetNum);
plotData(vEstFilt, sampledData, sampledVicon, sampledTime, datasetNum)