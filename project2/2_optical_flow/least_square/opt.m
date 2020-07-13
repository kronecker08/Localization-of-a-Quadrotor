%function vEstimated = opt()
%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

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
t1 = sTime(datasetNum);
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
    %for calculating height
    [position, orientation, R_c2w] = estimatePose(sampledData, n-1);
    for_h = [R_c2w(:,1),R_c2w(:,2),H_cr(1:3,4)-R_c2w*position];
    pw = 0;
    A = zeros(2*length(v),6);
    v_vec = zeros(2*length(v),1);
        for i =1 :length(n_p_p)
            h = for_h\n_p_p(:,i);
            z = 1/h(3,1);
            row_1 = [-1/z, 0 , n_p_p(1,i)/z, n_p_p(1,i)*n_p_p(2,i), -(1+n_p_p(1,i)^2), n_p_p(2,i)];
            row_2 = [0, -1/z, n_p_p(2,i)/z, 1+n_p_p(2,i)^2, -n_p_p(1,i)*n_p_p(2,i), -n_p_p(1,i)];
            A(i+pw,:)= row_1;
            A(i+1+pw,:)= row_2;
            v_vec(i+pw) = v(1,i);
            v_vec(i+pw+1) = v(2,i);
            pw = pw+1;
        end
     %for least squares   
    v_w = A\v_vec;
    %disp(v_w)
    X_quadrotor = [(H_cr(1:3,1:3))' -(H_cr(1:3,1:3))'*[0 -H_cr(3,4) H_cr(2,4);H_cr(3,4) 0 -H_cr(1,4);-H_cr(2,4) H_cr(1,4) 0]; zeros(3,3) (H_cr(1:3,1:3))']*v_w;
   % convert to world frame
    ozyx = orientation(end:-1:1);
    R_w_r = eul2rotm(ozyx','ZYX');
    vel0 = R_w_r*X_quadrotor(1:3);
    omg0 = R_w_r(1:3,1:3)*X_quadrotor(4:6);
    vEstimated(:,n) = [vel0;omg0];  
end
vEstFilt = filter_v(vEstimated,datasetNum);

plotData(vEstFilt, sampledData, sampledVicon, sampledTime, datasetNum)