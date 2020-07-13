function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    H_cr = [0.7071 -0.7071 0.0000 -0.0400;-0.7071 -0.7071 0.0000 0;0.0000 0.0000 -1.0000 -0.0300;0 0 0 1.0000];
    psuccess = 0.99;
    k = round(log(1 - psuccess)/log(1- e^3));
    len_optV = length(optV);
    for_h = [R_c2w(:,1),R_c2w(:,2),H_cr(1:3,4)-R_c2w*Z];
    %%for all points
    pw = 0;
    A = zeros(2*length(optV),6);
    v_vec = zeros(2*length(optV),1);
    for i =1 :length(optPos)
        h = for_h\optPos(:,i);
        z = 1/h(3,1);
        row_1 = [-1/z, 0 , optPos(1,i)/z, optPos(1,i)*optPos(2,i), -(1+optPos(1,i)^2), optPos(2,i)];
        row_2 = [0, -1/z, optPos(2,i)/z, 1+optPos(2,i)^2, -optPos(1,i)*optPos(2,i), -optPos(1,i)];
        A(i+pw,:)= row_1;
        A(i+1+pw,:)= row_2;
        v_vec(i+pw) = optV(1,i);
        v_vec(i+pw+1) = optV(2,i);
        pw = pw+1;
    end
    %% for random points 
    
    AC = zeros(6,6);
    vC = zeros(6,1);
    error = zeros(length(optV),1);
    inlier_num = 0;
    dist = 0.01;
for i = 1:k
     r = randperm(len_optV,3);
     
     pwC = 0;
     for j =1:3
        h = for_h\optPos(:,r(j));
        z = 1/h(3,1);
        vC(j+pwC) = optV(1,r(j));
        vC(j+1+pwC) = optV(2,r(j));
        row_1 = [-1/z, 0 , optPos(1,r(j))/z, optPos(1,r(j))*optPos(2,r(j)), -(1+optPos(1,r(j))^2), optPos(2,r(j))];
        row_2 = [0, -1/z, optPos(2,r(j))/z, 1+optPos(2,r(j))^2, -optPos(1,r(j))*optPos(2,r(j)), -optPos(1,r(j))];
        AC(j+pwC,:) = row_1;
        AC(j+1+pwC,:) = row_2;
        pwC = pwC+1;
     end
     x_w  = AC\vC;
     err = A*x_w - v_vec;
     pg = 0;
     c_inlier = 0;
     y =1;
     inlier_id = [];
     for k =1:length(optV)
         error(k) = sqrt(err(k+pg)^2 + err(k+pg+1)^2);
         if error(k)<=dist
            c_inlier = c_inlier + 1;
            inlier_id(y,1) = k;
            y= y+1;
         end
         pg = pg+1;
     end
     %disp(c_inlier)
     if c_inlier > inlier_num
         inlier_num = c_inlier;
         inlier_id_f = inlier_id;
     end
     if inlier_num>= e*length(optV)
         break
     end
end
pwC = 0;
vff = zeros(2*length(inlier_id_f),1);
aff = zeros(2*length(inlier_id_f),6);
for j = 1 : length(inlier_id_f)
    h = for_h\optPos(:,inlier_id_f(j));
    z = 1/h(3,1);
    vff(j+pwC) = optV(1,inlier_id_f(j)); 
    vff(j+1+pwC) = optV(2,inlier_id_f(j));
    row_1 = [-1/z, 0 , optPos(1,inlier_id_f(j))/z, optPos(1,inlier_id_f(j))*optPos(2,inlier_id_f(j)), -(1+optPos(1,inlier_id_f(j))^2), optPos(2,inlier_id_f(j))];
    row_2 = [0, -1/z, optPos(2,inlier_id_f(j))/z, 1+optPos(2,inlier_id_f(j))^2, -optPos(1,inlier_id_f(j))*optPos(2,inlier_id_f(j)), -optPos(1,inlier_id_f(j))];
    aff(j+pwC,:) = row_1;
    aff(j+1+pwC,:) = row_2;
    pwC = pwC+1;
end
Vel = aff\vff;
    
    
     
     
     
        
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end