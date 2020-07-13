function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
    id = data(t).id;
%getting corner
res = getCorner(id);
%p1xp1yp2xp2yp3xp3ypxpy
p = [data(t).p1;data(t).p2;data(t).p3;data(t).p4]';
tri = [1 3 5 7];
n = length(p);
%% creating A
A = zeros(2*n,9);
jk = 0;
for i = 1:length(id)
    for t = 1:4
            j = tri(t);
            row_1 = [res(i,j) res(i,j+1) 1 0 0 0 -(res(i,j)*p(i,j)) -(p(i,j)*res(i,j+1)) -(p(i,j))];
            row_2 = [0 0 0 res(i,j) res(i,j+1) 1 -(p(i,j+1)*res(i,j)) -(p(i,j+1)*res(i,j+1)) -(p(i,j+1))];
            A(j+jk,:)= row_1;
            A(j+1+jk,:)= row_2;
    end
        jk = jk+8;
end
%svd A
    [~,~,v] = svd(A);
    %print(v)
    h = v(:,end);
    H = [h(1:3)';h(4:6)';h(7:9)'];
    H = H*sign(H(3,3));
    K = [311.0520 0 201.8724;0 311.3885 113.6210;0 0 1];
    k_inv_h = (K)\H;
    R = [k_inv_h(:,1),k_inv_h(:,2),cross(k_inv_h(:,1),k_inv_h(:,2))];
    [u_r,~,v_r] = svd(R);
    R_f = u_r*[1 0 0;0 1 0;0 0 det(u_r*v_r')]*v_r';
    R_c2w = R_f;
    T = k_inv_h(:,3)/norm(k_inv_h(:,1));
    w_h_c = [[R_f',-(R_f')*T];[0 0 0 1]];
    w_h_r = w_h_c*[0.7071 -0.7071 0.0000 -0.0400;-0.7071 -0.7071 0.0000 0;0.0000 0.0000 -1.0000 -0.0300;0 0 0 1.0000];
    position = w_h_r(1:3,end);
    o = (rotm2eul(w_h_r(1:3,1:3),'ZYX'))';
    orientation = o(end:-1:1);
end
