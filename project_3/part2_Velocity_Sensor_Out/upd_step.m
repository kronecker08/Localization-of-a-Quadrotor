function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %disp(uEst)
   % wRb = eul2rotm(flip(z_t(7:9)),'zyx');
    R_m =@(X)[cos(X(6))*cos(X(5)), cos(X(4))*sin(X(5))*sin(X(6))-sin(X(6))*cos(X(4)), cos(X(6))*sin(X(5))*cos(X(4))+sin(X(6))*sin(X(4));
    sin(X(6))*cos(X(5)), sin(X(6))*sin(X(4))*sin(X(5))+cos(X(6))*cos(X(4)), sin(X(6))*sin(X(5))*cos(X(4))-cos(X(6))*sin(X(4));
    -sin(X(5)), cos(X(5))*sin(X(4)) cos(X(5))*cos(X(4))];
    n = [0.5;0.5;0.5];
    Rt = diag(n);
    H_cr =  [0.7071 -0.7071 0.0000 -0.0400;-0.7071 -0.7071 0.0000 0;0.0000 0.0000 -1.0000 -0.0300;0 0 0 1.0000];
    v = z_t(4:6);
    process = @(X) ((H_cr(1:3,1:3) * R_m(X)'* [X(7);X(8);X(9)]) + ((H_cr(1:3,1:3)*[0 0.03 -0.0283;-0.03 0 -0.0283;0.0283 0.0283 0] *H_cr(1:3,1:3)'*v)));
    n = length(uEst);
    alpha=1e-3;                                
    ki=0;                                      
    beta=2;  
    lambda=alpha^2*(n+ki)-n;
    c=n+lambda; 
    w_0_m= lambda/c;
    w_i_m = 0.5/c; 
    w_i_c = w_i_m;
    w_0_c = w_0_m + (1-alpha^2+beta);
    C = chol(covarEst,'lower');
    u_mat = zeros(n,2*length(C)+1);
    u_mat(:,1) = uEst;
    inc = 1;
for i =1 : length(C)
        u_mat(:,i+inc)= uEst + sqrt(c)*C(:,i);
        u_mat(:,i+inc+1) = uEst - sqrt(c)*C(:,i);
        inc = inc + 1;
    end
    z_mat = zeros(3,length(u_mat));
    for i = 1 : length(u_mat)
        inp = u_mat(:,i);
        z_mat(:,i) = process(inp);
    end
    for i = 1:length(z_mat)
        if i == 1
            z_est = w_0_m * z_mat(:,i);
        else 
            z_est = w_i_m * z_mat(:,i) + z_est;
        end
    end
    for i = 1:length(z_mat)
        if i == 1
            
           ct = w_0_c * (u_mat(:,i) - uEst) * (z_mat(:,i)-z_est)';

        
        else
            
            ct = w_i_c * (u_mat(:,i)- uEst) * (z_mat(:,i)-z_est)' + ct; 
        end
    end
    for i = 1:length(z_mat)
        if i == 1
            st = w_0_c * (z_mat(:,i)-z_est) * (z_mat(:,i)-z_est)';
        else
            st = w_i_c * (z_mat(:,i)-z_est) * (z_mat(:,i)-z_est)' + st;
        end
    end
     st = st + Rt;
    kt = ct * inv(st);
    uCurr = uEst + kt *(z_t(1:3)-z_est);
    covar_curr = covarEst - kt*st*kt';
    %disp(uCurr)

end