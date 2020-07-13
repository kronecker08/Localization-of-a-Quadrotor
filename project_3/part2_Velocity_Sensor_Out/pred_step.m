function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
    X = sym('X',[15,1]);
    G = @(X) [1 0 -sin(X(5));
            0         cos(X(4))  sin(X(4))*cos(X(5));
            0   -sin(X(4)) cos(X(4))*cos(X(5))];
    R =@(X)[cos(X(6))*cos(X(5)), cos(X(4))*sin(X(5))*sin(X(6))-sin(X(6))*cos(X(4)), cos(X(6))*sin(X(5))*cos(X(4))+sin(X(6))*sin(X(4));
    sin(X(6))*cos(X(5)), sin(X(6))*sin(X(4))*sin(X(5))+cos(X(6))*cos(X(4)), sin(X(6))*sin(X(5))*cos(X(4))-cos(X(6))*sin(X(4));
    -sin(X(5)), cos(X(5))*sin(X(4)) cos(X(5))*cos(X(4))];
    
    Ginv = matlabFunction(simplify(inv(G(X))),'Vars',{X});
    process = @(X,U,N) [X(7:9);Ginv(X) * (U(1:3)-X(10:12)-N(1:3));[0;0;-9.81] + R(X)*(U(4:6)-X(13:15)-N(4:6));N(7:9);N(10:12)];
    N_g = [1;1;1]*0.5;
    N_a = [1;1;1]*0.5; 
    N_bias_g = [1;1;1]*0.5;
    N_bias_a = [1;1;1]*0.5;
    Q_t = diag([N_g; N_a; N_bias_g; N_bias_a]);
    x_aug = [uPrev;zeros(length(Q_t),1)];
    covar_aug = [[covarPrev,zeros(length(covarPrev),length(Q_t))];[zeros(length(Q_t),length(covarPrev)),Q_t]];
    alpha=1e-3;                                
    ki=1;                                      
    beta=2;  
    n = length(x_aug);
    u_t = [angVel;acc];
    lambda=alpha^2*(n+ki)-n;
    c=n+lambda; 
    w_0_m= lambda/c;
    w_i_m = 0.5/c; 
    w_i_c = w_i_m;
    w_0_c = w_0_m + (1-alpha^2+beta);
    C = chol(covar_aug,'lower');
    u_mat = zeros(n,2*length(C)+1);
    u_mat(:,1) = x_aug;
    inc = 1;
    for i = 1 : length(C)
        u_mat(:,inc+i) = x_aug + sqrt(c)* C(:,i);
        u_mat(:,inc+i+1)= x_aug - sqrt(c) * C(:,i);
        inc = inc+1;
    end
    x_mat = zeros(length(uPrev),length(u_mat));
    for j =1 : length(u_mat)
        x = u_mat(:,j);
        x_u = x(1:length(uPrev));
        x_n = x(length(uPrev)+1 : end);
        x_mat(:,j) = x_u + dt*process(x_u,u_t,x_n);
    end 

    for i= 1: length(x_mat)
        if i == 1 
            uEst = w_0_m * x_mat(:,i);
        else 
            uEst = w_i_m * x_mat(:,i)+ uEst;
        end
    end
    for i = 1:length(x_mat)
        if i == 1
            covarEst = w_0_c * (x_mat(:,i)- uEst) * (x_mat(:,i)- uEst)';
        else
            covarEst = w_i_c * (x_mat(:,i)- uEst) * (x_mat(:,i)- uEst)' + covarEst;
        end
    end

end 
     

        
        
            
    