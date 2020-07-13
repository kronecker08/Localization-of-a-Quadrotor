function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
R_t = diag([ones(3,1)*1e-3;ones(3,1)*1e-3]);
C = [eye(3) zeros(3) zeros(3) zeros(3) zeros(3);zeros(3) eye(3) zeros(3) zeros(3) zeros(3)];
K_t = (covarEst*C')*(inv(C*covarEst*C' + R_t));
uCurr = uEst + K_t*(z_t - C * uEst);
covar_curr = covarEst - K_t * C * covarEst;    
end