function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
X = sym('X' ,[15 1]);
N = sym ('N' ,[1 12]);
U = sym( 'U', [1 6]);
 process = @(X,U,N)[X(7);X(8);X(9);- cos(X(5))*(N(1) - U(1) + X(10)) - sin(X(5))*(N(3) - U(3) + X(12));U(2) - N(2) - X(11) + (cos(X(5))*sin(X(4))*(N(3) - U(3) + X(12)))/cos(X(4)) - sin(X(4))*sin(X(5))*(N(1) - U(1) + X(10))/cos(X(4)); (sin(X(5))*(N(1) - U(1) + X(10)))/cos(X(4)) - (cos(X(5))*(N(3) - U(3) + X(12)))/cos(X(4));cos(X(4))*sin(X(6))*(N(5) - U(5) + X(14)) - (cos(X(6))*sin(X(5)) + cos(X(5))*sin(X(4))*sin(X(6)))*(N(6) - U(6) + X(15)) - (cos(X(5))*cos(X(6)) - sin(X(4))*sin(X(5))*sin(X(6)))*(N(4) - U(4) + X(13));- (cos(X(5))*sin(X(6)) + cos(X(6))*sin(X(4))*sin(X(5)))*(N(4) - U(4) + X(13)) - (sin(X(5))*sin(X(6)) - cos(X(5))*cos(X(6))*sin(X(4)))*(N(6) - U(6) + X(15)) - cos(X(4))*cos(X(6))*(N(5) - U(5) + X(14));cos(X(4))*sin(X(5))*(N(4) - U(4) + X(13)) - cos(X(4))*cos(X(5))*(N(6) - U(6) + X(15)) - sin(X(4))*(N(5) - U(5) + X(14)) - 981/100;N(7);N(8);N(9);N(10);N(11);N(12)];
 A_t = @(X,U,N)[ 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 1, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 1, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 1,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                         sin(X(5))*(N(1) - U(1) + X(10)) - cos(X(5))*(N(3) - U(3) + X(12)),                                                                                                                                                           0, 0, 0, 0,                       -cos(X(5)),    0,                      -sin(X(5)),                                           0,                0,                                           0;
 0, 0, 0, (cos(X(5))*(N(3) - U(3) + X(12)))/4 - (sin(X(5))*(N(1) - U(1) + X(10)))/4 + (cos((X(5)))*sin((X(4)))^2*(N(3)- U(3) + X(12)))/(4*cos(X(4))^2) - (sin(X(4))^2*sin(X(5))*(N(1) - U(1) + X(10)))/(4*cos(X(4))^2),                           - (cos(X(5))*sin(X(4))*(N(1) - U(1) + X(10)))/(4*cos(X(4))) - (sin(X(4))*sin(X(5))*(N(3) - U(3) + X(12)))/(4*cos(X(4))),                                                                                                                                                           0, 0, 0, 0, -(sin(X(4))*sin(X(5)))/(4*cos(X(4))), -1/4, (cos(X(5))*sin(X(4)))/(4*cos(X(4))),                                           0,                0,                                           0;
 0, 0, 0,                                                                         (sin(X(4))*sin(X(5))*(N(1) - U(1) + X(10)))/cos(X(4))^2 - (cos(X(5))*sin(X(4))*(N(3) - U(3) + X(12)))/cos(X(4))^2,                                                     (cos(X(5))*(N(1) - U(1) + X(10)))/cos(X(4)) + (sin(X(5))*(N(3) - U(3) + X(12)))/cos(X(4)),                                                                                                                                                           0, 0, 0, 0,                sin(X(5))/cos(X(4)),    0,              -cos(X(5))/cos(X(4)),                                           0,                0,                                           0;
 0, 0, 0,                                               cos(X(4))*sin(X(5))*sin(X(6))*(N(4) - U(4) + X(13)) - cos(X(4))*cos(X(5))*sin(X(6))*(N(6) - U(6) + X(15)) - sin(X(4))*sin(X(6))*(N(5) - U(5) + X(14)), (cos(X(6))*sin(X(5)) + cos(X(5))*sin(X(4))*sin(X(6)))*(N(4) - U(4) + X(13)) - (cos(X(5))*cos(X(6)) - sin(X(4))*sin(X(5))*sin(X(6)))*(N(6) - U(6) + X(15)), (cos(X(5))*sin(X(6)) + cos(X(6))*sin(X(4))*sin(X(5)))*(N(4) - U(4) + X(13)) + (sin(X(5))*sin(X(6)) - cos(X(5))*cos(X(6))*sin(X(4)))*(N(6) - U(6) + X(15)) + cos(X(4))*cos(X(6))*(N(5) - U(5) + X(14)), 0, 0, 0,                              0,    0,                             0,   sin(X(4))*sin(X(5))*sin(X(6)) - cos(X(5))*cos(X(6)),  cos(X(4))*sin(X(6)), - cos(X(6))*sin(X(5)) - cos(X(5))*sin(X(4))*sin(X(6));
 0, 0, 0,                                               cos(X(6))*sin(X(4))*(N(5) - U(5) + X(14)) + cos(X(4))*cos(X(5))*cos(X(6))*(N(6) - U(6) + X(15)) - cos(X(4))*cos(X(6))*sin(X(5))*(N(4) - U(4) + X(13)), (sin(X(5))*sin(X(6)) - cos(X(5))*cos(X(6))*sin(X(4)))*(N(4) - U(4) + X(13)) - (cos(X(5))*sin(X(6)) + cos(X(6))*sin(X(4))*sin(X(5)))*(N(6) - U(6) + X(15)), cos(X(4))*sin(X(6))*(N(5) - U(5) + X(14)) - (cos(X(6))*sin(X(5)) + cos(X(5))*sin(X(4))*sin(X(6)))*(N(6) - U(6) + X(15)) - (cos(X(5))*cos(X(6)) - sin(X(4))*sin(X(5))*sin(X(6)))*(N(4) - U(4) + X(13)), 0, 0, 0,                              0,    0,                             0, - cos(X(5))*sin(X(6)) - cos(X(6))*sin(X(4))*sin(X(5)), -cos(X(4))*cos(X(6)),   cos(X(5))*cos(X(6))*sin(X(4)) - sin(X(5))*sin(X(6));
 0, 0, 0,                                                                       cos(X(5))*sin(X(4))*(N(6) - U(6) + X(15)) - cos(X(4))*(N(5) - U(5) + X(14)) - sin(X(4))*sin(X(5))*(N(4) - U(4) + X(13)),                                                         cos(X(4))*cos(X(5))*(N(4) - U(4) + X(13)) + cos(X(4))*sin(X(5))*(N(6) - U(6) + X(15)),                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                             cos(X(4))*sin(X(5)),         -sin(X(4)),                            -cos(X(4))*cos(X(5));
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0;
 0, 0, 0,                                                                                                                                                                 0,                                                                                                                         0,                                                                                                                                                           0, 0, 0, 0,                              0,    0,                             0,                                           0,                0,                                           0];
U_t = @(X,U,N)[0,0,0,0,0,0,0,0,0,0,0,0;
                         0,  0,                         0,                                           0,                0,                                           0, 0, 0, 0, 0, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 0, 0, 0, 0, 0;
                   -cos(X(5)),  0,                  -sin(X(5)),                                           0,                0,                                           0, 0, 0, 0, 0, 0, 0;
 -(sin(X(4))*sin(X(5)))/cos(X(4)), -1, (cos(X(5))*sin(X(4)))/cos(X(4)),                                           0,                0,                                           0, 0, 0, 0, 0, 0, 0;
            sin(X(5))/cos(X(4)),  0,          -cos(X(5))/cos(X(4)),                                           0,                0,                                           0, 0, 0, 0, 0, 0, 0;
                          0,  0,                         0,   sin(X(4))*sin(X(5))*sin(X(6)) - cos(X(5))*cos(X(6)),  cos(X(4))*sin(X(6)), - cos(X(6))*sin(X(5)) - cos(X(5))*sin(X(4))*sin(X(6)), 0, 0, 0, 0, 0, 0;
                          0,  0,                         0, - cos(X(5))*sin(X(6)) - cos(X(6))*sin(X(4))*sin(X(5)), -cos(X(4))*cos(X(6)),   cos(X(5))*cos(X(6))*sin(X(4)) - sin(X(5))*sin(X(6)), 0, 0, 0, 0, 0, 0;
                          0,  0,                         0,                             cos(X(4))*sin(X(5)),         -sin(X(4)),                            -cos(X(4))*cos(X(5)), 0, 0, 0, 0, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 1, 0, 0, 0, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 1, 0, 0, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 0, 1, 0, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 0, 0, 1, 0, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 0, 0, 0, 1, 0;
                          0,  0,                         0,                                           0,                0,                                           0, 0, 0, 0, 0, 0, 1];
N_g = [1;1;1]*1; % for gyroscope
N_a = [1;1;1]*1; % for accelerometer
N_bias_g = [1;1;1]*0.001;  % for gyro bias
N_bias_a = [1;1;1]*0.001; % for accelerometer bias

a = [N_g;N_a;N_bias_g;N_bias_a];
Q_t = diag([N_g; N_a; N_bias_g; N_bias_a]);% Process noise
Q_d = Q_t * dt;
u_t = [angVel;acc];
F_t = eye(length(uPrev)) + A_t(uPrev, u_t, zeros(12,1)) * dt;
V_t = U_t(uPrev, u_t, zeros(12,1));


uEst = uPrev + dt * process(uPrev, u_t, zeros(12,1));
covarEst = F_t * covarPrev * F_t' + V_t * Q_d * V_t' ;                       
end