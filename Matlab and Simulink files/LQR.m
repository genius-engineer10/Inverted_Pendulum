clc;clear;
M = 2.4;    
m = 0.23;   
g = 9.8;   
l = 1;   
I = (m*l^2)/3;
L = 0.5;
Ksu = 1;
E_up = 2*m*g*l;
n = 0.388;

D_eq = 1 - (3 * m) / (4 * (M + m));

% state = [x; theta; x_dot; theta_dot]
A = [0,   0,                  1,   0;
     0,   0,                  0,   1;
     0,   (-3*m*g)/(4*D_eq*(M+m)), 0,   0;
     0,   (3*g)/(4*l*D_eq),    0,   0];

B = [0;
     0;
     1/(D_eq*(M+m));
     -3/(4*l*D_eq*(M+m))];


Q = diag([100, 1, 50, 1]);
R = 0.1;                   

% Calculate the LQR gain matrix K
K = lqr(A, B, Q, R)

%%
Acl = A - B*K;
poles = eig(Acl)