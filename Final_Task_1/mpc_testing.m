clear; clc;

X = [zeros(3, 1); zeros(3,1); zeros(3,1); zeros(3, 1)];
pf = [1; 1; 0; 1; -1; 0; -1; 1; 0; -1; -1; 0];
Xd = [0; 0; 0.3; zeros(3,1); zeros(3,1); zeros(3, 1)];

gait = 'standing';

%mpc(X, pf, Xd, gait) 

%mpc_simulink(X, pf);

t = 1;
qp_simulink(X, pf, t)