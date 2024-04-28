clear; clc;

X = [zeros(3, 1); zeros(3,1); zeros(3,1); zeros(3, 1)];
pf = [1; 1; 0; 1; -1; 0; -1; 1; 0; -1; -1; 0];
Xd = [0; 0; 0.3; zeros(3,1); zeros(3,1); zeros(3, 1)];

gaitname = 'standing';
gaitperiod = 0.06;

%mpc(X, pf, Xd, gait) 

%mpc_simulink(X, pf);

t = 1;
%qp_simulink(X, pf, t)

N = 10;
dt = 0.03;
[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);
mpc_simulink(X, pf, t, N, dt, ftcontacts)