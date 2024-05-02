clear; clc;

%X = [1; 2; 0.3; deg2rad(2); 2; 1; zeros(3,1); zeros(3, 1)];
%pf = [1; 1; 0; 1; -1; 0; -1; 1; 0; -1; -1; 0];
X = [-0.02; 0; 0.25; zeros(3, 1); zeros(3, 1); zeros(3, 1)];
pf = [0.19; 0.13; 0.01; 0.19; -0.13; 0.01; -0.17; 0.14; 0.01; -0.17; -0.14; 0.01];
Xd = [0; 0; 0.3; zeros(3,1); zeros(3,1); zeros(3, 1)];

gaitname = 'trotting';
gaitperiod = 0.12;

%mpc(X, pf, Xd, gait) 

%mpc_simulink(X, pf);

t = 0;
%qp_simulink(X, pf, t)

N = 10;
dt = 0.03;
[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);

mpc_t = 0.67;
mpc_rrf = mpc_simulink(X, Xd, pf, mpc_t, N,dt, ftcontacts);
mpc_mohsen = mpc_mohsen(X, Xd, pf, mpc_t, N, dt, ftcontacts);
mpc_mohsen_all = mpc_mohsen_allocate(X, Xd, pf, mpc_t, N, dt, ftcontacts);
isequal(mpc_rrf, mpc_mohsen)
isequal(mpc_mohsen_all, mpc_mohsen)