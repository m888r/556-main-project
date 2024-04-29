clear; clc;

N = 10;
mpc_dt = 0.03;
gaitperiod = 0.09;
legs = 4;
Xd = [0; 0; 0.2; zeros(3,1); zeros(3,1); zeros(3,1)];
walking_Xd = [0; 0; 0.2; 0; 0; 0; zeros(3,1); zeros(3,1)];


t = 0.9;
gaitname = "trotting";
[currcontact, ftcontacts] = project_gait(t,N,mpc_dt, gaitperiod, gaitname)
t = 1.2;
[currcontact, ftcontacts] = project_gait(t,N,mpc_dt, gaitperiod, gaitname)


% rrf = swing_control(x, v_des, K_step, pf, dpf, t, T_stance, curr_contact, ftcontact_next)
%rrf_swing = swing_control(X, walking_Xd(4:6), 0.1, pf, dpf, t, gaitperiod, currcontact, ftcontacts);