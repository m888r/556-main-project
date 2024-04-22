function rrf = locoController(X, pf, t, gaitname, q, dq)
N = 10;
dt = 0.03;
gaitperiod = 0.06;
forces = 4;
rrf = zeros(forces*3, 1);

[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);

% If current and future contacts are all 1, do standing PD
if all(currcontact == 1) && all(ftcontacts == 1)
    qDes = stand(t);
    dqDes = zeros(12, 1);
    u = jointPD(qDes, q, dqDes, dq);
    for ind = 1:forces
        rrf(ind*3 - 2:ind*3) = torque_to_force(u(), q())
    end
    
else
% Else, run mpc
    ftcontact_next = ftcontacts(1:forces);

    rrf = mpc_simulink(X, pf, t, N, dt);
end





rrf = [];
end