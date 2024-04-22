function rrf = locoController(X, pf, t, gaitname, q, dq)
N = 10;
dt = 0.03;
gaitperiod = 0.06;
legs = 4;
rrf = zeros(12, 1);

[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);

% If current and future contacts are all 1, do standing PD
if all(currcontact == 1) && all(ftcontacts == 1)
    qDes = stand(t);
    dqDes = zeros(12, 1);
    u = jointPD(qDes, q, dqDes, dq);
    for ind = 1:legs
        start_ind = ind*3 - 2;
        end_ind = ind*3;
        rrf(start_ind:end_ind) = torque_to_force(u(start_ind:end_ind), q(start_ind:end_ind));
    end
% Else, run mpc
else
    % If leg starts swing phase, run swing control for it
    rrf_swing = zeros(12, 1);
    ftcontact_next = ftcontacts(1:legs);
    for ind = 1:legs
        if currcontact(ind) == 1 && ftcontact_next(ind) == 0
            %rrf(ind*3 - 2: ind*3) = SWING CONTROL for the specific leg
        end
    end

    rrf_mpc = mpc_simulink(X, pf, t, N, dt, ftcontacts);
    % Add robot reaction forces from swing and mpc
    rrf = rrf_swing + rrf_mpc;
end
end