function rrf = locoController(X, pf, t, gaitname, q, dq)
N = 10;
dt = 0.03;
gaitperiod = 0.06;
legs = 4;
rrf = zeros(12, 1);

localSwingTimer = zeros(4,1);

[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);
% If current and future contacts are all 1, do standing PD
if all(currcontact == 1) && all(ftcontacts == 1)
    qDes = stand(t);
    dqDes = zeros(12, 1);
    u = jointPD(qDes, q, dqDes, dq);
    rrf = torque_to_force(u, q);
    disp('standing')
% Else, run mpc
else
    disp('mpc')
    % If leg starts swing phase, run swing control for it
    rrf_swing = zeros(12, 1);
    ftcontact_next = ftcontacts(1:legs);
    for ind = 1:legs
        if ftcontact_next(ind) == 0
            if currcontact == 1
            % reset local timer to 0
                localSwingTimer(ind) = 0;
            end
            % iterate swing with local timer
            % [pf, dpf] = swing_trajectory()
            % rrf_swing(ind*3 - 2: ind*3) = swing_control()
        end
    end

    rrf_mpc = mpc_simulink(X, pf, t, N, dt, ftcontacts);
    % Add robot reaction forces from swing and mpc
    rrf = rrf_swing + rrf_mpc;
end
end