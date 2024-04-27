function [rrf,pdTorque] = locoController(X, pf, t, gaitname, q, dq)
N = 10;
dt = 0.03;
gaitperiod = 0.06;
legs = 4;
rrf = zeros(12, 1);
pdTorque = zeros(12,1);
localSwingTimer = zeros(4,1);

[currcontact, ftcontacts] = project_gait(t,N,dt, gaitperiod, gaitname);
% If current and future contacts are all 1, do standing PD
if all(currcontact == 1) && all(ftcontacts == 1)
    % Joint PD
    % qDes = stand(t);
    % dqDes = zeros(12, 1);
    % u = jointPD(qDes, q, dqDes, dq);
    % pdTorque = u;
    % rrf = torque_to_force(u, q);

    % Cartesian PD?
    % R = eul2rotm(X(4:6)');
    % Kp_front = -30;
    % Kp_back = -30;
    % Kd_front = -10;
    % Kd_back = -10;
    % Fbody_front = R'*[0;0;Kp_front * (0.2 - X(3)) + Kd_front * (0 - X(9))]
    % Fbody_back = R'*[0;0;Kp_back * (0.2 - X(3)) + Kd_back * (0 - X(9))]
    % rrf = [Fbody_front;Fbody_front;Fbody_back;Fbody_back];

    % QP
    rrf = qp_simulink(X, pf, t);
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