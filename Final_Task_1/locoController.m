function [rrf, rrf_world, pf_des_w, hips, pf_current_relbody, curr_pf_target] = locoController(X, pf, dpf, t, q, dq)
persistent last_mpc_run;
persistent rrf_mpc;

rrf_world = zeros(12, 1);

if isempty(rrf_mpc)
    rrf_mpc = zeros(12, 1);
end

if isempty(last_mpc_run)
    last_mpc_run = 0;
end

N = 10;
mpc_dt = 0.03;
gaitperiod = 0.12;
legs = 4;
rrf = zeros(12, 1);
grf_mpc = zeros(12, 1);

Xd = [0; 0; 0.25; zeros(3,1); zeros(3,1); zeros(3,1)];

velTarget = speed_ramp(t, 0.65, 2, 0, 1.7);
walking_Xd = [0; 0; 0.25; 0; 0; 0; velTarget; 0; 0; zeros(3,1)];
pf_des_w = zeros(12, 1);
hips = zeros(12, 1);
pf_current_relbody = zeros(12, 1);
curr_pf_target = zeros(12, 1);

gaitname = gaitScheduler(X, pf, t);

walking_x_Q = [0, 30, 30, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
walking_x_Kstep = 0.1;

[currcontact, ftcontacts] = project_gait(t,N,mpc_dt, gaitperiod, gaitname);
% If current and future contacts are all 1, do standing PD
if isequal(gaitname, "standing")
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
    rrf = qp_simulink(X, pf, t, Xd);
    % Else, run mpc
else

    Q_current = walking_x_Q;
    Kstep = walking_x_Kstep;

    % If leg starts swing phase, run swing control for it
    [rrf_swing, pf_des_w, hips, pf_current_relbody, curr_pf_target] = swing_control(X, walking_Xd(7:9), Kstep, pf, dpf, t, gaitperiod, currcontact, ftcontacts);


    if (t - last_mpc_run) >= mpc_dt
        [rrf_mpc, grf_mpc] = mpc_simulink(X, walking_Xd, pf, t, N, mpc_dt, Q_current, ftcontacts);
        % [rrf_mpc, grf_mpc] = mpc_mohsen_allocate(X, walking_Xd, pf, t, N, mpc_dt, ftcontacts);
        last_mpc_run = t;
    end
    
    % Add robot reaction forces from swing and mpc
    rrf = rrf_swing + rrf_mpc;
    
end

for ind = 1:4
    rrf_world(ind*3-2:ind*3) = eul2rotm(X(4:6)') * rrf(ind*3-2:ind*3);
end

end