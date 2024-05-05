function [rrf, rrf_world, pf_des_w, hips, pf_current_relbody, curr_pf_target,pf_target] = locoController_obstacle(X, pf, dpf, t, q, dq)
persistent last_mpc_run;
persistent rrf_mpc;

rrf_world = zeros(12, 1);

pf_target = zeros(12, 1);

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

Xd = [0; 0; 0.2; zeros(3,1); zeros(3,1); zeros(3,1)];

velTarget = speed_ramp(t, 0.65, 2, 0, 0);

% Get zTarget and pitchTarget
% obs1Start = 1; %where base of obs1 starts wrt world origin
% zStart = 0.28;
% xCurr = X(1);

% pitchTarget = 0;
% if xCurr < obs1Start-0.4
%     zTarget = zStart;
% elseif xCurr > obs1Start-0.4 && xCurr < obs1Start+0.1
%     zTarget = zStart + 0.07;
%     pitchTarget = -pi/8;
% elseif xCurr > obs1Start+0.1 && xCurr < obs1Start+0.8
%     zTarget = zStart + 0.15;
%     pitchTarget = 0;
%     velTarget = 0.1;
% else
%     zTarget = zStart; %add total staircase height
%     % pitchTarget = 0;
% end
% walking_Xd = [0; 0; zTarget; 0; pitchTarget; 0; velTarget; 0; 0; zeros(3,1)];
walking_Xd = [0; 0; 0.2; 0; 0; 0; 0; 0; 3; zeros(3,1)];


pf_des_w = zeros(12, 1);
hips = zeros(12, 1);
pf_current_relbody = zeros(12, 1);
curr_pf_target = zeros(12, 1);

gaitname = gaitScheduler_obstacle(X, pf, t);

if isequal(gaitname, "jumpingg")
    walking_Xd = [0; 0; 0; 0; -pi/4; 0; 2; 0; 3; zeros(3,1)];
    walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
elseif isequal(gaitname, "soaringg")
    walking_Xd = [0; 0; 0; 0; 0; 0; 0; 0; 0; zeros(3,1)];
    walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
end

walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
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
    % Else, run
elseif isequal(gaitname, "soaringg")
    % keep the robot joints in a good state for landing
    kP = 3000;
    kD = 20;
    % hips = get_hip_pos_world(X);
    % pf_target = hips; % pf_target is desired foot position relative to body frame (position and orientation)
    % pf_target(3) = -0.3;
    % pf_target(6) = -0.3;
    % pf_target(9) = -0.3;
    % pf_target(12) = -0.3;
    pf_target = [0.2;0.14;-0.2;
        0.2;-0.14;-0.2;
        -0.15;0.14;-0.25;
        -0.15;-0.14;-0.25];
    
    com = X(1:3);
    vcom = X(7:9);
    pf_body = zeros(12,1);
    for ind = 1:4
        pf_body(ind*3-2:ind*3) = eul2rotm(X(4:6)')' * (pf(ind*3-2:ind*3) - com);
    end
    dpf_body = zeros(12,1);
    for ind = 1:4
        dpf_body(ind*3-2:ind*3) = eul2rotm(X(4:6)')' * (dpf(ind*3-2:ind*3) - vcom);
    end
    
    rrf = kP*(pf_target - pf_body) + kD*(-dpf_body); %in body orientation
    
else
    
    Q_current = walking_x_Q;
    R_f = 0.00005;
    Kstep = walking_x_Kstep;
    
    % If leg starts swing phase, run swing control for it
    [rrf_swing, pf_des_w, hips, pf_current_relbody, curr_pf_target] = swing_control_obstacle(X, walking_Xd(7:9), Kstep, pf, dpf, t, gaitperiod, currcontact, ftcontacts);
    
    
    if (t - last_mpc_run) >= mpc_dt
        [rrf_mpc, grf_mpc] = mpc_simulink(X, walking_Xd, pf, t, N, mpc_dt, Q_current, R_f, ftcontacts);
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