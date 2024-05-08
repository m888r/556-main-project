function [rrf, rrf_world, pf_des_w, hips, pf_current_relbody, curr_pf_target,pf_target] = locoController_obstacle(X, pf, dpf, t, q, dq)
persistent last_mpc_run;
persistent rrf_mpc;
persistent prev_gait;
persistent prev_gait_startTime;

if isempty(prev_gait_startTime)
    prev_gait_startTime = 0;
end

if isempty(prev_gait)
    prev_gait = "standing";
end

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

Xd = [X(1); X(2); 0.2; zeros(3,1); zeros(3,1); zeros(3,1)];

% velTarget = speed_ramp(t, 0.65, 2, 0, 0);

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



pf_des_w = zeros(12, 1);
hips = zeros(12, 1);
pf_current_relbody = zeros(12, 1);
curr_pf_target = zeros(12, 1);

[gaitname, landHeight, jumpVel, jumpAngle, walkVel, height,R_f, walking_x_Q,pf_target,soarPD] = gaitScheduler_obstacle(X, pf, t);
gaitname
walking_Xd = [0; 0; height; 0; 0; 0; walkVel; 0; 0; zeros(3,1)]; %walking
% walking_x_Q = [0, 30, 30, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0]; %yaw 30->150
walking_x_Kstep = 0.1;


if isequal(gaitname, "jumpingg")
    walking_Xd = [0; 0; 0; 0; jumpAngle; 0; jumpVel; zeros(3,1)];
    walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
elseif isequal(gaitname, "soaringg")
    walking_Xd = [0; 0; 0; 0; 0; 0; 0; 0; 0; zeros(3,1)];
    walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
elseif isequal(gaitname, "singleFt")
    walking_Xd = [0; 0; 0; 0; 0; 0; 0.3; 0; 0; zeros(3,1)];
    walking_x_Q = [0, 30, 0, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0];
end
t_proj_gait = t;
if isequal(gaitname, "standing") && isequal(prev_gait, "trotting")
    t_proj_gait = 0;
end
[currcontact, ftcontacts] = project_gait(t_proj_gait,N,mpc_dt, gaitperiod, gaitname);
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
    
    % pf_target = [0.25;0.14;-0.1;
    %     0.25;-0.14;-0.1;
    %     -0.15;0.14;-0.1;
    %     -0.15;-0.14;-0.1];
    
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
    if soarPD == 1
        rrf = kP*(pf_target - pf_body) + kD*(-dpf_body); %in body orientation
    end
elseif isequal(gaitname, "landingg")
    Xd = [X(1); X(2); landHeight; zeros(3,1); zeros(3,1); zeros(3,1)];
    rrf = qp_simulink_landing(X, pf, t, Xd);
else
    
    Q_current = walking_x_Q;
    
    Kstep = walking_x_Kstep;
    
    % If leg starts swing phase, run swing control for it
    [rrf_swing, pf_des_w, hips, pf_current_relbody, curr_pf_target] = swing_control(X, walking_Xd(7:9), Kstep, pf, dpf, t, gaitperiod, currcontact, ftcontacts);
    
    
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

prev_gait = gaitname;
end