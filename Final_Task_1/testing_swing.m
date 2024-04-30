clear; clc;

N = 10;
mpc_dt = 0.03;
gaitperiod = 0.09;
legs = 4;
Xd = [0; 0; 0.2; zeros(3,1); zeros(3,1); zeros(3,1)];
walking_Xd = [0; 0; 0.2; 0.4; 0; 0; zeros(3,1); zeros(3,1)];
v_des = walking_Xd(4:6);
K_step = 0.1;
T_stance = gaitperiod;

t = 0.8;
gaitname = "trotting";
% [currcontact, ftcontacts] = project_gait(t,N,mpc_dt, gaitperiod, gaitname)
curr_contact = [1;1;1;1];
ft_contacts = [1; 0; 0; 1; 1; 0; 0; 1; 1; 0; 0; 1; 0; 1; 1; 0; 0; 1; 1; 0; 0; 1; 1; 0; 1; 0; 0; 1; 1; 0; 0; 1; 1; 0; 0; 1; 0; 1; 1; 0];
ftcontact_next = ft_contacts(1:4);
% t = 1.2;
% [currcontact, ftcontacts] = project_gait(t,N,mpc_dt, gaitperiod, gaitname)


x = [-0.0063; 0.0002; 0.2235; 0.0034; 0.0778; -0.0107; -0.0028; 0.0000; 0.0007; 0.0002; 0.0020; 0.0015];
pf = [0.1898; 0.1320; 0.0099; 0.1901; -0.1321; 0.0099; -0.1663; 0.1321; 0.0099; -0.1662; -0.1322; 0.0099];
dpf = 1.0e-03 * [-0.1850; 0.1379; 0.1178; -0.1431; -0.1198; 0.1256; -0.1039; 0.2695; 0.5505; -0.0765; -0.1567; 0.5850];


pf_start = zeros(12, 1);

pf_des = zeros(12, 1);

localSwingTimer = zeros(4, 1);

swingTimerStartTimes = zeros(4, 1);


hips = get_hip_pos_world(x)

for ind = 1:4
    if curr_contact(ind) == 1 && ftcontact_next(ind) == 0
        swingTimerStartTimes(ind) = t;
        pf_start(ind*3 - 2:ind*3) = pf(ind*3 - 2:ind*3);
        pf_des(ind*3 - 2:ind*3) = foot_placement(hips(ind*3 - 2:ind*3), x, v_des, K_step, T_stance);
    end
    localSwingTimer(ind) = t - swingTimerStartTimes(ind);
end

rrf = swing_control(x, v_des, K_step, pf, dpf, t, T_stance, curr_contact, ftcontact_next)
%rrf_swing = swing_control(X, walking_Xd(4:6), 0.1, pf, dpf, t, gaitperiod, currcontact, ftcontacts);

%{
    THIS FUNCTION is what you call in mpc to output the forces, you add
    this to the mpc forces and you're good to go

    NOTE: this doesn't work for turning yet because it only uses COM
    velocity for foot placement, we need to use COM angular velocity too
    for foot placement if we want to do turning too

    gait is a 4x1 vector of which legs are doing this stuff
    eg [1, 0, 1, 0] means FL swing, FR stance, RL swing, RR stance

    v_des is the desired com velocity [dx, dy, dz] usually dz = 0
    pf_all is all the foot positions
    dpf_all is all the foot velocities
    x is the robot state
    K_step is the proportional controller to make the robot step further
    forward or further backwards if it's lagging or leading the desired
    velocity v_des

    curr_t is the current time, but it should hopefully reset somehow
    depending on when gait was changed, so that it's always [0,
    T_stance]... --> decide whether to reset this in MPC or in this
    function using a persistent thing that tracks whether the gait has
    changed (reset when gait changes, which is what we do in MPC rn)
    UPDATE: it's being done in lococontroller.m
    
    
%}

function rrf = swing_control(x, v_des, K_step, pf, dpf, t, T_stance, curr_contact, ftcontact_next)

persistent localSwingTimer;
persistent swingTimerStartTimes;
persistent pf_des;
persistent pf_start;

if isempty(pf_start)
    pf_start = zeros(12, 1);
end

if isempty(pf_des)
    pf_des = zeros(12, 1);
end

if isempty(localSwingTimer)
    localSwingTimer = zeros(4, 1);
end

if isempty(swingTimerStartTimes)
    swingTimerStartTimes = zeros(4, 1);
end

hips = get_hip_pos_world(x);
if t < 0.9
    disp(hips);
    
end

for ind = 1:4
    if curr_contact(ind) == 1 && ftcontact_next(ind) == 0
        swingTimerStartTimes(ind) = t;
        pf_start(ind*3 - 2:ind*3) = pf(ind*3 - 2:ind*3);
        pf_des(ind*3 - 2:ind*3) = foot_placement(hips(ind*3 - 2:ind*3), x, v_des, K_step, T_stance);
    end
    localSwingTimer(ind) = t - swingTimerStartTimes(ind);
end

% update: only one foot at a time, don't need this for loop or the 12x1
% rrf, instead do 3x1 rrf
% TODO: implement this function using the pseudocode below
% for loop looping through each leg that's in swing phase, use gait to
% decide which ones and whether to skip it or go to the next leg
% transform x(1:3) (com position) to find p_hip for the correct leg
% use p_hip v_des K_step and T_stance to find the foot placement policy
% for this leg with the foot_placement function
% use pf_des from the foot_placement function and swing_cartesian_PD to
% find the force needed from this leg
% end for loop

rrf = zeros(12, 1);
for i = 1:4
    if ftcontact_next(i) == 0
        kP = 100;
        kD = 20;
        curr_t = localSwingTimer(i);
        rrf(i*3-2:i*3) = swing_cartesian_PD(kP, kD, pf(i*3-2:i*3), dpf(i*3-2:i*3), curr_t, T_stance, pf_start(i*3-2:i*3), pf_des(i*3-2:i*3));
    end
end

% output to rrfs, 12x1 vector of all the robot reaction forces in world
% frame, needs to be rotated to body frame before being sent out of the
% function, can do it here or can do it outside (probably outside after
% adding it to the world frame MPC forces)
end


% change this for stairs, and change it for turning
function pf_des = foot_placement(p_hip, x, v_des, K_step, T_stance)

v_com = [x(7); x(8); 0];
pf_des = [p_hip(1); p_hip(2); 0] + (T_stance/2)*v_com + K_step*(v_com - v_des);

end

%{
    Do swing control for a single leg
    Does PD for the leg by taking a target pf from the swing trajectory
    bezier and outputs an rrf for that leg to do
    pf_start and pf_des are the starting and ending positions of the swing
    trajectory
%}

function rrf = swing_cartesian_PD(kP, kD, curr_pf, curr_dpf, curr_t, T_stance, pf_start, pf_des)

% calculate the target position and end effector velocity
[curr_pf_target, curr_dpf_target] = swing_trajectory(curr_t, T_stance, pf_start, pf_des);

rrf = kP*(curr_pf_target - curr_pf) + kD*(curr_dpf_target - curr_dpf);

end

%{
    pf = [x, y, z]
    dpf = gradient[pf] or whatever the trajectory wants, motion profile etc
    curr_t = current time, s, should be [0, T_stance]
    pf_start = foot position when switching from stance to swing
    pf_des = foot position at end of trajectory [x, y, z]
%}
function [curr_pf_target, curr_dpf_target] = swing_trajectory(curr_t, T_stance, pf_start, pf_des)
% implement a linearly interpolated trajectory from pf_start to pf_des
t = curr_t / T_stance;
P_height = 0.1; % height control point is at z=0.1
P0 = pf_start;
P1 = [(pf_des(1) - pf_start(1)) / 2; (pf_des(2) - pf_start(2)) / 2; P_height];
P2 = pf_des;



% 2nd order bezier: (1-t)^2 * P0 * 2 + 2*(1-t)*t*P1 + t^2*P2
curr_pf_target = P0.*(1-t)^2 + 2.*t.*P1.*(1-t) + P2.*(t^2);
curr_dpf_target = [0; 0; 0];
end
