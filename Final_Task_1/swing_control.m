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

function [rrf, pf_des_w, hips_rel_w, pf, curr_pf_target] = swing_control(x, v_des, K_step, pf_w, dpf, t, T_stance, curr_contact, ftcontact_next)

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
com = x(1:3);
hips_rel_w = zeros(12, 1);
for ind = 1:4
    hips_rel_w(ind*3 - 2:ind*3) = hips(ind*3 - 2:ind*3) + com;
end
% display(hips);

% getting foot positions (in world frame orientation) with respect to com position
com = x(1:3);
pf = zeros(12, 1);
for ind = 1:4
    pf(ind*3 - 2:ind*3) = pf_w(ind*3 - 2:ind*3) - com;
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

ypr = x(4:6);
Rot = eul2rotm(ypr');
rrf_unrotated = zeros(12, 1);
rrf = zeros(12, 1);
curr_pf_target = zeros(12, 1);
for i = 1:4
    if ftcontact_next(i) == 0
        kP = 3500;
        kD = 10;
        curr_t = localSwingTimer(i);
        [rrf_unrotated(i*3-2:i*3), curr_pf_target(i*3-2:i*3)] = swing_cartesian_PD(kP, kD, pf(i*3-2:i*3), dpf(i*3-2:i*3), curr_t, T_stance, pf_start(i*3-2:i*3), pf_des(i*3-2:i*3), x);
        curr_pf_target(i*3-2:i*3) = curr_pf_target(i*3-2:i*3) + com;
        rrf(i*3-2:i*3) = Rot'*rrf_unrotated(i*3-2:i*3);
        %rotating rrf into body frame
    end
end

% Troubleshooting: display desired foot positions in world frame, wrt world
% origin
pf_des_w = zeros(12, 1);
for ind = 1:4
    pf_des_w(ind*3 - 2:ind*3) = pf_des(ind*3 - 2:ind*3) + com;
end

% output to rrfs, 12x1 vector of all the robot reaction forces in world
% frame, needs to be rotated to body frame before being sent out of the
% function, can do it here or can do it outside (probably outside after
% adding it to the world frame MPC forces)


end


% change this for stairs, and change it for turning
function pf_des = foot_placement(p_hip, x, v_des, K_step, T_stance)

v_com = [x(7); x(8); 0];
pf_des = [p_hip(1); p_hip(2); 0 - x(3)];% + (T_stance/2)*v_com + K_step*(v_com - v_des);
%display(pf_des);

end

%{
    Do swing control for a single leg
    Does PD for the leg by taking a target pf from the swing trajectory
    bezier and outputs an rrf for that leg to do
    pf_start and pf_des are the starting and ending positions of the swing
    trajectory
%}

function [rrf, curr_pf_target] = swing_cartesian_PD(kP, kD, curr_pf, curr_dpf, curr_t, T_stance, pf_start, pf_des, x)

% calculate the target position and end effector velocity
[curr_pf_target, curr_dpf_target] = swing_trajectory(curr_t, T_stance, pf_start, pf_des, x);

rrf = kP*(curr_pf_target - curr_pf) + kD*(curr_dpf_target - curr_dpf);

end

%{
    pf = [x, y, z]
    dpf = gradient[pf] or whatever the trajectory wants, motion profile etc
    curr_t = current time, s, should be [0, T_stance]
    pf_start = foot position when switching from stance to swing
    pf_des = foot position at end of trajectory [x, y, z]
%}
function [curr_pf_target, curr_dpf_target] = swing_trajectory(curr_t, T_stance, pf_start, pf_des, x)
% implement a linearly interpolated trajectory from pf_start to pf_des
t = curr_t / T_stance;
P_height = 0.06; % height control point is at z=0.05
P0 = pf_start;
P1 = [(pf_des(1) - pf_start(1)) / 2; (pf_des(2) - pf_start(2)) / 2; P_height - x(3)];
P2 = pf_des;



% 2nd order bezier: (1-t)^2 * P0 * 2 + 2*(1-t)*t*P1 + t^2*P2
curr_pf_target = (P2-P0)*t + P0; 
curr_pf_target(3) = P0(3)*(1-t)^2 + 2*t*P1(3)*(1-t) + P2(3)*(t^2);
% curr_pf_target = P0.*(1-t)^2 + 2.*t.*P1.*(1-t) + P2.*(t^2);
curr_dpf_target = [0; 0; 0];
end