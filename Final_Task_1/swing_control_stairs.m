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

function [rrf, pf_des_w, hips, pf, curr_pf_target] = swing_control_stairs(x, v_des, K_step, pf_w, dpf, t, T_stance, curr_contact, ftcontact_next,stairStart)

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
        pf_des(ind*3 - 2:ind*3) = foot_placement(hips(ind*3 - 2:ind*3), x, v_des, K_step, T_stance,stairStart);
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
        % kP = 3000;
        % kD = 10;
        
        kP = 5000; %work
        kD = 20;
        
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
function pf_des = foot_placement(p_hip, x, v_des, K_step, T_stance,stairStart)

v_com = [x(7); x(8); 0];
pf_des = [p_hip(1); p_hip(2); 0 - x(3)] + (T_stance/2)*v_com + K_step*(v_des - v_com);
%display(pf_des);

% get desired foot positions in world frame, wrt world origin
com = x(1:3);
pf_des_w = pf_des + com;

stairWidth = 0.2; % width of stairs from the side (so in x direction)
stairHeight = 0.1;
amountDeadband = 0.3; %ratio of stairWidth that shall not get foot placed on
%  (on each side, aka beginning of stairWidth and end)

%find stair bounds to avoid edge or corner
stairBoundsX = []; % 5x2 matrix of most extreme allowed X positions at each step
for step = 0:4
    bound1 = stairStart + step*stairWidth + amountDeadband*stairWidth;
    bound2 = stairStart + (step+1)*stairWidth - amountDeadband*stairWidth;
    stairBoundsX = [stairBoundsX; [bound1, bound2]];
end

%determine stair foot is going to
stair = -1;
if pf_des_w(1) <= stairStart && pf_des_w(1) > stairStart-0.1 %stair 0 represents being close to base of staircase
    stair = 0;
elseif pf_des_w(1) > stairStart && pf_des_w(1) <= stairStart+stairWidth
    stair = 1;
elseif pf_des_w(1) > stairStart+stairWidth && pf_des_w(1) <= stairStart+stairWidth*2
    stair = 2;
elseif pf_des_w(1) > stairStart+stairWidth*2 && pf_des_w(1) <= stairStart+stairWidth*3
    stair = 3;
elseif pf_des_w(1) > stairStart+stairWidth*3 && pf_des_w(1) <= stairStart+stairWidth*4
    stair = 4;
elseif pf_des_w(1) > stairStart+stairWidth*4
    stair = 5;
end

%determine desired foot position based on stair
if stair==0 % stair 0 represents being close to base of staircase
    pf_des(1) = stairStart-0.1 - com(1) - (T_stance)*v_com(3);
elseif ismember(stair,[1,2,3,4])
    if pf_des_w(1) < stairBoundsX(stair,1)
        pf_des(1) = stairBoundsX(stair,1) - com(1);
    elseif pf_des_w(1) > stairBoundsX(stair,2)
        pf_des(1) = stairBoundsX(stair,2) - com(1) - (T_stance)*v_com(1);
    end
    pf_des(3) = stairHeight*stair - com(3) - (T_stance)*v_com(3);
    com(3);
elseif stair==5
    pf_des(3) = 0.5 - com(3) - (T_stance)*v_com(3);
end

end

% should generalize by just using the hip speed
% function pf_des = foot_placement_turning(p_hip, v_hip, x, K_step, T_stance)
%     v_com = [x(7); x(8); 0];
%     angvel_com = [x(10); x(11); x(12)]; % yaw pitch roll
%     pf_des(1) = [p_hip(1); p_hip(2); 0 - x(3)] + (T_stance/2)*v_com + K_step*(dyaw_des - x(10));

% end

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
t = curr_t / (T_stance-0.03); %subtract 0.03 because for some reason trajectory target gets started late
P_height = 0.15; % height control point

P0 = pf_start;
P1 = [pf_start(1); pf_start(2); pf_start(3) + P_height];
P2 = [pf_des(1); pf_des(2); pf_start(3) + P_height];
P3 = pf_des;

% % 3rd Order Bezier
curr_pf_target = (1-t)^3 * P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + P3*t^3;
curr_dpf_target = [0; 0; 0];

%experimenting with triangle path
% t_up = 0.5;
% if t < t_up
%     curr_pf_target = (P1-P0)*t/t_up + P0;
% else
%     curr_pf_target = (P3-P1)*(t-t_up)/(1-t_up) + P1;
% end

end