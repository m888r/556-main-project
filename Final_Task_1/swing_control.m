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

function rrf = swing_control(p_hip, x, v_des, K_step, pf, dpf, curr_t, T_stance)

    % update: only one foot at a time, don't need this for loop
    % TODO: implement this function using the pseudocode below
    % for loop looping through each leg that's in swing phase, use gait to
    % decide which ones and whether to skip it or go to the next leg
        % transform x(1:3) (com position) to find p_hip for the correct leg
        % use p_hip v_des K_step and T_stance to find the foot placement policy
        % for this leg with the foot_placement function
        % use pf_des from the foot_placement function and swing_cartesian_PD to
        % find the force needed from this leg
    % end for loop
    
    pf_des = foot_placement(p_hip, x, v_des, K_step, T_stance);
    kP = 10;
    kD = 0.1;
    rrf = swing_cartesian_PD(kP, kD, pf, dpf, curr_t, T_stance, pf_start, pf_des);
    
    % output to rrfs, 12x1 vector of all the robot reaction forces in world
    % frame, needs to be rotated to body frame before being sent out of the
    % function, can do it here or can do it outside (probably outside after
    % adding it to the world frame MPC forces)

end

function pf_des = foot_placement(p_hip, x, v_des, K_step, T_stance)
    
    v_com = x(7:9);
    pf_des = p_hip + (T_stance/2)*v_com + K_step*(v_com - v_des);
    
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
function [pf, dpf] = swing_trajectory(curr_t, T_stance, pf_start, pf_des)
    % implement a linearly interpolated trajectory from pf_start to pf_des
    t = curr_t / T_stance;
    P_height = 0.1; % height control point is at z=0.1
    P0 = pf_start;
    P1 = [(pf_des(1) - pf_start(1)) / 2, (pf_des(2) - pf_start(2)) / 2, P_height];
    P2 = pf_des;
    
    % 2nd order bezier: (1-t)^2 * P0 * 2 + 2*(1-t)*t*P1 + t^2*P2
    pf = P0.*(1-t)^2 + 2.*t.*P1.*(1-t) + P2.*(t^2);
    dpf = [0; 0; 0];
end