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

