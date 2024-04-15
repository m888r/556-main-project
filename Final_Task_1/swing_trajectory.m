%{
    pf = [x, y, z]
    dpf = gradient[pf] or whatever the trajectory wants, motion profile etc
    curr_t = current time, s, should be [0, T_stance]
    pf_start = foot position when switching from stance to swing
    pf_des = foot position at end of trajectory [x, y, z]
%}
function [pf, dpf] = swing_trajectory(curr_t, pf_start, pf_des)
    % implement a linearly interpolated trajectory from pf_start to pf_des
    
end