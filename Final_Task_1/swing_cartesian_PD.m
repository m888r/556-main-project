%{
    Do swing control for a single leg
    Does PD for the leg by taking a target pf from the swing trajectory
    bezier and outputs an rrf for that leg to do
    pf_start and pf_des are the starting and ending positions of the swing
    trajectory
%}

function rrf = swing_cartesian_PD(kP, kD, curr_t, T_stance, pf_start, pf_des, curr_pf, curr_dpf)

    rrf = zeros(12, 1);
    curr_pf = curr_pf;
    
    % calculate the target position and end effector velocity
    [curr_pf_target, curr_dpf_target] = swing_trajectory(curr_t, T_stance, pf_start, pf_des);

    rrf = kP*(curr_pf_target - curr_pf) + kD*(dpf_target - curr_dpf_target);
    
end