%{
    F_world = [F1_world, F2_world, F3_world, F4_world]
    x = robot states, need it for the COM angle
    q = joint angles

    torques = joint torques for all joints

    tau_i = -J_i'*R'*F_i
%}

function torques = legs_force_to_torque(F_world, x, q)
    ypr = x(4:6);
    bodyR = eul2rotm(ypr);

    torques = zeros(12, 1);
    for i = 1:4:12
        J_i = 
        torques(i) = -J_i'*bodyR'*F_world(i);
    end
end