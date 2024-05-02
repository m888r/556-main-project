%{
    Constructs a reference COM trajectory for MPC to use
    Every step, find all the future positions of the COM based on the 
    set angular and linear velocities in the Xd vector
    Xd = [pos; ang; vel; angvel]
    note: this MPC formulation uses Xd = [pos; ang; vel; angvel; 9.81]
%}

function [X_traj] = referenceTrajectory(Xd, X, N, dt)

    Xd_len = 13;
    X_traj = zeros(Xd_len*N, 1);

    for i = 1:N
        vel_new = Xd(7:9);
        pos_new = X(1:3) + (vel_new * dt * i);
        angvel_new = Xd(10:12);
        ang_new = X(4:6) + (angvel_new * dt * i);
        X_traj((i-1)*Xd_len+1:i*Xd_len) = [pos_new; ang_new; vel_new; angvel_new; 9.81];
    end

end