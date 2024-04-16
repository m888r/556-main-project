clc;
close all;

leg = ["Front Left", "Front Right", "Rear Left", "Rear Right"];
% Joint Torque Plots
figure("Name", "Leg Joint Torque Plots");
torques = squeeze(out.torque.Data);
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.torque.Time, torques(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.torque.Time, torques(3*n - 1, :));
    % Calf
    plot(out.torque.Time, torques(3*n, :));
    xlabel('t (s)'); ylabel('\tau (Nm)'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n));
    ylim([-50 95])
end

% Joint Angle Plots
figure("Name", "Leg Joint Angle Plots");
angles = transpose(squeeze(out.angle.Data));
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.angle.Time, angles(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.angle.Time, angles(3*n - 1, :));
    % Calf
    plot(out.angle.Time, angles(3*n, :));
    xlabel('t [s]'); ylabel('\theta (rad)'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n))
    ylim([-3 2])
end

% Joint Velocity Plots
figure("Name", "Leg Joint Velocity Plots");
velocities = transpose(squeeze(out.velocity.Data));
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.velocity.Time, velocities(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.velocity.Time, velocities(3*n - 1, :));
    % Calf
    plot(out.velocity.Time, velocities(3*n, :));
    xlabel('t (s)'); ylabel('\omega (rad/s)'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n));
    ylim([-2.8 6])
end