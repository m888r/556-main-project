clc;
%clear;
close all;
% FL (Hip, Thigh, Calf)
% FR (...)
% RL (...)
% RR (...)
leg = ["FL", "FR", "RL", "RR"];

% Joint Torque Plots
figure("Name", "Leg Joint Torque Plots");
us = squeeze(out.u.Data);
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.u.Time, us(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.u.Time, us(3*n - 1, :));
    % Calf
    plot(out.u.Time, us(3*n, :));
    xlabel('t [s]'); ylabel('\tau [Nm]'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n));
    ylim([-40 68])
end

% Joint Angle Plots
figure("Name", "Leg Joint Angle Plots");
qs = transpose(squeeze(out.q.Data));
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.q.Time, qs(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.q.Time, qs(3*n - 1, :));
    % Calf
    plot(out.q.Time, qs(3*n, :));
    xlabel('t [s]'); ylabel('\theta [rad]'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n))
    ylim([-3 2])
end

% Joint Velocity Plots
figure("Name", "Leg Joint Velocity Plots");
dqs = transpose(squeeze(out.dq.Data));
for n = 1:4
    subplot(2, 2, n);
    % Hip
    plot(out.dq.Time, dqs(3*n - 2, :));
    hold on;
    % Thigh
    plot(out.dq.Time, dqs(3*n - 1, :));
    % Calf
    plot(out.dq.Time, dqs(3*n, :));
    xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('Hip', 'Thigh', 'Calf');
    title(leg(n));
    ylim([-2.8 6])
end