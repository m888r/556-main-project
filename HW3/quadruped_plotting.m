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
end

% Joint Angle Plots


% Joint Velocity Plots
