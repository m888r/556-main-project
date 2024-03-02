clc;
clear;
close all;

figure("Name", "Leg Plots");


% FL
subplot(2, 2, 1);
title("FL");
plot(out.u.Time, out.u.Data(1, 1, :));
hold on;
plot(out.u.Time, out.u.Data(2, 1, :));
plot(out.u.Time, out.u.Data(3, 1, :));

% plot(out.)

% FR
subplot(2, 2, 2);
title("FR");

% RL
subplot(2, 2, 3);
title("RL");

% RR
subplot(2, 2, 4);
title("RR");
