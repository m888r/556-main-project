clear
F_FL = [0;0;1];
F_FR = [0;0;1];
F_RL = [0;0;1];
F_RF = [0;0;1];
F = [F_FL; F_FR; F_RL; F_RF];
Q = [0;0;pi/2;0;0;0;0;0;0;0;0;0];
x = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% torques = legs_force_to_torque2(F, Q)
[torques_new, pf] = legs_force_to_torque_fk(F, Q, x)

