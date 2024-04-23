function u = jointPD(qDes, q, dqDes, dq)
% FL (H, T, C)
% FR (...), RL (...), RF (...)

%         kp_hip = 10;
%         kp_thigh = 10;
%         kp_calf = 10;
%         kd_hip = 1;
%         kd_thigh = 1;
%         kd_calf = 1;
    kp = 60*6;
    kd = 10*10;

    u = kp .* (qDes - q) + kd .* (dqDes - dq); 
end