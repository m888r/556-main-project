function u = jointPD(qDes, q, dqDes, dq)
%         kp_hip = 10;
%         kp_thigh = 10;
%         kp_calf = 10;
%         kd_hip = 1;
%         kd_thigh = 1;
%         kd_calf = 1;
    kp = 10;
    kd = 1;

    u = kp * (qDes - q) + kd * (dqDes - dq); 
end