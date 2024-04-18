% Converts force at toe in body frame to resulting joint torques
% Inputs: 
%   torques - joint torque (3x1)
%   q - joints angles (3x1 in radians)
function F = torque_to_force(torques, q)
    % Kinematic parameters
    a1 = 0.1; %m
    a2 = 0.2; %m
    a3 = 0.2; %m

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    %transformation from body frame to hip frame
    R_B0 = eul2rotm([-pi/2,0,-pi/2]);
    T_B0 = [R_B0,[0;0;0];zeros(1,3),1];
    
    %transformation from hip frame to thigh frame
    R = eul2rotm([pi/2,0,pi/2]); %for coordinate flipping, before q1 applied
    R_T = [R,[0;0;0];zeros(1,3),1];
    T_01rot = [cos(q1) -sin(q1) 0 0; sin(q1) cos(q1) 0 0 ; 0 0 1 0; 0 0 0 1];
    T_01 = T_01rot * R_T; %combine coordinate flip and q1 application
    
    %transformation from thigh to knee frame
    T_12trans = eye(4);
    T_12trans(1:3,4) = [a2; 0; a1];
    T_12rot = [cos(q2) -sin(q2) 0 0; sin(q2) cos(q2) 0 0 ; 0 0 1 0; 0 0 0 1];
    T_12 = T_12rot * T_12trans;
    
    %transformation from knee to toe frame
    T_23trans = eye(4);
    T_23trans(1:3,4) = [a3; 0; 0];
    T_23rot = [cos(q3) -sin(q3) 0 0; sin(q3) cos(q3) 0 0 ; 0 0 1 0; 0 0 0 1];
    T_23 = T_23rot * T_23trans;
    
    %compute frame origins (all in body frame)
    P_EE = [0;0;0;1];
    O_0 = T_B0* P_EE;
    O_1 = T_B0* T_01 * P_EE;
    O_2 = T_B0* T_01 * T_12 * P_EE;
    O_3 = T_B0* T_01 * T_12 * T_23 * P_EE;
    
    %compute joint axes (all in body frame)
    z0 = [1;0;0];
    z1 = [0;-cos(q1);-sin(q1)];
    z2 = [0;-cos(q1);-sin(q1)];
    
    %compute Jacobian
    J = [cross(z0, O_3(1:3)-O_0(1:3)), ...
        cross(z1, O_3(1:3)-O_1(1:3)), ...
        cross(z2, O_3(1:3)-O_2(1:3))];
    
    F = J * torques;
end
