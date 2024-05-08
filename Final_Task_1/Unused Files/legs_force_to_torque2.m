% Converts force at toe in body frame to resulting joint torques
% Inputs: 
%   F - forces at toe for all 4 legs (12x1). Order is FL,FR,RL,RF
%   q - joints angles for all 4 legs (12x1 in radians). Order is FL,FR,RL,RF
function torques = legs_force_to_torque2(F, Q)
    
    torques = zeros(12,1);
    
    % iterate through legs. Order is FL,FR,RL,RF
    for i = 0:3s
        % Kinematic parameters
        a1 = 0.1; %m
        a2 = 0.2; %m
        a3 = 0.2; %m
        if i == 0 || i == 2
            a1 = -a1;
        end

        q = Q(3*i+1:3*i+3);
        f = F(3*i+1:3*i+3);
        %transformation from body frame to hip frame
        % R_B0 = eul2rotm([-pi/2,0,-pi/2]);
        R_B0 = [0 0 1;%for coordinate flipping, before q1 applied
            -1 0 0;
             0 -1 0];
        T_B0 = [R_B0,[0;0;0];zeros(1,3),1];
        
        %transformation from hip frame to thigh frame
        % R = eul2rotm([pi/2,0,pi/2]); 
        R = [0 0 1;%for coordinate flipping, before q1 applied
            1 0 0;
             0 1 0];
        R_T = [R,[0;0;0];zeros(1,3),1];
        T_01rot = [cos(q(1)) -sin(q(1)) 0 0; sin(q(1)) cos(q(1)) 0 0 ; 0 0 1 0; 0 0 0 1];
        T_01 = T_01rot * R_T; %combine coordinate flip and q1 application
        
        %transformation from thigh to knee frame
        T_12trans = eye(4);
        T_12trans(1:3,4) = [a2; 0; a1];
        T_12rot = [cos(q(2)) -sin(q(2)) 0 0; sin(q(2)) cos(q(2)) 0 0 ; 0 0 1 0; 0 0 0 1];
        T_12 = T_12rot * T_12trans;
        
        %transformation from knee to toe frame
        T_23trans = eye(4);
        T_23trans(1:3,4) = [a3; 0; 0];
        T_23rot = [cos(q(3)) -sin(q(3)) 0 0; sin(q(3)) cos(q(3)) 0 0 ; 0 0 1 0; 0 0 0 1];
        T_23 = T_23rot * T_23trans;
        
        %compute frame origins (all in body frame)
        P_EE = [0;0;0;1];
        O_0 = T_B0* P_EE;
        O_1 = T_B0* T_01 * P_EE;
        O_2 = T_B0* T_01 * T_12 * P_EE;
        O_3 = T_B0* T_01 * T_12 * T_23 * P_EE;
        
        %compute joint axes (all in body frame)
        z0 = [1;0;0];
        z1 = [0;-cos(q(1));-sin(q(1))];
        z2 = [0;-cos(q(1));-sin(q(1))];
        
        %compute Jacobian
        J = [cross(z0, O_3(1:3)-O_0(1:3)), ...
            cross(z1, O_3(1:3)-O_1(1:3)), ...
            cross(z2, O_3(1:3)-O_2(1:3))];
        
        t = J' * f;

        torques(3*i+1:3*i+3) = t;
    end
end
