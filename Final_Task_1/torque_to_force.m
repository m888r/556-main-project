% Converts force at toe in body frame to resulting joint torques
% Inputs: 
%   torques - joint torque (3x1)
%   q - joints angles (3x1 in radians)
function F = torque_to_force(torques, Q)
    offset_x = 0.1805;
    offset_y = -0.047;
    offset_z = 0.0;
    F = zeros(12,1);
    % Kinematic parameters
    a1 = 0.045; %m
    a2 = 0.2; %m
    a3 = 0.2; %m
    
    % iterate through legs. Order is FL,FR,RL,RF
    for i = 0:3

        legSign = 1;
        if i == 0 || i == 2
            legSign = -1;
        end
        
        frontBackSign = 1;
        if i == 2 || i == 3
            frontBackSign = -1;
        end

        q = Q(3*i+1:3*i+3);
        f = F(3*i+1:3*i+3);

        % s1 = sin(q(1));
        % s2 = sin(q(2));
        % s3 = sin(q(3));
        % 
        % c1 = cos(q(1));
        % c2 = cos(q(2));
        % c3 = cos(q(3));
        %transformation from body frame to hip frame
        % R_B0 = eul2rotm([-pi/2,0,-pi/2]);
        R_B0 = [0 0 1;%for coordinate flipping, before q1 applied
            -1 0 0;
             0 -1 0];
        % d_BodyToHip = [offset_x * frontBackSign; offset_y * legSign; offset_z];
        % T_B0 = [R_B0,d_BodyToHip;zeros(1,3),1];
        T_B0 = [R_B0,zeros(3, 1);zeros(1,3),1];
        
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
        T_12trans(1:3,4) = [a2; 0; a1*legSign];
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
        
        torque = torques(3*i +1 : 3*i + 3);

        F(3*i+1:3*i+3) = J * torque; %.* [1;-1;1];
    end
    
end
