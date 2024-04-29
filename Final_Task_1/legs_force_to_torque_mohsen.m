% Converts force at toe in body frame to resulting joint torques
% Inputs:
%   F - forces at toe for all 4 legs (12x1). Order is FL,FR,RL,RR
%   q - joints angles for all 4 legs (12x1 in radians). Order is
%   FL,FR,RL,RR
function torques = legs_force_to_torque_mohsen(F, Q)
torques = zeros(12,1);
J = zeros(3, 3);
for ind = 1:4
    q = Q(3*ind-2:3*ind);
    f = F(3*ind-2:3*ind);
    J = computeLegJacobian(q,ind);
    torques(3*ind-2:3*ind) = pinv(J)*f;
end


%torques = legs_force_to_torque(F, Q);
end
