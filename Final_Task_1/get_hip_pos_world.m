function [p_hip] = get_hip_pos_world(x)
    p_com = x(1:3);

    % Hip offset wrt COM
    offset_x = 0.1805;
    offset_y = 0.047;
    offset_z = 0.0;

    % transform to all the hips for each leg and output hip positions as:
    % FL, FR, RL, RR
    % in world frame? --> H0b
    R = eul2rotm(x(4:6));
    
    
end