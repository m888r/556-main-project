function [p_hip_all] = get_hip_pos_world(x)
    p_com = x(1:3);

    % Hip offset wrt COM
    offset_x = 0.1805;
    offset_y = 0.047;

    % playing around with offset_y to account for hip motor thickness
    % offset_y = 0.13;
    offset_z = 0.0;

    % transform to all the hips for each leg and output hip positions as:
    % FL, FR, RL, RR
    % only rotate to world frame, no offset
    R = eul2rotm(x(4:6)');
    
    hip_FL = R * [offset_x; offset_y; offset_z];
    hip_FR = R * [offset_x; -offset_y; offset_z];
    hip_RL = R * [-offset_x; offset_y; offset_z];
    hip_RR = R * [-offset_x; -offset_y; offset_z];

    % still relative to body position, just rotated into world frame

    p_hip_all = [hip_FL; hip_FR; hip_RL; hip_RR];

end