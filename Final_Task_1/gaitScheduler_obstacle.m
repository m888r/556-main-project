function [gaitname, landHeight, jumpVel, jumpAngle, walkVel, height, R_f, x_Q, pf_target,soarPD] = gaitScheduler_obstacle(X, pf, t)
persistent state2_start;
persistent state3_start;
persistent obstacleState;
% 1 - standing and trotting towards first obstacle
% 2 - trot in place, stop, jump over obstacle and trot towards final obstacle

if isempty(obstacleState)
    obstacleState = 1;
end

if isempty(state2_start)
    state2_start = 0;
end

if isempty(state3_start)
    state3_start = 0;
end
% landHeight return variable only needed for landing
landHeight = 0;
% trot until reaching certain X, do jump, then continue trotting
jumpAngle = 0;
jumpVel = [0;0;0];
soarPD = 0;
height = 0;
walkVel = 0;
R_f = 0.00005;
x_Q = [0, 30, 30, 30, 300, 150, 4, 4, 4, 1, 1, 1, 0]; %default

pf_target = [0.25;0.14;-0.1;
    0.25;-0.14;-0.1;
    -0.15;0.14;-0.1;
    -0.15;-0.14;-0.1];

if X(1) > 0.64%0.7 % right before first obstacle
    obstacleState = 2;
    
    if state2_start == 0 %first iteration that state2 starts
        state2_start = t;
    end
end

if X(1) > 5.58%5.68
    obstacleState = 3;
    
    if state3_start == 0  %first iteration that state3 starts
        state3_start = t;
    end
end

% if t < 0.65
%     gaitname = "standing";
%     walkVel = 0;
% elseif t < 0.8
%     gaitname = "jumpingg";
%     jumpVel = [2;0;3];
%     jumpAngle = -pi/4;
% elseif t < 1
%     gaitname = "soaringg";
% elseif t < 1.7
%     gaitname = "landingg";
%     landHeight = 0.4;
% elseif t < 1.85
%     gaitname = "jumpingg";
%     jumpVel = [2;0;2];
%     jumpAngle = 0;
% elseif t < 2.2
%     gaitname = "soaringg";
% elseif t < 2.6
%     gaitname = "landingg";
%     landHeight = 0.2;
% else
%     gaitname = "trotting";
% end


if obstacleState == 1
    % trot up to first obstacle
    if t < 0.65
        gaitname = "standing";
    else
        gaitname = "trotting";
        height = 0.25;
        % walkVel = 0.2;
        walkVel = speed_ramp(t, 0.65, 1, 0, 0.7);
        disp("first trot")
    end
    
elseif obstacleState == 2
    % jumping over first obstacle and trotting to last obstacle
    
    deccelerate = 0.9;
    trotInPlace = 0.15;
    standing = 0.3;
    jumping = 0.15;
    soaring = 0.35; %0.45
    landing = 0.35;
    jumping2 = 0.15;
    soaring2 = 0.4;
    landing2 = 0.6;
    
    if t-state2_start < deccelerate
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0.7+speed_ramp(t, 0, deccelerate, 0, -0.7);
        disp("second trot")
    elseif t-state2_start < deccelerate + trotInPlace
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0;
        disp("second trot")
    elseif t-state2_start < deccelerate + trotInPlace + standing
        gaitname = "standing";
    elseif t-state2_start < deccelerate + trotInPlace + standing + jumping
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = -pi/4; %-pi/4;
        R_f = 0.00005;
    elseif t-state2_start < deccelerate + trotInPlace + standing + jumping + soaring
        gaitname = "soaringg";
        soarPD = 1;
    elseif t-state2_start <  deccelerate + trotInPlace + standing + jumping + soaring + landing
        gaitname = "landingg";
        landHeight = 0.4;
    elseif t-state2_start < deccelerate + trotInPlace + standing + jumping + soaring + landing + jumping2
        gaitname = "jumpingg";
        jumpVel = [2;0;3];
        jumpAngle = 0;
        R_f = 0.00005;
    elseif t-state2_start < deccelerate + trotInPlace + standing + jumping + soaring + landing + jumping2 + soaring2
        gaitname = "soaringg";
        soarPD = 1;
    elseif t-state2_start < deccelerate + trotInPlace + standing + jumping + soaring + landing + jumping2 + soaring2 + landing2
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
        height = 0.25;
        walkVel = speed_ramp(t-state2_start, ...
            deccelerate + trotInPlace + standing + jumping + soaring + landing + jumping2 + soaring2 + landing2 ...
            , deccelerate + trotInPlace + standing + jumping + soaring + landing + jumping2 + soaring2 + landing2 + 1, 0, 1.5);
        % walkVel = 0.5;
    end
else
    % jumping over final obstacle (state 3)
    deccelerate = 1;
    trotInPlace = 0.6;
    standing = 0.3;
    jumping = 0.15;
    soaring = 0.75; %0.45
    soarPD_begin = 0.65;
    landing = 0.3;
    
    if t-state3_start < deccelerate
        gaitname = "trotting";
        height = 0.25;
        walkVel = 1.5 + speed_ramp(t-state3_start, 0, 1, 0, -1.5);
        t-state3_start;
        disp("second trot")
    elseif t-state3_start < deccelerate + trotInPlace
        gaitname = "trotting";
        x_Q = [0, 30, 30, 150, 300, 150, 4, 4, 4, 1, 1, 1, 0];
        % gaitname = "singleFt";
        height = 0.25;
        walkVel = 0;
        disp("second trot")
    elseif t-state3_start < deccelerate + trotInPlace + standing
        gaitname = "standing";
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping
        gaitname = "jumpingg";
        jumpVel = [3;0;7];
        jumpAngle = -pi; %-pi/2;
        % R_f = 0.00001
        R_f = 0.000005;
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping + soaring
        gaitname = "soaringg";
        
        soarPD = 1;
        
        pf_target = [-0.15;0.14;-0.0;
            -0.15;-0.14;-0.0;
            -0.25;0.14;-0.05;
            -0.25;-0.14;-0.05];
        
        
        if t-state3_start > deccelerate + trotInPlace + standing + jumping + soarPD_begin
            
            pf_target = [0.25;0.14;-0.25;
                0.25;-0.14;-0.25;
                -0.15;0.14;-0.3;
                -0.15;-0.14;-0.3];
        end
    elseif t-state3_start < deccelerate + trotInPlace + standing + jumping + soaring + landing
        gaitname = "landingg";
        landHeight = 0.2;
    else
        gaitname = "trotting";
        height = 0.25;
        walkVel = 0.5;
    end
end

end